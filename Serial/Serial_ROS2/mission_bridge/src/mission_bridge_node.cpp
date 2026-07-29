/*
 * @file    mission_bridge_node.cpp
 * @brief   车端串口协议栈与 ROS2 桥接节点
 * @author  Wanone111
 * @note    该节点通过 ESP8266 串口透传与车端通信，负责应用层消息解析、
 *          (session, seq) 去重、session 隔离、心跳链路监测、PAYLOAD_RELEASE 应答，
 *          并将车辆状态以 std_msgs 话题发布到 ROS2。
 */

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "mission_bridge/protocol.hpp"
#include "mission_bridge/serial_port.hpp"

extern "C" {
#include "serial.h"
}

namespace mission_bridge
{

/*
 * @brief  车端串口协议栈桥接节点类。
 * @note   读线程阻塞读串口并逐字节喂 Serial_ParseByte，定时器线程负责链路监测与周期重发。
 */
class MissionBridgeNode : public rclcpp::Node
{
public:
    /*
     * @brief  构造函数，声明参数、创建发布者与定时器并启动读线程。
     */
    MissionBridgeNode() : Node("mission_bridge")
    {
        port_ = declare_parameter<std::string>("serial.port", "/dev/ttyUSB0");
        baudrate_ = declare_parameter<int>("serial.baudrate", 115200);
        reconnect_interval_ms_ = declare_parameter<int>("serial.reconnect_interval_ms", 2000);
        heartbeat_period_ms_ = declare_parameter<int>("protocol.heartbeat_period_ms", 1000);
        heartbeat_timeout_beats_ = declare_parameter<int>("protocol.heartbeat_timeout_beats", 3);
        dedup_window_size_ = declare_parameter<int>("protocol.dedup_window", 32);
        if (dedup_window_size_ < 1)
        {
            dedup_window_size_ = 1;
        }

        start_pub_ = create_publisher<std_msgs::msg::UInt32>("/mission/start", 10);
        progress_pub_ = create_publisher<std_msgs::msg::Float32>("/car/progress", 10);
        speed_pub_ = create_publisher<std_msgs::msg::Float32>("/car/speed", 10);
        phase_pub_ = create_publisher<std_msgs::msg::UInt8>("/car/phase", 10);
        state_pub_ = create_publisher<std_msgs::msg::String>("/mission/state", 10);
        fault_pub_ = create_publisher<std_msgs::msg::String>("/mission/fault", 10);

        Serial_ParserInit(&parser_);
        dedup_window_.assign((size_t)dedup_window_size_, {0U, 0U});

        link_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                        std::bind(&MissionBridgeNode::checkLinkTimeout, this));
        state_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&MissionBridgeNode::onStateTimer, this));
        phase_timer_ = create_wall_timer(std::chrono::milliseconds(heartbeat_period_ms_),
                                         std::bind(&MissionBridgeNode::onPhaseTimer, this));

        running_ = true;
        read_thread_ = std::thread(&MissionBridgeNode::readLoop, this);

        RCLCPP_INFO(get_logger(),
                    "mission_bridge 已启动: port=%s baudrate=%d hb_period=%dms timeout=%d beats dedup=%d",
                    port_.c_str(), baudrate_, heartbeat_period_ms_, heartbeat_timeout_beats_,
                    dedup_window_size_);
    }

    /*
     * @brief  析构函数，通知读线程退出、join 并关闭串口。
     */
    ~MissionBridgeNode() override
    {
        running_ = false;
        if (read_thread_.joinable())
        {
            read_thread_.join();
        }
        serial_.close();
    }

private:
    /*
     * @brief  可中断的毫秒级睡眠。
     * @param  ms: 睡眠时长，单位 ms。
     * @retval None
     * @note   按 50ms 分片睡眠以便及时响应退出标志。
     */
    void sleepInterruptible(int ms)
    {
        int remaining = ms;
        while (running_ && remaining > 0)
        {
            int slice = (remaining > 50) ? 50 : remaining;
            std::this_thread::sleep_for(std::chrono::milliseconds(slice));
            remaining -= slice;
        }
    }

    /*
     * @brief  读线程主循环：维护串口连接，逐字节解析并处理成帧。
     * @retval None
     * @note   打开失败按 reconnect_interval_ms 重试（节流告警）；读错误发布 SERIAL_LOST 并重连。
     */
    void readLoop()
    {
        uint8_t buf[256];

        while (running_ && rclcpp::ok())
        {
            if (!serial_.isOpen())
            {
                if (!serial_.open(port_, baudrate_))
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                         "串口 %s 打开失败，%d ms 后重试",
                                         port_.c_str(), reconnect_interval_ms_);
                    sleepInterruptible(reconnect_interval_ms_);
                    continue;
                }
                RCLCPP_INFO(get_logger(), "串口 %s @%d 已打开", port_.c_str(), baudrate_);
                Serial_ParserInit(&parser_);
            }

            ssize_t n = serial_.read(buf, sizeof(buf));
            if (n < 0)
            {
                if (errno == EINTR)
                {
                    continue;
                }
                RCLCPP_ERROR(get_logger(), "串口读错误: %s，准备重连", std::strerror(errno));
                publishFault("SERIAL_LOST: read error, reconnecting");
                serial_.close();
                sleepInterruptible(reconnect_interval_ms_);
                continue;
            }

            for (ssize_t i = 0; i < n; i++)
            {
                LogStatus_t status = Serial_ParseByte(&parser_, buf[i], &rx_frame_);
                if (status == LOG_STATUS_OK)
                {
                    if (!running_ || !rclcpp::ok())
                    {
                        break;
                    }
                    try
                    {
                        handleFrame(rx_frame_);
                    }
                    catch (const std::exception &e)
                    {
                        /* 关闭过程中发布可能抛异常，直接退出读线程 */
                        RCLCPP_DEBUG(get_logger(), "处理帧时异常: %s", e.what());
                        if (!rclcpp::ok())
                        {
                            break;
                        }
                    }
                }
                else if (status == LOG_STATUS_SERIAL_CRC_ERROR ||
                         status == LOG_STATUS_SERIAL_TAIL_ERROR ||
                         status == LOG_STATUS_SERIAL_LENGTH_ERROR)
                {
                    /* CRC 等帧错误只计数，帧头噪声不计 */
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    crc_err_++;
                }
            }
        }
    }

    /*
     * @brief  处理一帧解析完成的应用层消息。
     * @param  frame: 解析完成的传输层帧。
     * @retval None
     * @note   依次执行 session 隔离、(session, seq) 去重和按类型分发。
     */
    void handleFrame(const Serial &frame)
    {
        MessageHeader header;
        if (!DecodeHeader(frame.data, frame.length, header))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "收到非法应用层消息，len=%u，已丢弃", frame.length);
            return;
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);

            /* session 隔离 */
            if (header.type == MsgType::START)
            {
                if (session_set_ && header.session_id != active_session_)
                {
                    RCLCPP_WARN(get_logger(), "START 帧 session %u 替换当前 session %u，去重窗口已重置",
                                header.session_id, active_session_);
                }
                if (!session_set_ || header.session_id != active_session_)
                {
                    resetDedup();
                }
                active_session_ = header.session_id;
                session_set_ = true;
            }
            else
            {
                if (!session_set_)
                {
                    active_session_ = header.session_id;
                    session_set_ = true;
                    RCLCPP_INFO(get_logger(), "从首个有效帧采纳 session=%u", active_session_);
                }
                else if (header.session_id != active_session_)
                {
                    session_drop_++;
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                         "丢弃 session 不匹配的帧: type=0x%02X session=%u (active=%u)",
                                         (unsigned)header.type, header.session_id, active_session_);
                    return;
                }
            }

            /* (session, seq) 去重 */
            if (isDuplicate(header.session_id, header.seq))
            {
                dup_++;
                return;
            }
            pushDedup(header.session_id, header.seq);
            rx_++;
        }

        dispatch(header, frame.data + 3U, frame.length - 3U);
    }

    /*
     * @brief  按消息类型分发处理。
     * @param  header: 已解析的消息公共头。
     * @param  payload: 负载数据指针。
     * @param  len: 负载长度。
     * @retval None
     */
    void dispatch(const MessageHeader &header, const uint8_t *payload, size_t len)
    {
        switch (header.type)
        {
            case MsgType::START:
            {
                uint32_t mission_id;
                if (!DecodeStart(payload, len, mission_id))
                {
                    return;
                }
                RCLCPP_INFO(get_logger(), "收到 START: mission_id=%u session=%u",
                            mission_id, header.session_id);
                std_msgs::msg::UInt32 msg;
                msg.data = mission_id;
                start_pub_->publish(msg);
                break;
            }

            case MsgType::CAR_STATE:
            {
                uint8_t phase;
                if (!DecodeCarState(payload, len, phase))
                {
                    return;
                }
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    phase_ = phase;
                }
                std_msgs::msg::UInt8 msg;
                msg.data = phase;
                phase_pub_->publish(msg);
                break;
            }

            case MsgType::CAR_PROGRESS:
            {
                CarProgressPayload progress;
                if (!DecodeCarProgress(payload, len, progress))
                {
                    return;
                }
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    progress_m_ = progress.mileage;
                    speed_mps_ = progress.speed;
                    progress_seen_ = true;
                }
                publishProgress(progress.mileage, progress.speed);
                break;
            }

            case MsgType::HEARTBEAT:
            {
                HeartbeatPayload heartbeat;
                if (!DecodeHeartbeat(payload, len, heartbeat))
                {
                    return;
                }
                std::lock_guard<std::mutex> lock(state_mutex_);
                hb_seen_ = true;
                last_hb_time_ = std::chrono::steady_clock::now();
                hb_seq_ = heartbeat.hb_seq;
                if (!link_up_)
                {
                    link_up_ = true;
                    RCLCPP_INFO(get_logger(), "链路恢复: link up, hb_seq=%u", hb_seq_);
                }
                break;
            }

            case MsgType::PAYLOAD_RELEASE:
            {
                uint8_t release_id;
                if (!DecodePayloadRelease(payload, len, release_id))
                {
                    return;
                }
                RCLCPP_INFO(get_logger(), "收到 PAYLOAD_RELEASE: release_id=%u seq=%u",
                            release_id, header.seq);
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    last_release_id_ = (int)release_id;
                }
                sendPayloadAck(header.seq);
                break;
            }

            case MsgType::PAYLOAD_ACK:
                /* 本节点只发 ACK，收到对端 ACK 仅记录 */
                RCLCPP_DEBUG(get_logger(), "收到 PAYLOAD_ACK: session=%u seq=%u",
                             header.session_id, header.seq);
                break;

            case MsgType::MISSION_ABORT:
            {
                uint8_t reason;
                if (!DecodeMissionAbort(payload, len, reason))
                {
                    return;
                }
                RCLCPP_WARN(get_logger(), "收到 MISSION_ABORT: reason=%u", reason);
                char text[64];
                std::snprintf(text, sizeof(text), "MISSION_ABORT: reason=%u", reason);
                publishFault(text);
                break;
            }

            default:
                break;
        }
    }

    /*
     * @brief  查询 (session, seq) 是否已在去重窗口中。
     * @param  session_id: 会话编号。
     * @param  seq: 帧序号。
     * @retval true 表示为重复帧。
     * @note   调用方须已持有 state_mutex_。
     */
    bool isDuplicate(uint8_t session_id, uint8_t seq)
    {
        for (size_t i = 0; i < dedup_count_; i++)
        {
            if (dedup_window_[i].first == session_id && dedup_window_[i].second == seq)
            {
                return true;
            }
        }
        return false;
    }

    /*
     * @brief  将 (session, seq) 写入去重环形窗口。
     * @param  session_id: 会话编号。
     * @param  seq: 帧序号。
     * @retval None
     * @note   调用方须已持有 state_mutex_。
     */
    void pushDedup(uint8_t session_id, uint8_t seq)
    {
        dedup_window_[dedup_head_] = {session_id, seq};
        dedup_head_ = (dedup_head_ + 1U) % dedup_window_.size();
        if (dedup_count_ < dedup_window_.size())
        {
            dedup_count_++;
        }
    }

    /*
     * @brief  重置去重窗口。
     * @retval None
     * @note   调用方须已持有 state_mutex_。
     */
    void resetDedup()
    {
        dedup_head_ = 0U;
        dedup_count_ = 0U;
    }

    /*
     * @brief  发送 PAYLOAD_ACK 应答帧。
     * @param  acked_seq: 被应答帧的 SEQ。
     * @retval None
     * @note   acked_type 固定为 0x05（PAYLOAD_RELEASE），result 固定为 0x00 成功；串口写加互斥锁。
     */
    void sendPayloadAck(uint8_t acked_seq)
    {
        uint8_t payload[3] = {(uint8_t)MsgType::PAYLOAD_RELEASE, acked_seq, 0x00};
        std::lock_guard<std::mutex> lock(write_mutex_);
        std::vector<uint8_t> frame = BuildMessage(MsgType::PAYLOAD_ACK, active_session_,
                                                  tx_seq_++, payload, sizeof(payload));
        if (frame.empty())
        {
            return;
        }
        ssize_t written = serial_.write(frame.data(), frame.size());
        if (written < 0 || (size_t)written != frame.size())
        {
            RCLCPP_WARN(get_logger(), "PAYLOAD_ACK 发送失败: written=%zd", written);
        }
    }

    /*
     * @brief  事件发布故障消息。
     * @param  text: 故障描述文本。
     * @retval None
     */
    void publishFault(const std::string &text)
    {
        if (!rclcpp::ok())
        {
            return;
        }
        std_msgs::msg::String msg;
        msg.data = text;
        fault_pub_->publish(msg);
    }

    /*
     * @brief  发布里程与速度。
     * @param  mileage: 里程，单位 m。
     * @param  speed: 速度，单位 m/s。
     * @retval None
     */
    void publishProgress(float mileage, float speed)
    {
        std_msgs::msg::Float32 progress_msg;
        progress_msg.data = mileage;
        progress_pub_->publish(progress_msg);
        std_msgs::msg::Float32 speed_msg;
        speed_msg.data = speed;
        speed_pub_->publish(speed_msg);
    }

    /*
     * @brief  心跳超时检查定时器回调（100ms）。
     * @retval None
     * @note   超过 beats*period 未收到 HEARTBEAT 则 link 置 down 并发布 LINK_DOWN fault，
     *         只在 up->down 跳变时发布一次；链路判定只看 HEARTBEAT 帧。
     */
    void checkLinkTimeout()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!link_up_ || !hb_seen_)
        {
            return;
        }

        auto timeout = std::chrono::milliseconds((int64_t)heartbeat_timeout_beats_ * heartbeat_period_ms_);
        if (std::chrono::steady_clock::now() - last_hb_time_ > timeout)
        {
            link_up_ = false;
            char text[96];
            std::snprintf(text, sizeof(text), "LINK_DOWN: heartbeat timeout after %d beats",
                          heartbeat_timeout_beats_);
            RCLCPP_WARN(get_logger(), "%s", text);
            publishFault(text);
        }
    }

    /*
     * @brief  状态发布定时器回调（10Hz）。
     * @retval None
     * @note   发布 /mission/state 紧凑单行状态，并重发最近一次里程/速度保证 >=10Hz。
     */
    void onStateTimer()
    {
        bool link_up;
        bool session_set;
        uint8_t active_session;
        uint8_t phase;
        float progress_m;
        float speed_mps;
        uint16_t hb_seq;
        uint64_t rx;
        uint64_t dup;
        uint64_t crc_err;
        bool progress_seen;
        int last_release_id;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            link_up = link_up_;
            session_set = session_set_;
            active_session = active_session_;
            phase = phase_;
            progress_m = progress_m_;
            speed_mps = speed_mps_;
            hb_seq = hb_seq_;
            rx = rx_;
            dup = dup_;
            crc_err = crc_err_;
            progress_seen = progress_seen_;
            last_release_id = last_release_id_;
        }

        char session_text[8];
        if (session_set)
        {
            std::snprintf(session_text, sizeof(session_text), "%u", active_session);
        }
        else
        {
            std::snprintf(session_text, sizeof(session_text), "-");
        }

        char release_text[8];
        if (last_release_id >= 0)
        {
            std::snprintf(release_text, sizeof(release_text), "%d", last_release_id);
        }
        else
        {
            std::snprintf(release_text, sizeof(release_text), "-");
        }

        char text[192];
        std::snprintf(text, sizeof(text),
                      "link=%s session=%s phase=%s progress_m=%.2f speed_mps=%.2f hb_seq=%u "
                      "rx=%" PRIu64 " dup=%" PRIu64 " crc_err=%" PRIu64 " release=%s",
                      link_up ? "up" : "down", session_text, PhaseToString(phase),
                      (double)progress_m, (double)speed_mps, hb_seq, rx, dup, crc_err,
                      release_text);

        std_msgs::msg::String msg;
        msg.data = text;
        state_pub_->publish(msg);

        if (progress_seen)
        {
            publishProgress(progress_m, speed_mps);
        }
    }

    /*
     * @brief  phase 重发定时器回调（心跳周期）。
     * @retval None
     * @note   按心跳周期定时重发当前 phase，与事件发布互补。
     */
    void onPhaseTimer()
    {
        std_msgs::msg::UInt8 msg;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            msg.data = phase_;
        }
        phase_pub_->publish(msg);
    }

    /* 参数 */
    std::string port_;
    int baudrate_;
    int reconnect_interval_ms_;
    int heartbeat_period_ms_;
    int heartbeat_timeout_beats_;
    int dedup_window_size_;

    /* 发布者 */
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr start_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr progress_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr phase_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_pub_;

    /* 定时器 */
    rclcpp::TimerBase::SharedPtr link_timer_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr phase_timer_;

    /* 串口与读线程 */
    SerialPort serial_;
    std::thread read_thread_;
    std::atomic<bool> running_{false};
    SerialParser parser_;
    Serial rx_frame_;

    /* 互斥锁：state_mutex_ 保护共享状态，write_mutex_ 保护串口写 */
    std::mutex state_mutex_;
    std::mutex write_mutex_;

    /* session 隔离 */
    bool session_set_{false};
    uint8_t active_session_{0};

    /* (session, seq) 去重环形窗口 */
    std::vector<std::pair<uint8_t, uint8_t>> dedup_window_;
    size_t dedup_head_{0};
    size_t dedup_count_{0};

    /* 链路状态 */
    bool link_up_{false};
    bool hb_seen_{false};
    std::chrono::steady_clock::time_point last_hb_time_{};
    uint16_t hb_seq_{0};

    /* 最近一次车辆状态 */
    uint8_t phase_{PHASE_IDLE};
    float progress_m_{0.0F};
    float speed_mps_{0.0F};
    bool progress_seen_{false};
    int last_release_id_{-1};

    /* 发送序号与统计计数 */
    uint8_t tx_seq_{0};
    uint64_t rx_{0};
    uint64_t dup_{0};
    uint64_t crc_err_{0};
    uint64_t session_drop_{0};
};

} // namespace mission_bridge

/*
 * @brief  节点入口函数。
 * @param  argc: 参数个数。
 * @param  argv: 参数列表。
 * @retval 进程退出码。
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mission_bridge::MissionBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
