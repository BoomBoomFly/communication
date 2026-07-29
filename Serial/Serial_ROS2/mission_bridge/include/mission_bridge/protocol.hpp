/*
 * @file    protocol.hpp
 * @brief   应用层协议编解码头文件
 * @author  Wanone111
 * @note    该文件定义了 mission_bridge 应用层消息类型、负载编解码接口和线帧组帧接口。
 *          应用层消息承载于串口传输帧 DATA 区，前 3 字节为公共头 MSG_TYPE | SESSION_ID | SEQ，
 *          随后为各类型负载，多字节字段全部采用小端序。
 */

#ifndef MISSION_BRIDGE_PROTOCOL_HPP
#define MISSION_BRIDGE_PROTOCOL_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

namespace mission_bridge
{

/*
 * @brief  应用层消息类型。
 * @note   该枚举对应 DATA 区公共头中的 MSG_TYPE 字段。
 */
enum class MsgType : uint8_t
{
    START = 0x01,           /* 任务开始，负载为 mission_id u32 */
    CAR_STATE = 0x02,       /* 车辆阶段状态，负载为 phase u8 */
    CAR_PROGRESS = 0x03,    /* 车辆进度，负载为 mileage f32 + speed f32 */
    HEARTBEAT = 0x04,       /* 心跳，负载为 hb_seq u16 + time_ms u32 + link_status u8 */
    PAYLOAD_RELEASE = 0x05, /* 投放指令，负载为 release_id u8 */
    PAYLOAD_ACK = 0x06,     /* 投放应答，负载为 acked_type u8 + acked_seq u8 + result u8 */
    MISSION_ABORT = 0x07,   /* 任务中止，负载为 reason u8 */
};

/*
 * @brief  车辆阶段状态值。
 * @note   该枚举对应 CAR_STATE 消息负载中的 phase 字段。
 */
enum CarPhase : uint8_t
{
    PHASE_IDLE = 0,     /* 空闲 */
    PHASE_STARTUP = 1,  /* 启动 */
    PHASE_RUNNING = 2,  /* 运行 */
    PHASE_ARRIVE_A = 3, /* 到达 A 点 */
    PHASE_ARRIVE_B = 4, /* 到达 B 点 */
    PHASE_ARRIVE_C = 5, /* 到达 C 点 */
    PHASE_ARRIVE_D = 6, /* 到达 D 点 */
    PHASE_COMPLETED = 7,/* 完成 */
};

/*
 * @brief  应用层消息公共头。
 * @note   所有消息 DATA 区前 3 字节均为 MSG_TYPE | SESSION_ID | SEQ。
 */
struct MessageHeader
{
    MsgType type;      /* 消息类型 */
    uint8_t session_id;/* 会话编号 */
    uint8_t seq;       /* 帧序号 */
};

/*
 * @brief  HEARTBEAT 消息负载。
 */
struct HeartbeatPayload
{
    uint16_t hb_seq;    /* 心跳序号 */
    uint32_t time_ms;   /* 对端时间戳，单位 ms */
    uint8_t link_status;/* 对端链路状态 */
};

/*
 * @brief  CAR_PROGRESS 消息负载。
 */
struct CarProgressPayload
{
    float mileage; /* 里程，单位 m */
    float speed;   /* 速度，单位 m/s */
};

/*
 * @brief  将阶段状态值转换为字符串。
 * @param  phase: 阶段状态值。
 * @retval 阶段状态对应的字符串，未知值返回 "UNKNOWN"。
 */
const char *PhaseToString(uint8_t phase);

/*
 * @brief  解析应用层消息公共头。
 * @param  data: DATA 区数据指针。
 * @param  len: DATA 区长度。
 * @param  header: 输出解析得到的公共头。
 * @retval true 表示解析成功，false 表示长度不足或消息类型非法。
 */
bool DecodeHeader(const uint8_t *data, size_t len, MessageHeader &header);

/*
 * @brief  解析 START 消息负载。
 * @param  payload: 负载数据指针（公共头之后）。
 * @param  len: 负载长度。
 * @param  mission_id: 输出任务编号。
 * @retval true 表示解析成功。
 */
bool DecodeStart(const uint8_t *payload, size_t len, uint32_t &mission_id);

/*
 * @brief  解析 CAR_STATE 消息负载。
 * @param  payload: 负载数据指针（公共头之后）。
 * @param  len: 负载长度。
 * @param  phase: 输出阶段状态值。
 * @retval true 表示解析成功。
 */
bool DecodeCarState(const uint8_t *payload, size_t len, uint8_t &phase);

/*
 * @brief  解析 CAR_PROGRESS 消息负载。
 * @param  payload: 负载数据指针（公共头之后）。
 * @param  len: 负载长度。
 * @param  progress: 输出里程与速度。
 * @retval true 表示解析成功。
 */
bool DecodeCarProgress(const uint8_t *payload, size_t len, CarProgressPayload &progress);

/*
 * @brief  解析 HEARTBEAT 消息负载。
 * @param  payload: 负载数据指针（公共头之后）。
 * @param  len: 负载长度。
 * @param  heartbeat: 输出心跳负载。
 * @retval true 表示解析成功。
 */
bool DecodeHeartbeat(const uint8_t *payload, size_t len, HeartbeatPayload &heartbeat);

/*
 * @brief  解析 PAYLOAD_RELEASE 消息负载。
 * @param  payload: 负载数据指针（公共头之后）。
 * @param  len: 负载长度。
 * @param  release_id: 输出投放编号。
 * @retval true 表示解析成功。
 */
bool DecodePayloadRelease(const uint8_t *payload, size_t len, uint8_t &release_id);

/*
 * @brief  解析 MISSION_ABORT 消息负载。
 * @param  payload: 负载数据指针（公共头之后）。
 * @param  len: 负载长度。
 * @param  reason: 输出中止原因。
 * @retval true 表示解析成功。
 */
bool DecodeMissionAbort(const uint8_t *payload, size_t len, uint8_t &reason);

/*
 * @brief  将应用层 DATA 区组帧为完整线帧。
 * @param  data: DATA 区数据指针。
 * @param  len: DATA 区长度，不得超过 SERIAL1_MAX_DATA_LEN。
 * @retval 完整线帧 0x0F 0xF0 LEN DATA... CRC16_LO CRC16_HI 0xFF；长度非法时返回空 vector。
 * @note   CRC16/MODBUS 由 Serial_CalcCrc16 计算，覆盖 HEADER1+HEADER2+LEN+DATA，低字节在前。
 */
std::vector<uint8_t> BuildFrame(const uint8_t *data, size_t len);

/*
 * @brief  按公共头与负载组成应用层消息并组帧为完整线帧。
 * @param  type: 消息类型。
 * @param  session_id: 会话编号。
 * @param  seq: 帧序号。
 * @param  payload: 负载数据指针，可为 nullptr（负载长度为 0 时）。
 * @param  payload_len: 负载长度。
 * @retval 完整线帧；长度非法时返回空 vector。
 */
std::vector<uint8_t> BuildMessage(MsgType type, uint8_t session_id, uint8_t seq,
                                  const uint8_t *payload, size_t payload_len);

} // namespace mission_bridge

#endif /* MISSION_BRIDGE_PROTOCOL_HPP */
