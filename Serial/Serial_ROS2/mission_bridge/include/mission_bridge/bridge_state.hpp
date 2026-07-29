#ifndef MISSION_BRIDGE_BRIDGE_STATE_HPP
#define MISSION_BRIDGE_BRIDGE_STATE_HPP

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "mission_bridge/protocol.hpp"

namespace mission_bridge
{

enum class FrameDecision
{
    ACCEPT,
    DUPLICATE,
    SESSION_MISMATCH,
    STALE_SESSION,
};

class SessionTracker
{
public:
    explicit SessionTracker(std::size_t dedup_window_size);

    FrameDecision observe(MsgType type, uint8_t session_id, uint8_t seq);
    FrameDecision observeStart(uint8_t session_id, uint8_t seq, uint32_t mission_id);

    bool hasSession() const { return session_set_; }
    uint8_t activeSession() const { return active_session_; }
    uint8_t activeStartSeq() const { return active_start_seq_; }
    uint32_t activeMissionId() const { return active_mission_id_; }

private:
    static bool isNewerSession(uint8_t candidate, uint8_t current);
    bool isDuplicate(uint8_t session_id, uint8_t seq) const;
    void remember(uint8_t session_id, uint8_t seq);
    void resetDedup();

    bool session_set_{false};
    uint8_t active_session_{0};
    uint8_t active_start_seq_{0};
    uint32_t active_mission_id_{0};
    std::vector<std::pair<uint8_t, uint8_t>> dedup_window_;
    std::size_t dedup_head_{0};
    std::size_t dedup_count_{0};
};

enum class LinkTransition
{
    NONE,
    UP,
    DOWN,
};

class HeartbeatWatchdog
{
public:
    explicit HeartbeatWatchdog(std::int64_t timeout_ns);

    LinkTransition observe(std::int64_t now_ns, uint16_t heartbeat_seq);
    LinkTransition poll(std::int64_t now_ns);
    LinkTransition reset();

    bool linkUp() const { return link_up_; }
    bool hasHeartbeat() const { return heartbeat_seen_; }
    uint16_t heartbeatSeq() const { return heartbeat_seq_; }

private:
    std::int64_t timeout_ns_;
    std::int64_t last_heartbeat_ns_{0};
    uint16_t heartbeat_seq_{0};
    bool heartbeat_seen_{false};
    bool link_up_{false};
};

enum class SerialTransition
{
    NONE,
    CONNECTED,
    DISCONNECTED,
};

/*
 * @brief 串口连接边沿状态机。
 * @note  首次失败也产生一次 DISCONNECTED，持续失败不重复刷 fault，
 *        之后成功/再次断开分别产生 CONNECTED/DISCONNECTED。
 */
class SerialConnectionState
{
public:
    SerialTransition observe(bool connected);

    bool hasObservation() const { return observed_; }
    bool connected() const { return connected_; }

private:
    bool observed_{false};
    bool connected_{false};
};

} // namespace mission_bridge

#endif // MISSION_BRIDGE_BRIDGE_STATE_HPP
