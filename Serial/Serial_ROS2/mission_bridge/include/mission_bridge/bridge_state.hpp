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
};

class SessionTracker
{
public:
    explicit SessionTracker(std::size_t dedup_window_size);

    FrameDecision observe(MsgType type, uint8_t session_id, uint8_t seq);

    bool hasSession() const { return session_set_; }
    uint8_t activeSession() const { return active_session_; }

private:
    bool isDuplicate(uint8_t session_id, uint8_t seq) const;
    void remember(uint8_t session_id, uint8_t seq);
    void resetDedup();

    bool session_set_{false};
    uint8_t active_session_{0};
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

} // namespace mission_bridge

#endif // MISSION_BRIDGE_BRIDGE_STATE_HPP
