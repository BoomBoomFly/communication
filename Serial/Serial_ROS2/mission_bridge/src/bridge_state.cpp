#include "mission_bridge/bridge_state.hpp"

#include <algorithm>
#include <stdexcept>

namespace mission_bridge
{

SessionTracker::SessionTracker(std::size_t dedup_window_size)
    : dedup_window_(std::max<std::size_t>(1U, dedup_window_size), {0U, 0U})
{
}

FrameDecision SessionTracker::observe(MsgType type, uint8_t session_id, uint8_t seq)
{
    if (type == MsgType::START)
    {
        if (!session_set_ || session_id != active_session_)
        {
            resetDedup();
        }
        active_session_ = session_id;
        session_set_ = true;
    }
    else if (!session_set_)
    {
        active_session_ = session_id;
        session_set_ = true;
    }
    else if (session_id != active_session_)
    {
        return FrameDecision::SESSION_MISMATCH;
    }

    if (isDuplicate(session_id, seq))
    {
        return FrameDecision::DUPLICATE;
    }
    remember(session_id, seq);
    return FrameDecision::ACCEPT;
}

bool SessionTracker::isDuplicate(uint8_t session_id, uint8_t seq) const
{
    for (std::size_t i = 0; i < dedup_count_; ++i)
    {
        if (dedup_window_[i].first == session_id && dedup_window_[i].second == seq)
        {
            return true;
        }
    }
    return false;
}

void SessionTracker::remember(uint8_t session_id, uint8_t seq)
{
    dedup_window_[dedup_head_] = {session_id, seq};
    dedup_head_ = (dedup_head_ + 1U) % dedup_window_.size();
    if (dedup_count_ < dedup_window_.size())
    {
        ++dedup_count_;
    }
}

void SessionTracker::resetDedup()
{
    dedup_head_ = 0U;
    dedup_count_ = 0U;
}

HeartbeatWatchdog::HeartbeatWatchdog(std::int64_t timeout_ns)
    : timeout_ns_(timeout_ns)
{
    if (timeout_ns_ <= 0)
    {
        throw std::invalid_argument("heartbeat timeout must be positive");
    }
}

LinkTransition HeartbeatWatchdog::observe(std::int64_t now_ns, uint16_t heartbeat_seq)
{
    heartbeat_seen_ = true;
    last_heartbeat_ns_ = now_ns;
    heartbeat_seq_ = heartbeat_seq;
    if (!link_up_)
    {
        link_up_ = true;
        return LinkTransition::UP;
    }
    return LinkTransition::NONE;
}

LinkTransition HeartbeatWatchdog::poll(std::int64_t now_ns)
{
    if (!heartbeat_seen_ || !link_up_ || now_ns < last_heartbeat_ns_)
    {
        return LinkTransition::NONE;
    }
    if (now_ns - last_heartbeat_ns_ > timeout_ns_)
    {
        link_up_ = false;
        return LinkTransition::DOWN;
    }
    return LinkTransition::NONE;
}

} // namespace mission_bridge
