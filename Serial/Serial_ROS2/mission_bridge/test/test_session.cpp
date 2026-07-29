#include <iostream>

#include "mission_bridge/bridge_state.hpp"

namespace
{

bool check(bool condition, const char *expression, int line)
{
    if (!condition)
    {
        std::cerr << "CHECK failed at line " << line << ": " << expression << '\n';
    }
    return condition;
}

#define CHECK(value) do { if (!check((value), #value, __LINE__)) return 1; } while (false)

} // namespace

int main()
{
    using namespace mission_bridge;

    SessionTracker tracker(2U);
    CHECK(tracker.observe(MsgType::HEARTBEAT, 4U, 0U) ==
          FrameDecision::SESSION_MISMATCH);
    CHECK(tracker.observeStart(4U, 1U, 2U) == FrameDecision::ACCEPT);
    CHECK(tracker.hasSession());
    CHECK(tracker.activeSession() == 4U);
    CHECK(tracker.activeStartSeq() == 1U);
    CHECK(tracker.activeMissionId() == 2U);
    CHECK(tracker.observeStart(4U, 1U, 2U) == FrameDecision::DUPLICATE);
    CHECK(tracker.observeStart(4U, 99U, 1U) == FrameDecision::DUPLICATE);
    CHECK(tracker.observe(MsgType::HEARTBEAT, 5U, 2U) == FrameDecision::SESSION_MISMATCH);

    CHECK(tracker.observe(MsgType::HEARTBEAT, 4U, 2U) == FrameDecision::ACCEPT);
    CHECK(tracker.observe(MsgType::CAR_STATE, 4U, 3U) == FrameDecision::ACCEPT);
    CHECK(tracker.observe(MsgType::HEARTBEAT, 4U, 2U) == FrameDecision::DUPLICATE);
    CHECK(tracker.observe(MsgType::CAR_PROGRESS, 4U, 4U) == FrameDecision::ACCEPT);
    CHECK(tracker.observe(MsgType::HEARTBEAT, 4U, 2U) == FrameDecision::ACCEPT);

    CHECK(tracker.observeStart(3U, 1U, 1U) == FrameDecision::STALE_SESSION);
    CHECK(tracker.observeStart(9U, 1U, 1U) == FrameDecision::ACCEPT);
    CHECK(tracker.activeSession() == 9U);
    CHECK(tracker.observe(MsgType::HEARTBEAT, 4U, 2U) == FrameDecision::SESSION_MISMATCH);
    CHECK(tracker.observeStart(9U, 1U, 1U) == FrameDecision::DUPLICATE);

    SessionTracker wrapped(4U);
    CHECK(wrapped.observeStart(255U, 7U, 2U) == FrameDecision::ACCEPT);
    CHECK(wrapped.observeStart(0U, 8U, 1U) == FrameDecision::ACCEPT);
    CHECK(wrapped.activeSession() == 0U);
    CHECK(wrapped.observeStart(128U, 9U, 1U) == FrameDecision::STALE_SESSION);
    return 0;
}
