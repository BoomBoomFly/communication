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
    CHECK(tracker.observe(MsgType::START, 4U, 1U) == FrameDecision::ACCEPT);
    CHECK(tracker.hasSession());
    CHECK(tracker.activeSession() == 4U);
    CHECK(tracker.observe(MsgType::START, 4U, 1U) == FrameDecision::DUPLICATE);
    CHECK(tracker.observe(MsgType::HEARTBEAT, 5U, 2U) == FrameDecision::SESSION_MISMATCH);

    CHECK(tracker.observe(MsgType::HEARTBEAT, 4U, 2U) == FrameDecision::ACCEPT);
    CHECK(tracker.observe(MsgType::CAR_STATE, 4U, 3U) == FrameDecision::ACCEPT);
    CHECK(tracker.observe(MsgType::START, 4U, 1U) == FrameDecision::ACCEPT);

    CHECK(tracker.observe(MsgType::START, 9U, 1U) == FrameDecision::ACCEPT);
    CHECK(tracker.activeSession() == 9U);
    CHECK(tracker.observe(MsgType::HEARTBEAT, 4U, 2U) == FrameDecision::SESSION_MISMATCH);
    CHECK(tracker.observe(MsgType::START, 9U, 1U) == FrameDecision::DUPLICATE);

    SessionTracker adopted(4U);
    CHECK(adopted.observe(MsgType::HEARTBEAT, 7U, 10U) == FrameDecision::ACCEPT);
    CHECK(adopted.activeSession() == 7U);
    return 0;
}
