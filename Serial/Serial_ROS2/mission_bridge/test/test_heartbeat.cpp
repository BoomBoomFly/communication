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

    HeartbeatWatchdog watchdog(3'000'000'000LL);
    CHECK(!watchdog.linkUp());
    CHECK(watchdog.poll(5'000'000'000LL) == LinkTransition::NONE);
    CHECK(watchdog.observe(10'000'000'000LL, 40U) == LinkTransition::UP);
    CHECK(watchdog.linkUp());
    CHECK(watchdog.heartbeatSeq() == 40U);
    CHECK(watchdog.poll(13'000'000'000LL) == LinkTransition::NONE);
    CHECK(watchdog.poll(13'000'000'001LL) == LinkTransition::DOWN);
    CHECK(!watchdog.linkUp());
    CHECK(watchdog.poll(14'000'000'000LL) == LinkTransition::NONE);
    CHECK(watchdog.observe(15'000'000'000LL, 41U) == LinkTransition::UP);
    CHECK(watchdog.observe(16'000'000'000LL, 42U) == LinkTransition::NONE);
    CHECK(watchdog.poll(18'999'999'999LL) == LinkTransition::NONE);
    return 0;
}
