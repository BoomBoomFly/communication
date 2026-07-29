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

    SerialConnectionState state;
    CHECK(!state.hasObservation());
    CHECK(state.observe(false) == SerialTransition::DISCONNECTED);
    CHECK(state.hasObservation());
    CHECK(!state.connected());
    CHECK(state.observe(false) == SerialTransition::NONE);
    CHECK(state.observe(true) == SerialTransition::CONNECTED);
    CHECK(state.connected());
    CHECK(state.observe(true) == SerialTransition::NONE);
    CHECK(state.observe(false) == SerialTransition::DISCONNECTED);
    CHECK(!state.connected());
    CHECK(state.observe(true) == SerialTransition::CONNECTED);
    return 0;
}
