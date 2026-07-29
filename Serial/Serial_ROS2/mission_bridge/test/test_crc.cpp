#include <cstdint>
#include <iostream>
#include <vector>

#include "mission_bridge/protocol.hpp"

extern "C" {
#include "serial.h"
}

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

LogStatus_t parse(const std::vector<uint8_t> &bytes, Serial &frame)
{
    SerialParser parser{};
    Serial_ParserInit(&parser);
    LogStatus_t status = LOG_STATUS_BUSY;
    for (const auto byte : bytes)
    {
        status = Serial_ParseByte(&parser, byte, &frame);
        if (status != LOG_STATUS_BUSY && status != LOG_STATUS_OK)
        {
            return status;
        }
    }
    return status;
}

} // namespace

int main()
{
    using namespace mission_bridge;

    const uint8_t canonical[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK(Serial_CalcCrc16(canonical, sizeof(canonical)) == 0x4B37U);

    const uint8_t payload[] = {0x04U, 0x03U, 0x02U, 0x01U};
    auto encoded = BuildMessage(MsgType::START, 9U, 7U, payload, sizeof(payload));
    CHECK(!encoded.empty());

    Serial frame{};
    CHECK(parse(encoded, frame) == LOG_STATUS_OK);
    CHECK(frame.length == 7U);
    CHECK(frame.data[0] == static_cast<uint8_t>(MsgType::START));
    CHECK(frame.data[1] == 9U);
    CHECK(frame.data[2] == 7U);

    encoded[4] ^= 0x01U;
    CHECK(parse(encoded, frame) == LOG_STATUS_SERIAL_CRC_ERROR);
    return 0;
}
