#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>

#include "mission_bridge/protocol.hpp"

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

    const uint8_t header_bytes[] = {
        static_cast<uint8_t>(MsgType::HEARTBEAT), 0x21U, 0x42U};
    MessageHeader header{};
    CHECK(DecodeHeader(header_bytes, sizeof(header_bytes), header));
    CHECK(header.type == MsgType::HEARTBEAT);
    CHECK(header.session_id == 0x21U);
    CHECK(header.seq == 0x42U);
    CHECK(!DecodeHeader(nullptr, 0U, header));
    const uint8_t invalid_type[] = {0x80U, 0U, 0U};
    CHECK(!DecodeHeader(invalid_type, sizeof(invalid_type), header));

    const uint8_t start_payload[] = {0x78U, 0x56U, 0x34U, 0x12U};
    uint32_t mission_id = 0U;
    CHECK(DecodeStart(start_payload, sizeof(start_payload), mission_id));
    CHECK(mission_id == 0x12345678U);
    CHECK(!DecodeStart(start_payload, 3U, mission_id));

    const float mileage = 12.5F;
    const float speed = -1.25F;
    uint32_t mileage_bits = 0U;
    uint32_t speed_bits = 0U;
    std::memcpy(&mileage_bits, &mileage, sizeof(mileage_bits));
    std::memcpy(&speed_bits, &speed, sizeof(speed_bits));
    uint8_t progress_payload[8]{};
    for (unsigned i = 0; i < 4U; ++i)
    {
        progress_payload[i] = static_cast<uint8_t>((mileage_bits >> (8U * i)) & 0xFFU);
        progress_payload[4U + i] = static_cast<uint8_t>((speed_bits >> (8U * i)) & 0xFFU);
    }
    CarProgressPayload progress{};
    CHECK(DecodeCarProgress(progress_payload, sizeof(progress_payload), progress));
    CHECK(std::fabs(progress.mileage - mileage) < 0.0001F);
    CHECK(std::fabs(progress.speed - speed) < 0.0001F);

    const uint8_t heartbeat_payload[] = {
        0x34U, 0x12U, 0x04U, 0x03U, 0x02U, 0x01U, 0x01U};
    HeartbeatPayload heartbeat{};
    CHECK(DecodeHeartbeat(heartbeat_payload, sizeof(heartbeat_payload), heartbeat));
    CHECK(heartbeat.hb_seq == 0x1234U);
    CHECK(heartbeat.time_ms == 0x01020304U);
    CHECK(heartbeat.link_status == 1U);

    CHECK(std::strcmp(PhaseToString(PHASE_RUNNING), "RUNNING") == 0);
    CHECK(std::strcmp(PhaseToString(0xFFU), "UNKNOWN") == 0);
    return 0;
}
