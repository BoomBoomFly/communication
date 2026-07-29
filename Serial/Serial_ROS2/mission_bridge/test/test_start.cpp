#include <cstdint>
#include <iostream>
#include <vector>

#include "mission_bridge/bridge_state.hpp"
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

bool decodeStartFrame(const std::vector<uint8_t> &wire, mission_bridge::MessageHeader &header,
                      uint32_t &mission_id)
{
    SerialParser parser{};
    Serial frame{};
    Serial_ParserInit(&parser);
    LogStatus_t status = LOG_STATUS_BUSY;
    for (uint8_t byte : wire)
    {
        status = Serial_ParseByte(&parser, byte, &frame);
    }
    return status == LOG_STATUS_OK &&
           mission_bridge::DecodeHeader(frame.data, frame.length, header) &&
           header.type == mission_bridge::MsgType::START &&
           mission_bridge::IsPayloadLengthValid(header.type, frame.length - 3U) &&
           mission_bridge::DecodeStart(frame.data + 3U, frame.length - 3U, mission_id);
}

} // namespace

int main()
{
    using namespace mission_bridge;

    const uint8_t payload[] = {0x02U, 0x00U, 0x00U, 0x00U};
    const auto wire = BuildMessage(MsgType::START, 10U, 42U, payload, sizeof(payload));
    MessageHeader header{};
    uint32_t mission_id = 0U;
    CHECK(decodeStartFrame(wire, header, mission_id));
    CHECK(mission_id == 2U);
    CHECK(IsSupportedMissionId(mission_id));
    CHECK(header.session_id == 10U);
    CHECK(header.seq == 42U);

    SessionTracker tracker(32U);
    unsigned published_events = 0U;
    if (tracker.observeStart(header.session_id, header.seq, mission_id) ==
        FrameDecision::ACCEPT)
    {
        ++published_events;
    }
    CHECK(published_events == 1U);
    CHECK(tracker.activeMissionId() == 2U);

    /* 完全重复和同 session 换 seq 的 START 都不能产生新事件。 */
    CHECK(tracker.observeStart(10U, 42U, 2U) == FrameDecision::DUPLICATE);
    CHECK(tracker.observeStart(10U, 43U, 1U) == FrameDecision::DUPLICATE);
    CHECK(published_events == 1U);

    /* 旧 session 不能替换；仅更新的 session 能替换并携带新任务编号。 */
    CHECK(tracker.observeStart(9U, 1U, 1U) == FrameDecision::STALE_SESSION);
    CHECK(tracker.observeStart(11U, 1U, 1U) == FrameDecision::ACCEPT);
    CHECK(tracker.activeSession() == 11U);
    CHECK(tracker.activeStartSeq() == 1U);
    CHECK(tracker.activeMissionId() == 1U);
    CHECK(!IsSupportedMissionId(0U));
    CHECK(!IsSupportedMissionId(4U));

    /* bridge 重启产生新 source_epoch；迟到的旧 context 不能匹配当前实例。 */
    StartContext old_context{};
    old_context.mission_id = MISSION_ID_VERTICAL_TEST;
    old_context.session_id = 20U;
    old_context.seq = 7U;
    old_context.source_epoch = 1001U;
    StartContext restarted_context = old_context;
    restarted_context.session_id = 21U;
    restarted_context.seq = 1U;
    restarted_context.source_epoch = 1002U;
    const auto old_event = DecodeStartContext(EncodeStartContext(old_context));
    const auto restarted_event = DecodeStartContext(EncodeStartContext(restarted_context));
    const uint32_t current_source_epoch = restarted_event.source_epoch;
    const auto is_current_start = [current_source_epoch](const StartContext &context,
                                                         uint32_t start_mission_id) {
        return context.source_epoch == current_source_epoch &&
               context.mission_id == start_mission_id;
    };
    CHECK(restarted_event.source_epoch == 1002U);
    CHECK(old_event.source_epoch != current_source_epoch);
    CHECK(old_event.mission_id == restarted_event.mission_id);
    CHECK(old_event.session_id != restarted_event.session_id);
    CHECK(!is_current_start(old_event, MISSION_ID_VERTICAL_TEST));
    CHECK(is_current_start(restarted_event, MISSION_ID_VERTICAL_TEST));
    return 0;
}
