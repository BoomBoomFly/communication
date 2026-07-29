/*
 * @file    protocol.cpp
 * @brief   应用层协议编解码实现文件
 * @author  Wanone111
 * @note    该文件实现了 mission_bridge 应用层消息公共头解析、各类型负载编解码和线帧组帧。
 *          多字节字段全部采用小端序，逐字节拼装以保证大小端平台行为一致。
 */

#include "mission_bridge/protocol.hpp"

#include <cstring>

#include "serial.h"

namespace mission_bridge
{

namespace
{

/*
 * @brief  以小端序读取 u16。
 * @param  in: 数据指针。
 * @retval 读取到的 u16 值。
 */
uint16_t GetU16Le(const uint8_t *in)
{
    return (uint16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8U));
}

/*
 * @brief  以小端序读取 u32。
 * @param  in: 数据指针。
 * @retval 读取到的 u32 值。
 */
uint32_t GetU32Le(const uint8_t *in)
{
    return (uint32_t)in[0] | ((uint32_t)in[1] << 8U) | ((uint32_t)in[2] << 16U) |
           ((uint32_t)in[3] << 24U);
}

/*
 * @brief  以小端序读取 f32。
 * @param  in: 数据指针。
 * @retval 读取到的 f32 值。
 */
float GetF32Le(const uint8_t *in)
{
    uint32_t bits = GetU32Le(in);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

} // namespace

const char *PhaseToString(uint8_t phase)
{
    switch (phase)
    {
        case PHASE_IDLE:      return "IDLE";
        case PHASE_STARTUP:   return "STARTUP";
        case PHASE_RUNNING:   return "RUNNING";
        case PHASE_ARRIVE_A:  return "ARRIVE_A";
        case PHASE_ARRIVE_B:  return "ARRIVE_B";
        case PHASE_ARRIVE_C:  return "ARRIVE_C";
        case PHASE_ARRIVE_D:  return "ARRIVE_D";
        case PHASE_COMPLETED: return "COMPLETED";
        default:              return "UNKNOWN";
    }
}

bool IsSupportedMissionId(uint32_t mission_id)
{
    return mission_id == MISSION_ID_CONTEST_TASK1 ||
           mission_id == MISSION_ID_CONTEST_TASK2 ||
           mission_id == MISSION_ID_VERTICAL_TEST;
}

uint64_t EncodeStartContext(const StartContext &context)
{
    return static_cast<uint64_t>(context.mission_id) |
           (static_cast<uint64_t>(context.session_id) << 16U) |
           (static_cast<uint64_t>(context.seq) << 24U) |
           (static_cast<uint64_t>(context.source_epoch) << 32U);
}

StartContext DecodeStartContext(uint64_t encoded)
{
    StartContext context{};
    context.mission_id = static_cast<uint16_t>(encoded & 0xFFFFU);
    context.session_id = static_cast<uint8_t>((encoded >> 16U) & 0xFFU);
    context.seq = static_cast<uint8_t>((encoded >> 24U) & 0xFFU);
    context.source_epoch = static_cast<uint32_t>((encoded >> 32U) & 0xFFFFFFFFU);
    return context;
}

bool DecodeHeader(const uint8_t *data, size_t len, MessageHeader &header)
{
    if (data == nullptr || len < 3U)
    {
        return false;
    }

    uint8_t type = data[0];
    if (type < (uint8_t)MsgType::START || type > (uint8_t)MsgType::MISSION_ABORT)
    {
        return false;
    }

    header.type = (MsgType)type;
    header.session_id = data[1];
    header.seq = data[2];
    return true;
}

bool IsPayloadLengthValid(MsgType type, size_t len)
{
    switch (type)
    {
        case MsgType::START:           return len == 4U;
        case MsgType::CAR_STATE:       return len == 1U;
        case MsgType::CAR_PROGRESS:    return len == 8U;
        case MsgType::HEARTBEAT:       return len == 7U;
        case MsgType::PAYLOAD_RELEASE: return len == 1U;
        case MsgType::PAYLOAD_ACK:     return len == 3U;
        case MsgType::MISSION_ABORT:   return len == 1U;
        default:                       return false;
    }
}

bool DecodeStart(const uint8_t *payload, size_t len, uint32_t &mission_id)
{
    if (payload == nullptr || len != 4U)
    {
        return false;
    }

    mission_id = GetU32Le(payload);
    return true;
}

bool DecodeCarState(const uint8_t *payload, size_t len, uint8_t &phase)
{
    if (payload == nullptr || len != 1U)
    {
        return false;
    }

    phase = payload[0];
    return true;
}

bool DecodeCarProgress(const uint8_t *payload, size_t len, CarProgressPayload &progress)
{
    if (payload == nullptr || len != 8U)
    {
        return false;
    }

    progress.mileage = GetF32Le(payload);
    progress.speed = GetF32Le(payload + 4U);
    return true;
}

bool DecodeHeartbeat(const uint8_t *payload, size_t len, HeartbeatPayload &heartbeat)
{
    if (payload == nullptr || len != 7U)
    {
        return false;
    }

    heartbeat.hb_seq = GetU16Le(payload);
    heartbeat.time_ms = GetU32Le(payload + 2U);
    heartbeat.link_status = payload[6];
    return true;
}

bool DecodePayloadRelease(const uint8_t *payload, size_t len, uint8_t &release_id)
{
    if (payload == nullptr || len != 1U)
    {
        return false;
    }

    release_id = payload[0];
    return true;
}

bool DecodeMissionAbort(const uint8_t *payload, size_t len, uint8_t &reason)
{
    if (payload == nullptr || len != 1U)
    {
        return false;
    }

    reason = payload[0];
    return true;
}

std::vector<uint8_t> BuildFrame(const uint8_t *data, size_t len)
{
    std::vector<uint8_t> frame;

    if (len > SERIAL1_MAX_DATA_LEN || (data == nullptr && len > 0U))
    {
        return frame;
    }

    frame.resize(len + SERIAL1_FRAME_OVERHEAD);
    frame[0] = SERIAL1_HEADER1;
    frame[1] = SERIAL1_HEADER2;
    frame[2] = (uint8_t)len;
    if (len > 0U)
    {
        std::memcpy(&frame[3], data, len);
    }

    /* CRC16/MODBUS 覆盖 HEADER1 + HEADER2 + LEN + DATA，低字节在前 */
    uint16_t crc = Serial_CalcCrc16(frame.data(), (uint16_t)(3U + len));
    frame[3U + len] = (uint8_t)(crc & 0xFFU);
    frame[4U + len] = (uint8_t)((crc >> 8U) & 0xFFU);
    frame[5U + len] = SERIAL1_TAIL;

    return frame;
}

std::vector<uint8_t> BuildMessage(MsgType type, uint8_t session_id, uint8_t seq,
                                  const uint8_t *payload, size_t payload_len)
{
    std::vector<uint8_t> data;

    if (payload_len + 3U > SERIAL1_MAX_DATA_LEN || (payload == nullptr && payload_len > 0U))
    {
        return data;
    }

    data.resize(3U + payload_len);
    data[0] = (uint8_t)type;
    data[1] = session_id;
    data[2] = seq;
    if (payload_len > 0U)
    {
        std::memcpy(&data[3], payload, payload_len);
    }

    return BuildFrame(data.data(), data.size());
}

} // namespace mission_bridge
