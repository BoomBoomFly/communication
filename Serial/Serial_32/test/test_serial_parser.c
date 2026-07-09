/*
 * @file    test_serial_parser.c
 * @brief   Serial_32 串口帧解析单元测试文件
 * @author  Wanone111
 * @note    该文件用于验证新版 UART1 帧格式、CRC16 校验和错误返回码。
 */
#include "serial.h"

#include <assert.h>
#include <stddef.h>
#include <string.h>

/*
 * @brief  构造 UART1 测试帧。
 * @param  output: 输出帧缓冲区。
 * @param  payload: 数据区指针。
 * @param  length: 数据区长度。
 * @retval size_t: 构造后的帧长度。
 * @note   帧格式为 0x0F 0xF0 LEN DATA... CRC16_LOW CRC16_HIGH 0xFF。
 */
static size_t BuildSerial1Frame(uint8_t *output, const uint8_t *payload, uint8_t length)
{
    uint16_t crc;

    output[0] = SERIAL1_HEADER1;
    output[1] = SERIAL1_HEADER2;
    output[2] = length;
    memcpy(&output[3], payload, length);

    crc = Serial_CalcCrc16(output, (uint16_t)(3U + length));
    output[3U + length] = (uint8_t)(crc & 0xFFU);
    output[4U + length] = (uint8_t)((crc >> 8U) & 0xFFU);
    output[5U + length] = SERIAL1_TAIL;

    return (size_t)(6U + length);
}

/*
 * @brief  向解析器输入一组字节直到解析完成或出现错误。
 * @param  parser: 串口解析器指针。
 * @param  frame: 串口帧输出指针。
 * @param  bytes: 输入字节数组。
 * @param  length: 输入字节数量。
 * @retval LogStatus_t: 解析结果。
 */
static LogStatus_t FeedUntilDone(SerialParser *parser,
                                 Serial *frame,
                                 const uint8_t *bytes,
                                 size_t length)
{
    LogStatus_t status = LOG_STATUS_BUSY;
    size_t i;

    for (i = 0U; i < length; i++)
    {
        status = Serial_ParseByte(parser, bytes[i], frame);
        if (status != LOG_STATUS_BUSY)
        {
            return status;
        }
    }

    return status;
}

/*
 * @brief  Serial_32 串口帧解析测试入口。
 * @param  None
 * @retval int: 返回 0 表示测试通过。
 */
int main(void)
{
    SerialParser parser;
    Serial frame;
    uint8_t raw_frame[16];
    const uint8_t payload[] = {0x10U, 0x20U, 0x30U};
    size_t raw_length;
    LogStatus_t status;

    assert(Serial_CalcCrc16((const uint8_t *)"123456789", 9U) == 0x4B37U);

    Serial_ParserInit(&parser);
    Serial_FrameClear(&frame);
    raw_length = BuildSerial1Frame(raw_frame, payload, (uint8_t)sizeof(payload));
    status = FeedUntilDone(&parser, &frame, raw_frame, raw_length);
    assert(status == LOG_STATUS_OK);
    assert(frame.length == sizeof(payload));
    assert(memcmp(frame.data, payload, sizeof(payload)) == 0);
    assert(parser.state == PARSE_STATE_WAIT_HEADER1);

    Serial_ParserInit(&parser);
    Serial_FrameClear(&frame);
    raw_length = BuildSerial1Frame(raw_frame, payload, (uint8_t)sizeof(payload));
    raw_frame[3U + sizeof(payload)] ^= 0x01U;
    status = FeedUntilDone(&parser, &frame, raw_frame, raw_length);
    assert(status == LOG_STATUS_SERIAL_CRC_ERROR);
    assert(parser.state == PARSE_STATE_WAIT_HEADER1);

    Serial_ParserInit(&parser);
    Serial_FrameClear(&frame);
    raw_frame[0] = SERIAL1_HEADER1;
    raw_frame[1] = SERIAL1_HEADER2;
    raw_frame[2] = (uint8_t)(SERIAL1_MAX_DATA_LEN + 1U);
    status = FeedUntilDone(&parser, &frame, raw_frame, 3U);
    assert(status == LOG_STATUS_SERIAL_LENGTH_ERROR);
    assert(parser.state == PARSE_STATE_WAIT_HEADER1);

    assert(Serial_ParseByte(NULL, SERIAL1_HEADER1, &frame) == LOG_STATUS_INVALID_PARAM);
    assert(Serial_ParseByte(&parser, SERIAL1_HEADER1, NULL) == LOG_STATUS_INVALID_PARAM);

    return 0;
}
