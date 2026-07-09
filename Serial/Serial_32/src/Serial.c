#include "serial.h"

/*
 * @file    Serial.c
 * @brief   串口通信模块实现文件
 * @author  Wanone111
 * @note    该文件实现了 UART1 新版帧解析、CRC16 校验、UART2 简单帧解析和接收中断入口。
 */

#ifndef SERIAL_HOST_TEST
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
#endif

volatile uint8_t RxData1;
volatile uint8_t RxData2;
volatile uint8_t Serial_RxFlag1;
volatile uint8_t Serial_RxFlag2;
Serial Serial_RxFrame1;
uint8_t Serial_RxPacket2[SERIAL2_FRAME_LEN];

static SerialParser s_uart1_parser;

/*
 * @brief  使用单个字节更新 CRC16/MODBUS 校验值。
 * @param  crc: 当前 CRC 值。
 * @param  byte: 要加入计算的字节。
 * @retval uint16_t: 更新后的 CRC 值。
 * @note   多字节 CRC 计算通过重复调用该函数完成。
 */
static uint16_t Serial_UpdateCrc16(uint16_t crc, uint8_t byte)
{
    uint8_t i;

    crc ^= byte;
    for (i = 0U; i < 8U; i++)
    {
        if ((crc & 0x0001U) != 0U)
        {
            crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
        }
        else
        {
            crc = (uint16_t)(crc >> 1U);
        }
    }

    return crc;
}

/*
 * @brief  清空串口帧结构体。
 * @param  frame: 指向 Serial 结构体的指针。
 * @retval None
 */
void Serial_FrameClear(Serial *frame)
{
    if (frame == NULL)
    {
        return;
    }

    memset(frame, 0, sizeof(*frame));
}

/*
 * @brief  初始化串口解析器。
 * @param  parser: 指向 SerialParser 结构体的指针。
 * @retval None
 * @note   该函数会将解析器状态恢复到等待帧头 1。
 */
void Serial_ParserInit(SerialParser *parser)
{
    if (parser == NULL)
    {
        return;
    }

    memset(parser, 0, sizeof(*parser));
    parser->state = PARSE_STATE_WAIT_HEADER1;
    parser->crc_calc = 0xFFFFU;
}

/*
 * @brief  计算 CRC16/MODBUS 校验值。
 * @param  data: 要计算 CRC 的数据指针。
 * @param  length: 数据长度。
 * @retval uint16_t: 计算得到的 CRC16 值。
 * @note   初始值为 0xFFFF，多项式为 0xA001。
 */
uint16_t Serial_CalcCrc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;

    if (data == NULL && length > 0U)
    {
        return 0U;
    }

    for (i = 0U; i < length; i++)
    {
        crc = Serial_UpdateCrc16(crc, data[i]);
    }

    return crc;
}

/*
 * @brief  解析串口接收到的字节。
 * @param  parser: 指向 SerialParser 结构体的指针。
 * @param  byte: 要解析的字节。
 * @param  frame: 指向 Serial 结构体的指针，用于存储解析后的帧数据。
 * @retval LogStatus_t: LOG_STATUS_BUSY 表示解析中，LOG_STATUS_OK 表示成功解析一帧，其它值表示错误。
 * @note   新版帧格式为 0x0F 0xF0 LEN DATA... CRC16_LOW CRC16_HIGH 0xFF。
 */
LogStatus_t Serial_ParseByte(SerialParser *parser, uint8_t byte, Serial *frame)
{
    if (parser == NULL || frame == NULL)
    {
        return LOG_STATUS_INVALID_PARAM;
    }

    switch (parser->state)
    {
        case PARSE_STATE_WAIT_HEADER1:
            if (byte != SERIAL1_HEADER1)
            {
                return LOG_STATUS_BUSY;
            }

            parser->crc_calc = 0xFFFFU;
            parser->crc_calc = Serial_UpdateCrc16(parser->crc_calc, byte);
            parser->state = PARSE_STATE_WAIT_HEADER2;
            return LOG_STATUS_BUSY;

        case PARSE_STATE_WAIT_HEADER2:
            if (byte != SERIAL1_HEADER2)
            {
                Serial_ParserInit(parser);
                if (byte == SERIAL1_HEADER1)
                {
                    parser->crc_calc = Serial_UpdateCrc16(parser->crc_calc, byte);
                    parser->state = PARSE_STATE_WAIT_HEADER2;
                }
                return LOG_STATUS_SERIAL_HEADER_ERROR;
            }

            parser->crc_calc = Serial_UpdateCrc16(parser->crc_calc, byte);
            parser->state = PARSE_STATE_READ_LENGTH;
            return LOG_STATUS_BUSY;

        case PARSE_STATE_READ_LENGTH:
#if (SERIAL1_MAX_DATA_LEN < 255U)
            if (byte > SERIAL1_MAX_DATA_LEN)
            {
                Serial_ParserInit(parser);
                return LOG_STATUS_SERIAL_LENGTH_ERROR;
            }
#endif

            parser->length = byte;
            parser->index = 0U;
            parser->crc_index = 0U;
            parser->crc_received = 0U;
            parser->crc_calc = Serial_UpdateCrc16(parser->crc_calc, byte);
            parser->state = (byte == 0U) ? PARSE_STATE_READ_CRC : PARSE_STATE_READ_DATA;
            return LOG_STATUS_BUSY;

        case PARSE_STATE_READ_DATA:
            if (parser->index >= SERIAL1_MAX_DATA_LEN)
            {
                Serial_ParserInit(parser);
                return LOG_STATUS_SERIAL_BUFFER_OVERFLOW;
            }

            parser->data[parser->index] = byte;
            parser->index++;
            parser->crc_calc = Serial_UpdateCrc16(parser->crc_calc, byte);
            if (parser->index >= parser->length)
            {
                parser->state = PARSE_STATE_READ_CRC;
            }
            return LOG_STATUS_BUSY;

        case PARSE_STATE_READ_CRC:
            if (parser->crc_index == 0U)
            {
                parser->crc_received = byte;
                parser->crc_index = 1U;
                return LOG_STATUS_BUSY;
            }

            parser->crc_received |= (uint16_t)((uint16_t)byte << 8U);
            if (parser->crc_received != parser->crc_calc)
            {
                Serial_ParserInit(parser);
                return LOG_STATUS_SERIAL_CRC_ERROR;
            }

            parser->state = PARSE_STATE_READ_TAIL;
            return LOG_STATUS_BUSY;

        case PARSE_STATE_READ_TAIL:
            if (byte != SERIAL1_TAIL)
            {
                Serial_ParserInit(parser);
                return LOG_STATUS_SERIAL_TAIL_ERROR;
            }

            Serial_FrameClear(frame);
            frame->length = parser->length;
            frame->crc = parser->crc_received;
            if (parser->length > 0U)
            {
                memcpy(frame->data, parser->data, parser->length);
            }
            Serial_ParserInit(parser);
            return LOG_STATUS_OK;

        default:
            Serial_ParserInit(parser);
            return LOG_STATUS_ERROR;
    }
}

/*
 * @brief  获取 UART1 接收完成标志。
 * @param  None
 * @retval uint8_t: 1 表示收到完整帧，0 表示没有新帧。
 */
uint8_t Serial_GetRxFlag1(void)
{
    if (Serial_RxFlag1 != 0U)
    {
        Serial_RxFlag1 = 0U;
        return 1U;
    }

    return 0U;
}

/*
 * @brief  获取 UART2 接收完成标志。
 * @param  None
 * @retval uint8_t: 1 表示收到完整帧，0 表示没有新帧。
 */
uint8_t Serial_GetRxFlag2(void)
{
    if (Serial_RxFlag2 != 0U)
    {
        Serial_RxFlag2 = 0U;
        return 1U;
    }

    return 0U;
}

/*
 * @brief  处理 UART1 当前接收到的字节。
 * @param  None
 * @retval LogStatus_t: UART1 字节解析结果。
 * @note   该函数只更新接收状态，不在中断中执行阻塞日志输出。
 */
LogStatus_t UART1_RxPacket(void)
{
    LogStatus_t status;

    status = Serial_ParseByte(&s_uart1_parser, RxData1, &Serial_RxFrame1);
    if (status == LOG_STATUS_OK)
    {
        Serial_RxFlag1 = 1U;
    }

    return status;
}

/*
 * @brief  处理 UART2 当前接收到的字节。
 * @param  None
 * @retval LogStatus_t: UART2 字节解析结果。
 * @note   UART2 当前采用 0xFF 0x0F DATA 0xFE 简单帧格式。
 */
LogStatus_t UART2_RxPacket(void)
{
    static uint8_t state;
    static uint8_t index;

    switch (state)
    {
        case 0U:
            if (RxData2 == SERIAL2_HEADER1)
            {
                Serial_RxPacket2[0] = RxData2;
                index = 1U;
                state = 1U;
            }
            return LOG_STATUS_BUSY;

        case 1U:
            if (RxData2 != SERIAL2_HEADER2)
            {
                state = 0U;
                index = 0U;
                return LOG_STATUS_SERIAL_HEADER_ERROR;
            }

            Serial_RxPacket2[index] = RxData2;
            index++;
            state = 2U;
            return LOG_STATUS_BUSY;

        case 2U:
            Serial_RxPacket2[index] = RxData2;
            index++;
            state = 3U;
            return LOG_STATUS_BUSY;

        case 3U:
            if (RxData2 != SERIAL2_TAIL)
            {
                state = 0U;
                index = 0U;
                return LOG_STATUS_SERIAL_TAIL_ERROR;
            }

            Serial_RxPacket2[index] = RxData2;
            Serial_RxFlag2 = 1U;
            state = 0U;
            index = 0U;
            return LOG_STATUS_OK;

        default:
            state = 0U;
            index = 0U;
            return LOG_STATUS_ERROR;
    }
}

#ifndef SERIAL_HOST_TEST
/**
  * @brief  串口接收中断回调函数。
  * @param  huart: 串口句柄。
  * @retval None
  * @note   中断中只推进状态机并重新开启接收，不执行阻塞日志输出。
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return;
    }

    if (huart->Instance == USART1)
    {
        (void)UART1_RxPacket();
        (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxData1, 1U);
    }
    else if (huart->Instance == USART2)
    {
        (void)UART2_RxPacket();
        (void)HAL_UART_Receive_IT(&huart2, (uint8_t *)&RxData2, 1U);
    }
    else
    {
        /* 未使用的 UART 实例，不处理。 */
    }
}
#endif
