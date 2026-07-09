/*
 * @file    serial.h
 * @brief   串口通信模块头文件
 * @author  Wanone111
 * @note    该文件定义了串口通信模块的接口和数据结构，包括新版帧格式、解析状态和解析器结构。
 */

#ifndef __SERIAL_H
#define __SERIAL_H

#include "main.h"
#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * @brief  串口通信模块常量定义。
 * @note   UART1 新版帧格式为 0x0F 0xF0 LEN DATA... CRC16_LOW CRC16_HIGH 0xFF。
 */
#define SERIAL1_HEADER1 0x0FU
#define SERIAL1_HEADER2 0xF0U
#define SERIAL1_TAIL    0xFFU

#define SERIAL2_HEADER1 0xFFU
#define SERIAL2_HEADER2 0x0FU
#define SERIAL2_TAIL    0xFEU

#ifndef SERIAL1_MAX_DATA_LEN
#define SERIAL1_MAX_DATA_LEN 255U
#endif

#if (SERIAL1_MAX_DATA_LEN > 255U)
#error "SERIAL1_MAX_DATA_LEN must be less than or equal to 255"
#endif

#define SERIAL1_CRC_SIZE        2U
#define SERIAL1_FRAME_OVERHEAD  6U
#define SERIAL2_FRAME_LEN       4U

/*
 * @brief  串口解析状态枚举。
 * @note   该枚举用于表示 UART1 新版帧解析的不同状态。
 */
typedef enum
{
    PARSE_STATE_WAIT_HEADER1 = 0,
    PARSE_STATE_WAIT_HEADER2,
    PARSE_STATE_READ_LENGTH,
    PARSE_STATE_READ_DATA,
    PARSE_STATE_READ_CRC,
    PARSE_STATE_READ_TAIL
} ParseState;

/*
 * @brief  UART1 串口帧结构体。
 * @note   该结构体保存解析完成后的数据长度、数据区和接收到的 CRC 值。
 */
typedef struct
{
    uint8_t length;
    uint8_t data[SERIAL1_MAX_DATA_LEN];
    uint16_t crc;
} Serial;

/*
 * @brief  UART1 串口解析器结构体。
 * @note   该结构体保存状态机运行过程中的临时数据，供 Serial_ParseByte 逐字节解析使用。
 */
typedef struct
{
    ParseState state;
    uint8_t length;
    uint8_t index;
    uint8_t crc_index;
    uint16_t crc_calc;
    uint16_t crc_received;
    uint8_t data[SERIAL1_MAX_DATA_LEN];
} SerialParser;

extern volatile uint8_t RxData1;
extern volatile uint8_t RxData2;
extern volatile uint8_t Serial_RxFlag1;
extern volatile uint8_t Serial_RxFlag2;
extern Serial Serial_RxFrame1;
extern uint8_t Serial_RxPacket2[SERIAL2_FRAME_LEN];

/*
 * @brief  清空串口帧结构体。
 * @param  frame: 指向 Serial 结构体的指针。
 * @retval None
 * @note   该函数用于在解析前或错误恢复时清空帧数据。
 */
void Serial_FrameClear(Serial *frame);

/*
 * @brief  初始化串口解析器。
 * @param  parser: 指向 SerialParser 结构体的指针。
 * @retval None
 * @note   该函数会将解析器状态恢复到等待帧头 1。
 */
void Serial_ParserInit(SerialParser *parser);

/*
 * @brief  计算 CRC16/MODBUS 校验值。
 * @param  data: 要计算 CRC 的数据指针。
 * @param  length: 数据长度。
 * @retval uint16_t: 计算得到的 CRC16 值。
 * @note   UART1 新版帧的 CRC 覆盖 HEADER1、HEADER2、LEN 和 DATA。
 */
uint16_t Serial_CalcCrc16(const uint8_t *data, uint16_t length);

/*
 * @brief  解析串口接收到的字节。
 * @param  parser: 指向 SerialParser 结构体的指针。
 * @param  byte: 要解析的字节。
 * @param  frame: 指向 Serial 结构体的指针，用于存储解析后的帧数据。
 * @retval LogStatus_t: LOG_STATUS_BUSY 表示解析中，LOG_STATUS_OK 表示成功解析一帧，其它值表示错误。
 * @note   该函数不直接读取全局接收变量，便于单元测试和多串口复用。
 */
LogStatus_t Serial_ParseByte(SerialParser *parser, uint8_t byte, Serial *frame);

/*
 * @brief  获取 UART1 接收完成标志。
 * @param  None
 * @retval uint8_t: 1 表示收到完整帧，0 表示没有新帧。
 * @note   读取到 1 后函数会自动清零标志位。
 */
uint8_t Serial_GetRxFlag1(void);

/*
 * @brief  获取 UART2 接收完成标志。
 * @param  None
 * @retval uint8_t: 1 表示收到完整帧，0 表示没有新帧。
 * @note   读取到 1 后函数会自动清零标志位。
 */
uint8_t Serial_GetRxFlag2(void);

/*
 * @brief  处理 UART1 当前接收到的字节。
 * @param  None
 * @retval LogStatus_t: UART1 字节解析结果。
 * @note   该函数通常由 HAL_UART_RxCpltCallback 调用。
 */
LogStatus_t UART1_RxPacket(void);

/*
 * @brief  处理 UART2 当前接收到的字节。
 * @param  None
 * @retval LogStatus_t: UART2 字节解析结果。
 * @note   UART2 当前采用 0xFF 0x0F DATA 0xFE 简单帧格式。
 */
LogStatus_t UART2_RxPacket(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERIAL_H */
