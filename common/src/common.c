/*
 * @file    common.c
 * @brief   通用功能实现文件
 * @author  Wanone111
 * @note    该文件实现了通用返回码字符串转换和平台无关日志输出功能。
 */
#include "common.h"

#include <stdio.h>

#ifndef LOG_MESSAGE_BUFFER_SIZE
#define LOG_MESSAGE_BUFFER_SIZE 128U
#endif

static LogOutputCallback_t s_log_output_callback;

/*
 * @brief  根据日志等级获取默认返回码。
 * @param  level: 日志等级。
 * @retval LogStatus_t: 日志等级对应的返回状态。
 * @note   WARN 和 ERROR 等级会分别返回 LOG_STATUS_WARN 和 LOG_STATUS_ERROR。
 */
static LogStatus_t LOG_StatusFromLevel(LogLevel_t level)
{
    switch (level)
    {
        case LOG_LEVEL_WARN:
            return LOG_STATUS_WARN;
        case LOG_LEVEL_ERROR:
            return LOG_STATUS_ERROR;
        case LOG_LEVEL_DEBUG:
        case LOG_LEVEL_INFO:
            return LOG_STATUS_OK;
        default:
            return LOG_STATUS_INVALID_PARAM;
    }
}

/*
 * @brief  将通用返回码转换为字符串。
 * @param  status: 要转换的返回码。
 * @retval 返回码对应的字符串。
 * @note   未知返回码会转换为 UNKNOWN。
 */
const char *LogStatus_ToString(LogStatus_t status)
{
    switch (status)
    {
        case LOG_STATUS_OK:
            return "OK";
        case LOG_STATUS_WARN:
            return "WARN";
        case LOG_STATUS_ERROR:
            return "ERROR";
        case LOG_STATUS_INVALID_PARAM:
            return "INVALID_PARAM";
        case LOG_STATUS_BUSY:
            return "BUSY";
        case LOG_STATUS_TIMEOUT:
            return "TIMEOUT";
        case LOG_STATUS_CRC_ERROR:
            return "CRC_ERROR";
        case LOG_STATUS_BUFFER_OVERFLOW:
            return "BUFFER_OVERFLOW";
        case LOG_STATUS_NOT_INITIALIZED:
            return "NOT_INITIALIZED";
        case LOG_STATUS_UART_ERROR:
            return "UART_ERROR";
        case LOG_STATUS_SERIAL_HEADER_ERROR:
            return "SERIAL_HEADER_ERROR";
        case LOG_STATUS_SERIAL_LENGTH_ERROR:
            return "SERIAL_LENGTH_ERROR";
        case LOG_STATUS_SERIAL_CRC_ERROR:
            return "SERIAL_CRC_ERROR";
        case LOG_STATUS_SERIAL_TAIL_ERROR:
            return "SERIAL_TAIL_ERROR";
        case LOG_STATUS_SERIAL_BUFFER_OVERFLOW:
            return "SERIAL_BUFFER_OVERFLOW";
        case LOG_STATUS_SERIAL_UART_ERROR:
            return "SERIAL_UART_ERROR";
        case LOG_STATUS_ROS_INIT_ERROR:
            return "ROS_INIT_ERROR";
        case LOG_STATUS_ROS_PARAM_ERROR:
            return "ROS_PARAM_ERROR";
        case LOG_STATUS_ROS_PUBLISH_ERROR:
            return "ROS_PUBLISH_ERROR";
        case LOG_STATUS_ROS_SUBSCRIBE_ERROR:
            return "ROS_SUBSCRIBE_ERROR";
        case LOG_STATUS_ROS_SERIAL_OPEN_ERROR:
            return "ROS_SERIAL_OPEN_ERROR";
        default:
            return "UNKNOWN";
    }
}

/*
 * @brief  将日志等级转换为字符串。
 * @param  level: 要转换的日志等级。
 * @retval 日志等级对应的字符串。
 * @note   未知日志等级会转换为 UNKNOWN。
 */
const char *LogLevel_ToString(LogLevel_t level)
{
    switch (level)
    {
        case LOG_LEVEL_DEBUG:
            return "DEBUG";
        case LOG_LEVEL_INFO:
            return "INFO";
        case LOG_LEVEL_WARN:
            return "WARN";
        case LOG_LEVEL_ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
    }
}

/*
 * @brief  设置日志输出回调函数。
 * @param  callback: 日志输出回调函数指针，传入 NULL 表示关闭日志输出。
 * @retval None
 * @note   common 层只保存回调，不直接依赖具体输出外设或 ROS2 日志接口。
 */
void LOG_SetOutputCallback(LogOutputCallback_t callback)
{
    s_log_output_callback = callback;
}

/*
 * @brief  获取当前日志输出回调函数。
 * @param  None
 * @retval 当前日志输出回调函数指针。
 */
LogOutputCallback_t LOG_GetOutputCallback(void)
{
    return s_log_output_callback;
}

/*
 * @brief  使用 va_list 按指定日志等级输出格式化日志。
 * @param  level: 日志等级。
 * @param  format: printf 风格的格式字符串。
 * @param  args: 可变参数列表。
 * @retval LogStatus_t: 日志写入结果。
 * @note   日志格式固定为 [LEVEL] message，输出缓冲区长度由 LOG_MESSAGE_BUFFER_SIZE 控制。
 */
LogStatus_t LOG_WriteVa(LogLevel_t level, const char *format, va_list args)
{
    char payload[LOG_MESSAGE_BUFFER_SIZE];
    char message[LOG_MESSAGE_BUFFER_SIZE];
    int payload_len;
    int message_len;
    LogStatus_t status = LOG_StatusFromLevel(level);

    if (status == LOG_STATUS_INVALID_PARAM || format == NULL)
    {
        return LOG_STATUS_INVALID_PARAM;
    }

    payload_len = vsnprintf(payload, sizeof(payload), format, args);
    if (payload_len < 0)
    {
        return LOG_STATUS_ERROR;
    }

    message_len = snprintf(message,
                           sizeof(message),
                           "[%s] %s",
                           LogLevel_ToString(level),
                           payload);
    if (message_len < 0)
    {
        return LOG_STATUS_ERROR;
    }

    if (s_log_output_callback != NULL)
    {
        s_log_output_callback(level, message);
    }

    if ((payload_len >= (int)sizeof(payload)) ||
        (message_len >= (int)sizeof(message)))
    {
        return LOG_STATUS_WARN;
    }

    return status;
}

/*
 * @brief  按指定日志等级输出格式化日志。
 * @param  level: 日志等级。
 * @param  format: printf 风格的格式字符串。
 * @retval LogStatus_t: 日志写入结果。
 */
LogStatus_t LOG_Write(LogLevel_t level, const char *format, ...)
{
    LogStatus_t status;
    va_list args;

    va_start(args, format);
    status = LOG_WriteVa(level, format, args);
    va_end(args);

    return status;
}

/*
 * @brief  输出 DEBUG 等级日志。
 * @param  format: printf 风格的格式字符串。
 * @retval LogStatus_t: 日志写入结果。
 */
LogStatus_t LOG_Debug(const char *format, ...)
{
    LogStatus_t status;
    va_list args;

    va_start(args, format);
    status = LOG_WriteVa(LOG_LEVEL_DEBUG, format, args);
    va_end(args);

    return status;
}

/*
 * @brief  输出 INFO 等级日志。
 * @param  format: printf 风格的格式字符串。
 * @retval LogStatus_t: 日志写入结果。
 */
LogStatus_t LOG_Info(const char *format, ...)
{
    LogStatus_t status;
    va_list args;

    va_start(args, format);
    status = LOG_WriteVa(LOG_LEVEL_INFO, format, args);
    va_end(args);

    return status;
}

/*
 * @brief  输出 WARN 等级日志。
 * @param  format: printf 风格的格式字符串。
 * @retval LogStatus_t: 日志写入结果。
 */
LogStatus_t LOG_Warn(const char *format, ...)
{
    LogStatus_t status;
    va_list args;

    va_start(args, format);
    status = LOG_WriteVa(LOG_LEVEL_WARN, format, args);
    va_end(args);

    return status;
}

/*
 * @brief  输出 ERROR 等级日志。
 * @param  format: printf 风格的格式字符串。
 * @retval LogStatus_t: 日志写入结果。
 */
LogStatus_t LOG_Error(const char *format, ...)
{
    LogStatus_t status;
    va_list args;

    va_start(args, format);
    status = LOG_WriteVa(LOG_LEVEL_ERROR, format, args);
    va_end(args);

    return status;
}
