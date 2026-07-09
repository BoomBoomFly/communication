/*
 * @file    common.h
 * @brief   通用定义头文件
 * @author  Wanone111
 * @note    该文件定义了通信模块共用的返回码、日志等级和日志输出接口。
 */

#ifndef COMMUNICATION_COMMON_H
#define COMMUNICATION_COMMON_H

#include <stdarg.h>
#include <stdint.h>

/*
 * @brief  应用模块共用的返回码。
 * @note   该枚举用于表示通信模块和日志模块的通用返回状态。
 */
typedef enum
{
    LOG_STATUS_OK = 0,              /* 操作成功 */
    LOG_STATUS_WARN,                /* 操作完成但存在警告 */
    LOG_STATUS_ERROR,               /* 通用错误 */
    LOG_STATUS_INVALID_PARAM,       /* 参数无效 */
    LOG_STATUS_BUSY,                /* 资源忙 */
    LOG_STATUS_TIMEOUT,             /* 操作超时 */
    LOG_STATUS_CRC_ERROR,           /* CRC 校验错误 */
    LOG_STATUS_BUFFER_OVERFLOW,     /* 缓冲区溢出 */
    LOG_STATUS_NOT_INITIALIZED,     /* 模块未初始化 */

    /* UART 相关错误码 */
    LOG_STATUS_UART_ERROR,          /* UART 操作错误 */
    LOG_STATUS_SERIAL_HEADER_ERROR, /* 串口帧头错误 */
    LOG_STATUS_SERIAL_LENGTH_ERROR, /* 串口长度错误 */
    LOG_STATUS_SERIAL_CRC_ERROR,    /* 串口 CRC 校验错误 */
    LOG_STATUS_SERIAL_TAIL_ERROR,   /* 串口帧尾错误 */
    LOG_STATUS_SERIAL_BUFFER_OVERFLOW, /* 串口缓冲区溢出 */
    LOG_STATUS_SERIAL_UART_ERROR,   /* 串口 UART 操作错误 */

    /* ROS 相关错误码 */
    LOG_STATUS_ROS_INIT_ERROR,      /* ROS 初始化错误 */
    LOG_STATUS_ROS_PARAM_ERROR,     /* ROS 参数错误 */
    LOG_STATUS_ROS_PUBLISH_ERROR,   /* ROS 发布错误 */
    LOG_STATUS_ROS_SUBSCRIBE_ERROR, /* ROS 订阅错误 */
    LOG_STATUS_ROS_SERIAL_OPEN_ERROR, /* ROS 串口打开错误 */
} LogStatus_t;

/*
 * @brief  日志等级，会打印在日志前缀中。
 * @note   该枚举用于表示日志的严重程度。
 */
typedef enum
{
    LOG_LEVEL_DEBUG = 0,            /* 调试信息 */
    LOG_LEVEL_INFO,                 /* 普通信息 */
    LOG_LEVEL_WARN,                 /* 警告信息 */
    LOG_LEVEL_ERROR                 /* 错误信息 */
} LogLevel_t;

/*
 * @brief  日志输出回调函数类型。
 * @param  level: 日志等级。
 * @param  message: 已格式化的日志字符串。
 * @note   该回调用于将 common 日志输出适配到 UART、RTT、SWO 或 ROS2 日志接口。
 */
typedef void (*LogOutputCallback_t)(LogLevel_t level, const char *message);

#ifdef __cplusplus
extern "C" {
#endif

/*
 * @brief  将通用返回码转换为字符串。
 * @param  status: 要转换的返回码。
 * @retval 返回码对应的字符串。
 * @note   该函数便于日志打印和问题定位。
 */
const char *LogStatus_ToString(LogStatus_t status);

/*
 * @brief  将日志等级转换为字符串。
 * @param  level: 要转换的日志等级。
 * @retval 日志等级对应的字符串。
 * @note   该字符串会用于日志前缀。
 */
const char *LogLevel_ToString(LogLevel_t level);

/*
 * @brief  设置日志输出回调函数。
 * @param  callback: 日志输出回调函数指针，传入 NULL 表示关闭日志输出。
 * @retval None
 * @note   common 层不直接依赖具体平台，由上层注册实际输出方式。
 */
void LOG_SetOutputCallback(LogOutputCallback_t callback);

/*
 * @brief  获取当前日志输出回调函数。
 * @param  None
 * @retval 当前日志输出回调函数指针。
 */
LogOutputCallback_t LOG_GetOutputCallback(void);

/*
 * @brief  按指定日志等级输出格式化日志。
 * @param  level: 日志等级。
 * @param  format: printf 风格的格式字符串。
 * @retval LogStatus_t: 日志写入结果。
 * @note   若未注册输出回调，函数仍会完成格式化检查并返回对应状态。
 */
LogStatus_t LOG_Write(LogLevel_t level, const char *format, ...);

/*
 * @brief  使用 va_list 按指定日志等级输出格式化日志。
 * @param  level: 日志等级。
 * @param  format: printf 风格的格式字符串。
 * @param  args: 可变参数列表。
 * @retval LogStatus_t: 日志写入结果。
 * @note   该函数用于封装 LOG_Debug、LOG_Info、LOG_Warn 和 LOG_Error。
 */
LogStatus_t LOG_WriteVa(LogLevel_t level, const char *format, va_list args);

/*
 * @brief  输出 DEBUG 等级日志。
 * @param  format: printf 风格的格式字符串。
 * @retval LogStatus_t: 日志写入结果。
 */
LogStatus_t LOG_Debug(const char *format, ...);

/*
 * @brief  输出 INFO 等级日志。
 * @param  format: printf 风格的格式字符串。
 * @retval LogStatus_t: 日志写入结果。
 */
LogStatus_t LOG_Info(const char *format, ...);

/*
 * @brief  输出 WARN 等级日志。
 * @param  format: printf 风格的格式字符串。
 * @retval LogStatus_t: 日志写入结果。
 */
LogStatus_t LOG_Warn(const char *format, ...);

/*
 * @brief  输出 ERROR 等级日志。
 * @param  format: printf 风格的格式字符串。
 * @retval LogStatus_t: 日志写入结果。
 */
LogStatus_t LOG_Error(const char *format, ...);

#ifdef __cplusplus
}
#endif

/*
 * @brief  兼容小写错误日志接口。
 * @note   该宏用于兼容已有代码中的 log_error("message") 写法。
 */
#define log_error(message) LOG_Error("%s", (message))

#endif /* COMMUNICATION_COMMON_H */
