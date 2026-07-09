/*
 * @file    test_common.c
 * @brief   common 模块单元测试文件
 * @author  Wanone111
 * @note    该文件用于验证通用返回码、日志等级和日志输出回调功能。
 */
#include "common.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static LogLevel_t captured_level;
static char captured_message[128];
static unsigned int captured_count;

/*
 * @brief  测试用日志捕获回调函数。
 * @param  level: 日志等级。
 * @param  message: 已格式化的日志字符串。
 * @retval None
 * @note   该函数用于验证 LOG_SetOutputCallback 注册后的日志输出内容。
 */
static void CaptureLog(LogLevel_t level, const char *message)
{
    captured_level = level;
    captured_count++;
    snprintf(captured_message, sizeof(captured_message), "%s", message);
}

/*
 * @brief  common 模块测试入口。
 * @param  None
 * @retval int: 返回 0 表示测试通过。
 */
int main(void)
{
    assert(strcmp(LogStatus_ToString(LOG_STATUS_OK), "OK") == 0);
    assert(strcmp(LogStatus_ToString(LOG_STATUS_CRC_ERROR), "CRC_ERROR") == 0);
    assert(strcmp(LogStatus_ToString(LOG_STATUS_SERIAL_CRC_ERROR), "SERIAL_CRC_ERROR") == 0);
    assert(strcmp(LogLevel_ToString(LOG_LEVEL_ERROR), "ERROR") == 0);

    LOG_SetOutputCallback(CaptureLog);

    assert(LOG_Info("rx length=%u", 8U) == LOG_STATUS_OK);
    assert(captured_count == 1U);
    assert(captured_level == LOG_LEVEL_INFO);
    assert(strcmp(captured_message, "[INFO] rx length=8") == 0);

    assert(LOG_Error("crc mismatch") == LOG_STATUS_ERROR);
    assert(captured_count == 2U);
    assert(captured_level == LOG_LEVEL_ERROR);
    assert(strcmp(captured_message, "[ERROR] crc mismatch") == 0);

    assert(LOG_Write(LOG_LEVEL_INFO, NULL) == LOG_STATUS_INVALID_PARAM);

    LOG_SetOutputCallback(NULL);
    assert(LOG_Warn("no output callback") == LOG_STATUS_WARN);

    return 0;
}
