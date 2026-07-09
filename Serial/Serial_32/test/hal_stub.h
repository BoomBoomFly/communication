/*
 * @file    hal_stub.h
 * @brief   STM32 HAL 测试桩头文件
 * @author  Wanone111
 * @note    该文件仅用于 host 侧语法检查，不参与真实 STM32 工程编译。
 */
#ifndef SERIAL32_HAL_STUB_H
#define SERIAL32_HAL_STUB_H

#include <stdint.h>

typedef struct
{
    void *Instance;
} UART_HandleTypeDef;

#define USART1 ((void *)0x40013800U)
#define USART2 ((void *)0x40004400U)

int HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);

#endif /* SERIAL32_HAL_STUB_H */
