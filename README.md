# Communication

该目录用于存放项目通信相关代码。目前公共定义位于 `common` 目录，串口通信代码位于 `Serial` 目录。

## 目录结构

- `common`：通信模块共用返回码、日志等级和日志输出接口。
- `Serial/Serial_32`：STM32 端串口收发与数据解析代码。
- `Serial/Serial_ROS2`：ROS2 端串口代码预留目录，当前相关源文件为空。

## common

`common/include/common.h` 保留 `LogStatus_t` 作为通信模块通用返回码，同时提供 `LogLevel_t` 日志等级和 `LOG_Debug/LOG_Info/LOG_Warn/LOG_Error` 格式化日志接口。

日志输出不直接绑定具体平台。使用方通过 `LOG_SetOutputCallback()` 注册输出回调：STM32 端可以接 UART、SWO/ITM 或 RTT，ROS2 端可以在回调中转接 `RCLCPP_*` 日志宏。

## Serial_32

`Serial_32` 面向 STM32F103C8T6，使用 STM32 HAL UART 接口。当前代码文件：

- `include/main.h`：基础头文件。
- `include/serial.h`：串口宏定义、全局变量声明和函数声明。
- `src/Serial.c`：UART1/UART2 接收状态机、CRC 校验和接收回调入口。

### UART 用途

- `USART1`：用于无人机与地面端通信。
- `USART2`：用于 STM32 与串口屏通信，后续可接入日志输出。

STM32F103C8T6 常见引脚映射：

- `USART1`：默认 `TX PA9`、`RX PA10`；重映射后 `TX PB6`、`RX PB7`。
- `USART2`：`TX PA2`、`RX PA3`。

`Serial.c` 原始注释中写有 `USART1 RX B15 / TX B14`，该引脚组合未在当前工程中验证；集成时以 STM32CubeMX 配置和实际原理图为准。

### 帧格式

`USART1` 接收帧：

```text
0x0F 0xF0 LEN DATA... CRC16_LOW CRC16_HIGH 0xFF
```

- `LEN`：数据区长度，最大受 `SERIAL1_MAX_DATA_LEN` 限制。
- `DATA`：原始数据区字节，由上层按业务协议继续解析。
- `CRC16_LOW CRC16_HIGH`：采用 CRC16/MODBUS，覆盖 `HEADER1 + HEADER2 + LEN + DATA...`，低字节在前。
- `0xFF`：帧尾。

`USART2` 接收帧：

```text
0xFF 0x0F DATA 0xFE
```

### 已完成的优化

- 补齐 `serial.h`，集中定义新版帧格式、解析结构体、缓冲区长度和函数声明。
- 使用 `LogStatus_t` 作为串口模块统一返回码，并增加 `LOG_STATUS_SERIAL_*` 错误码。
- 实现 `Serial_CalcCrc16()`，采用 CRC16/MODBUS。
- 实现 `Serial_ParseByte()`，按字节解析 UART1 新版帧。
- UART1 接收中断入口只推进状态机并置位接收完成标志。
- UART2 保留 `0xFF 0x0F DATA 0xFE` 简单帧解析。
- 给 UART 回调增加空指针保护，并避免在中断中执行阻塞日志输出。

### 已验证内容

已使用 host 测试方式验证不依赖 HAL 的公共解析逻辑：

- `common` 日志和状态码测试：通过。
- `Serial_ParseByte()` 新版帧解析测试：通过。
- `Serial_CalcCrc16("123456789") == 0x4B37`：通过。
- `SERIAL1_MAX_DATA_LEN=4U` 覆盖测试：长度溢出返回 `LOG_STATUS_SERIAL_LENGTH_ERROR`。
- 默认 `SERIAL1_MAX_DATA_LEN=255U` 编译检查：通过。

### 未验证内容

当前目录未包含完整 STM32CubeMX/STM32F1 HAL 工程、启动文件、链接脚本和芯片配置，因此尚未验证 STM32F103C8T6 真实固件构建与上板运行。

集成到 STM32F103C8T6 工程时，需要确保工程已正确引入 STM32F1 HAL，例如常见 CubeMX 工程中的 `stm32f1xx_hal.h`、`usart.c/.h` 和 `UART_HandleTypeDef huart1/huart2`。
