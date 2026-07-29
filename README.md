# Communication

该目录用于存放项目通信相关代码。目前公共定义位于 `common` 目录，串口通信代码位于 `Serial` 目录。

## 目录结构

- `common`：通信模块共用返回码、日志等级和日志输出接口。
- `Serial/Serial_32`：STM32 端串口收发与数据解析代码。
- `Serial/Serial_ROS2`：ROS2 端串口代码，当前包含 `mission_bridge` 功能包（车端串口协议栈与 ROS2 桥接节点）。

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

## Serial_ROS2（mission_bridge）

`Serial/Serial_ROS2/mission_bridge` 是 ROS2（foxy，ament_cmake，C++17）功能包，实现车端（ESP8266 串口透传）与 ROS2 之间的协议栈桥接节点。传输层帧格式与 STM32 端一致（`0x0F 0xF0 LEN DATA... CRC16_LO CRC16_HI 0xFF`），直接复用 `common` 和 `Serial_32` 的解析代码（`Serial.c` 以 `-DSERIAL_HOST_TEST` 编译脱离 STM32 HAL）。

### 包结构

- `include/mission_bridge/protocol.hpp` / `src/protocol.cpp`：应用层消息类型、phase 值、负载编解码与线帧组帧。
- `include/mission_bridge/serial_port.hpp` / `src/serial_port.cpp`：最小 POSIX termios 串口封装（8N1 无流控，9600~921600），打开接口风格与 `serial_driver_ros` 的 `SerialComm(port, baudrate)` 一致。
- `src/mission_bridge_node.cpp`：桥接节点（读线程 + 去重 + session 隔离 + 心跳链路监测）。
- `config/mission_bridge.yaml`：节点参数配置，风格参照 `serial_driver_ros/config/serial_config.yaml`。
- `launch/mission_bridge.launch.py`：launch 入口，从 YAML 加载参数，参照 `serial_driver_ros/launch/serial_driver.launch.py`。

> 与 `Serial/serial_driver_ros` 的关系：该子模块是其作者的演示驱动，帧格式为 `0x0F 0xF0/0xFF LEN DATA SUM8`（float 缩放 int16 数组 + 1 字节求和校验），与本仓库和 STM32 端约定的 CRC16/MODBUS 帧不兼容，且依赖需源码安装的 `serial` 库，因此子模块保留 `COLCON_IGNORE` 不参与构建；`mission_bridge` 仅参考其 YAML 参数配置和 launch 组织方式，串口层用零依赖的 termios 封装实现。

### 应用层协议（第0步）

所有消息 DATA 区前 3 字节为公共头 `MSG_TYPE(1B) | SESSION_ID(1B) | SEQ(1B)`，随后为各类型负载（多字节字段全部小端）：

| MSG_TYPE | 名称 | 负载 |
| --- | --- | --- |
| 0x01 | START | mission_id u32（任务编号；session ID 在公共头） |
| 0x02 | CAR_STATE | phase u8（0=IDLE 1=STARTUP 2=RUNNING 3=ARRIVE_A 4=ARRIVE_B 5=ARRIVE_C 6=ARRIVE_D 7=COMPLETED） |
| 0x03 | CAR_PROGRESS | mileage f32 + speed f32（里程 m、速度 m/s） |
| 0x04 | HEARTBEAT | hb_seq u16 + time_ms u32 + link_status u8 |
| 0x05 | PAYLOAD_RELEASE | release_id u8 |
| 0x06 | PAYLOAD_ACK | acked_type u8 + acked_seq u8 + result u8 |
| 0x07 | MISSION_ABORT | reason u8 |

### 话题接口（全部为发布者，std_msgs）

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/mission/start` | UInt32 | 收到 START 帧时事件发布 mission_id |
| `/car/progress` | Float32 | 里程（米），CAR_PROGRESS 事件转发 + 10Hz 定时重发 |
| `/car/speed` | Float32 | 速度（m/s），CAR_PROGRESS 事件转发 + 10Hz 定时重发 |
| `/car/phase` | UInt8 | CAR_STATE 事件发布 + 按心跳周期定时重发 |
| `/mission/state` | String | 10Hz 紧凑单行状态，如 `link=up session=2 phase=RUNNING progress_m=12.34 speed_mps=1.23 hb_seq=41 rx=123 dup=2 crc_err=0` |
| `/mission/fault` | String | 故障事件发布，如 `LINK_DOWN: heartbeat timeout after 3 beats`、`MISSION_ABORT: reason=3`、`SERIAL_LOST: read error, reconnecting` |

### 参数

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `serial.port` | string | `/dev/ttyUSB0` | 串口设备路径 |
| `serial.baudrate` | int | `115200` | 波特率（9600~921600） |
| `serial.reconnect_interval_ms` | int | `2000` | 断线重连间隔 |
| `protocol.heartbeat_period_ms` | int | `1000` | 心跳周期（也用作 phase 重发周期） |
| `protocol.heartbeat_timeout_beats` | int | `3` | 心跳超时拍数，超时判定为 LINK_DOWN |
| `protocol.dedup_window` | int | `32` | (session, seq) 去重环形窗口大小 |

### 构建与运行

```bash
cd /home/steam/px4/communication
source /opt/ros/foxy/setup.bash
colcon build --packages-select mission_bridge
source install/setup.bash
ros2 launch mission_bridge mission_bridge.launch.py
```

串口等参数在 `config/mission_bridge.yaml` 中修改（安装后位于 `share/mission_bridge/config/`）；临时覆盖可追加 ROS 参数，例如 `ros2 launch mission_bridge mission_bridge.launch.py --ros-args -p serial.port:=/dev/ttyUSB1`。

### 节点核心行为

- 专用读线程阻塞读串口，逐字节喂 `Serial_ParseByte()`；CRC 等帧错误只计数。读失败发布 `SERIAL_LOST` fault 并按 `reconnect_interval_ms` 重连（打开失败用节流告警避免刷屏）。
- 维护最近 N 个 (session, seq) 的环形去重窗口（N=`dedup_window`），重复帧丢弃并计 dup。
- session 隔离：START 帧总是采纳其 session（替换旧 session 时重置去重窗口并告警）；非 START 帧在 session 不匹配时丢弃，未设置时从首个有效帧采纳。
- 心跳超时（默认 3s）未收到 HEARTBEAT → link 置 down 并发布一次 `LINK_DOWN` fault；收到心跳恢复 up。链路判定只看 HEARTBEAT 帧。
- 收到 PAYLOAD_RELEASE 立即回 PAYLOAD_ACK（acked_type=0x05、acked_seq=对端 SEQ、result=0x00）。
- 收到 MISSION_ABORT 发布 fault 并记日志。
