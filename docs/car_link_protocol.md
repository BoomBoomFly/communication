# 小车通信协议 v1.1（陆空协同 · 车机链路）

2026 电赛 D 题 · BoomBoomFly 队 · 工位四

本文档是小车（STM32）与机载电脑（Jetson，`mission_bridge`）之间链路的**协议规范**，
STM32 端（`Serial_32`）与 ROS2 端（`Serial_ROS2/mission_bridge`）的实现均以本文档为准。
链路只承载指令与状态，不跑任何闭环控制（小车自主循线、无人机自主飞行）。

> v1.1：与 BoomBoomFly/communication 仓库 main（PR #2/#3 之后）对齐——
> START 发布值为 mission_id 并新增 `/mission/start/context`；session 建立/替换规则收紧；
> `/mission/state` 更名 `/car/link_state`；PAYLOAD_RELEASE 现阶段仅线协议兼容
> （机端不主动发送，收到回 ACK result=0x01）；反向心跳暂不实现。

## 1. 链路拓扑与角色

```text
┌──────── 小车 ────────┐                 ┌──────── 无人机 ────────┐
│ STM32 (Serial_32)    │ UART 115200 8N1 │ Jetson                 │
│   └─ ESP8266 softAP  │═══ WiFi TCP 透传 ═► socat /dev/bbf_car    │
│      192.168.4.1     │  :8080, CIPMUX=0 │   └─ mission_bridge    │
└──────────────────────┘                 └────────────────────────┘
```

- ESP8266 为 softAP + **TCP client** 透传（`AT+CIPMUX=0`、`AT+CIPMODE=1`、
  `AT+SAVETRANSLINK=1,"192.168.4.2",8080,"TCP"`），Jetson 为 TCP server。
- 物理层对协议透明：UART 直连与 WiFi 透传字节流完全一致，协议栈代码零改动。
- 协议端点：**车端 = STM32，机端 = Jetson/mission_bridge**。
- 机端 mission_bridge **没有任何 ROS 订阅**，输入仅来自配置的串口；全部 ROS 接口为发布输出。

## 2. 传输层帧格式

```text
0x0F 0xF0 LEN DATA... CRC16_LO CRC16_HI 0xFF
```

| 字段 | 长度 | 说明 |
| --- | --- | --- |
| HEADER | 2 | 固定 `0x0F 0xF0` |
| LEN | 1 | DATA 区长度，0~255 |
| DATA | LEN | 应用层消息（见 §3） |
| CRC16 | 2 | CRC16/MODBUS（poly 0xA001，init 0xFFFF），覆盖 HEADER+LEN+DATA，**低字节在前** |
| TAIL | 1 | 固定 `0xFF` |

- CRC 测试向量：对 ASCII `"123456789"` 计算结果为 `0x4B37`。
- 接收端按字节状态机解析；帧头/长度/CRC/帧尾错误只计数（`crc_err`），不中断解析。
- 参考实现：车端 `Serial_32/src/Serial.c`、机端复用同一文件（`-DSERIAL_HOST_TEST`）。

## 3. 应用层消息

### 3.1 公共头

所有消息 DATA 区前 3 字节为公共头，随后为负载；多字节字段一律**小端**：

```text
MSG_TYPE(1B) | SESSION_ID(1B) | SEQ(1B) | PAYLOAD...
```

- `SESSION_ID`/`SEQ` 均为 u8。session 规则见 §4.2。
- **负载长度必须与 §3.2 表完全一致**；尾随字节视为非法帧（计 `invalid_len`），
  非法帧不得改变当前 session。

### 3.2 消息表

| MSG_TYPE | 名称 | 方向 | 负载 | 触发时机 |
| --- | --- | --- | --- | --- |
| 0x01 | START | 车→机 | mission_id u32 | 一键启动（任务开始），同时建立/替换 session |
| 0x02 | CAR_STATE | 车→机 | phase u8 | 阶段变化时事件发送 |
| 0x03 | CAR_PROGRESS | 车→机 | mileage f32 + speed f32 | 循线行进中 ≥10 Hz |
| 0x04 | HEARTBEAT | 车→机 | hb_seq u16 + time_ms u32 + link_status u8 | 1 Hz 周期 |
| 0x05 | PAYLOAD_RELEASE | 车→机 | release_id u8 | 现阶段仅线协议兼容保留（见 §4.4） |
| 0x06 | PAYLOAD_ACK | 机→车 | acked_type u8 + acked_seq u8 + result u8 | 机端收到 RELEASE 后应答 |
| 0x07 | MISSION_ABORT | 车→机（保留双向） | reason u8 | 任务中止 |

### 3.3 枚举值

**START.mission_id**：`0` 无效；`1` 赛题任务 1（抛投）；`2` 赛题任务 2（动态起降）；
`3` 普通垂直测试（VERTICAL_TEST）。mission_id 为 0 或不在 1..3 时**不建立/替换 session、
不发布 START**（计 `invalid_payload`）。mission_bridge 只验证 session/seq 并转发任务编号，
不负责 Arm、飞行控制、投放或动态降落。

**CAR_STATE.phase**：0=IDLE、1=STARTUP、2=RUNNING、3~6=ARRIVE_A/B/C/D、7=COMPLETED。

**PAYLOAD_ACK.result**：`0x00` = 成功；`0x01` = unsupported（现阶段机端固定回此值）。

**MISSION_ABORT.reason**：1=LINK_LOSS、2=SAFETY_STOP、3=MISSION_TIMEOUT、4=MANUAL。

**HEARTBEAT.link_status**：`0x00` = 本端正常；非零 = 本端降级（自定义）。

## 4. 可靠性与会话管理

### 4.1 (session, seq) 去重

- 接收端在活动 session 的滑动窗口内按 `(SESSION_ID, SEQ)` 去重
  （窗口大小 `protocol.dedup_window`，默认 32），命中即丢弃（计 `dup`）。

### 4.2 session 建立与替换（v1.1 收紧）

- **只有合法 START 能建立 session**（首个合法 START 建立）。
- 之后只有按模 256 **前进 1..127** 的新 session 才能替换当前 session；
  相同 session 的任何 START（包括更换 seq 或 mission_id）按重放丢弃；
  旧/模糊 session 丢弃（计 `session_drop`）。
- 非 START 帧必须属于已建立的活动 session，否则丢弃（计 `session_drop`）。
- 发送端（车端）应在一次任务开始前递增 session，且不得一次跨越 128 个 session。
- **START 不做同 session 重发**（同 session 重发会被按重放丢弃）；
  车端重试起飞指令的正规途径是递增 session 重新发起任务。
- 接受新 session 时，机端重置去重窗口与心跳状态，等待该 session 的新 HEARTBEAT。

### 4.3 START 的 ROS 输出（mission_id + context 双话题）

机端仅在接受**新 session** 的 START 时发布一次，重复/旧 session 不发布：

1. 先发布 `/mission/start/context`（`std_msgs/UInt64`），位布局：
   `mission_id[15:0] | session_id[23:16] | seq[31:24] | source_epoch[63:32]`；
2. 紧邻发布 `/mission/start`（`std_msgs/UInt32`），值为 **mission_id**。

- `source_epoch` 由 bridge 每次启动生成（非零）；`protocol.source_epoch` 参数
  仅供隔离测试显式固定，生产保持 0（自动生成）。
- 消费端（飞行状态机）必须缓存 context 的单调接收时刻，只有在 mission_id 匹配、
  context/START 接收间隔满足自身 freshness、source_epoch 符合当前 bridge 实例、
  且 `(session_id, seq)` 未见过时才接受 START——以此拒绝 bridge 重启后迟到的旧 epoch 事件。
- context 与 START 均使用默认 volatile QoS。

### 4.4 PAYLOAD_RELEASE（现阶段：线协议兼容保留）

- 普通垂直飞行阶段**不实现** payload release 执行动作和动态降落；
  机端**不主动发送** RELEASE，也没有 `/mission/payload_release` 订阅。
- 机端收到车端 RELEASE 后回 PAYLOAD_ACK（acked_type=0x05、acked_seq=对端 SEQ、
  **result=0x01 unsupported**）。
- 后续阶段若启用机→车抛投告知（release_id、ACK 匹配重传等），
  在本文档追加版本小节并同步代码，不破坏现有帧格式。

### 4.5 心跳与链路判定

- 车→机 1 Hz HEARTBEAT；机端以 **3 拍超时**（默认 3 s）判 LINK_DOWN，
  边沿发布 `/mission/fault = LINK_DOWN`；收到心跳恢复 up。链路判定只看 HEARTBEAT 帧。
- 状态串中 `serial` 仅表示设备文件连接；`link=up` 要求串口已连接**且**
  活动 session 的 HEARTBEAT 新鲜。
- 反向心跳（机→车）**现阶段不实现**；如车端需要分辨"小车没发"与"链路全断"，
  后续版本追加。

### 4.6 断线重连

- 串口打开失败/读失败：机端按 `serial.reconnect_interval_ms`（默认 2 s）重连；
  边沿发布 `SERIAL_DOWN` / `SERIAL_RECOVERED`，持续失败不刷屏。
- ESP8266 透传模式下 TCP 断开自动重连 Jetson server（`SAVETRANSLINK` 已配置）。

## 5. ROS 接口汇总（机端，全部 std_msgs 发布）

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/mission/start` | UInt32 | 新 session START 发布一次，值 = mission_id |
| `/mission/start/context` | UInt64 | START 前紧邻发布的原子上下文（位布局见 §4.3） |
| `/car/progress` | Float32 | 里程（米），事件转发 + 10 Hz 定时重发 |
| `/car/speed` | Float32 | 速度（m/s），事件转发 + 10 Hz 定时重发 |
| `/car/phase` | UInt8 | CAR_STATE 事件发布 + 按心跳周期定时重发 |
| `/car/link_state` | String | 10 Hz 状态串：`serial=up link=up session=2 mission_id=3 start_seq=41 phase=RUNNING ...` |
| `/mission/fault` | String | 边沿事件：`SERIAL_DOWN`、`SERIAL_RECOVERED`、`LINK_DOWN`、`MISSION_ABORT` |

状态串统计字段：`dup`（重复帧）、`session_drop`（未建/非活动/旧 session 丢弃）、
`crc_err`（CRC/帧尾/长度错误）、`invalid_len`（应用层长度错误）、
`invalid_payload`（mission_id 非法等负载语义错误）。

## 6. 时序与性能预算

题面 90 s 任务时间线对链路的要求：

| 时刻 | 事件 | 链路动作 | 时延预算 |
| --- | --- | --- | --- |
| t=0 | 小车一键启动 | START（建立新 session） | 起飞指令 < 200 ms |
| t≤15 s | 小车到 B 点 | CAR_STATE(ARRIVE_B) | 事件 < 100 ms |
| B→D 之间 | 伴飞、抛投 | CAR_PROGRESS ≥10 Hz | 抛投确认 < 1 s |
| t≤90 s | 回到 A 点 | CAR_STATE(COMPLETED) | — |

- 链路遥测总量 ≈ 0.5 KB/s，ESP8266 透传时延典型 10~40 ms，余量充足。
- 心跳超时（3 s）不触发任务中止，只作链路告警；MISSION_ABORT 由任务逻辑显式发送。

## 7. 默认参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| UART | 115200 8N1 无流控 | STM32 ↔ ESP8266 |
| ESP8266 | AP `BBF_BOX`，TCP client → 192.168.4.2:8080 | 信道赛前扫描选定 |
| `protocol.heartbeat_period_ms` | 1000 | 心跳周期（也用作 phase 重发周期） |
| `protocol.heartbeat_timeout_beats` | 3 | 判 LINK_DOWN |
| `protocol.dedup_window` | 32 | (session, seq) 去重窗口 |
| `protocol.source_epoch` | 0 | 0=启动自动生成；非零仅隔离测试 |
| `serial.reconnect_interval_ms` | 2000 | 机端端口重开间隔 |

## 8. 实现对照与版本

- 机端实现：`Serial_ROS2/mission_bridge`（本仓库 main，含 CTest 自动测试）。
- 车端实现：`Serial_32`（本仓库）+ 应用层消息编解码（按 §3 实现）。
- v1 → v1.1（对齐仓库 PR #2/#3）：
  1. `/mission/start` 值由 session ID 改为 **mission_id**，新增 `/mission/start/context`；
  2. session 建立/替换规则收紧为 §4.2（仅合法 START、前进 1..127、同 session 重放丢弃）；
  3. `/mission/state` 更名 `/car/link_state`，fault 字符串改为边沿事件集；
  4. PAYLOAD_RELEASE 暂缓实现（§4.4），反向心跳暂缓（§4.5）。
- 注意：工位四交付包 `docs/02_接口核对与补丁说明.md` 中的 P0-A（start 值=session ID）
  与 P1-B（机→车 RELEASE 主动发送）两项建议**与仓库现阶段决定不一致，已按仓库为准放弃**；
  P0-B（offboard_cpp 侧 `/mission/start` 类型冲突）仍需状态机负责人按 UInt32 订阅适配。
