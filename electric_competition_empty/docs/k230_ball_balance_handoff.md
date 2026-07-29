# K230 与 MSPM0 平衡车通信约定

本文档与 K230 的 `K230下位机对接说明.md` 保持一致，适用于
`electric_competition_empty` 固件。K230 负责选题、钢球视觉与目标误差计算；
MSPM0 负责执行器闭环、底盘循迹、安全保护和 OLED 显示。

## 1. 接线

两端均使用 **3.3 V TTL、115200、8N1**，且必须共地：

| 信号 | K230 | MSPM0G3507 |
| --- | --- | --- |
| K230 发送 | UART3 TX / IO28 | UART2 RX / PB18 |
| K230 接收（预留） | UART3 RX / IO33 | UART2 TX / PB17 |
| 信号地 | GND | GND |

不要把 5 V UART 直接接入任一 GPIO。K230 每 20 ms 发送一次位置帧（约 50 Hz）。
MSPM0 当前只接收数据，不向 K230 发送控制命令。

## 2. 帧格式

所有多字节字段均为小端序：

```text
AA 55 01 type sequence payload_length payload... crc16_lo crc16_hi
```

- `version` 固定为 `0x01`。
- CRC 为 `CRC-16/CCITT-FALSE`：初值 `0xFFFF`、多项式 `0x1021`、无反射、无最终异或。
- CRC 覆盖 `version` 至 payload，不包括 `AA 55` 和 CRC 自身。

### 2.1 任务开始帧：`type = 0x13`

```text
AA 55 01 13 seq 01 flag crc16_lo crc16_hi
```

`flag` 必须为 `1..6`。MSPM0 收到此帧后会停止上一任务、清除 PID 积分与视觉状态，
然后进入对应任务；OLED 随即显示 `TASK n`。

| flag | 赛题任务 | MSPM0 动作 |
| ---: | --- | --- |
| 1 | 图传记录 | 不驱动车轮或摆杆，仅开始计时与显示 |
| 2 | 空车循迹一圈 | 启动循迹底盘 |
| 3 | `+50 mm -> -50 mm` | 等待视觉稳定后启动摆杆闭环 |
| 4 | AB 段居中 | 启动摆杆闭环和低速循迹 |
| 5 | 整圈居中 | 启动摆杆闭环和低速循迹 |
| 6 | 整圈保持启动点 | 等待参考点锁存后启动摆杆闭环和低速循迹 |

K230 的 GPIO27 确认键只应在 `flag=1..6` 时发送这一个任务开始帧。`flag=0`
仅用于 K230 调参，不发送 UART 帧。

### 2.2 钢球位置帧：`type = 0x12`

```text
AA 55 01 12 seq 05 flags error_mm_lo error_mm_hi confidence_lo confidence_hi crc16_lo crc16_hi
```

完整帧长为 **13 字节**。payload 定义如下：

| 偏移 | 字段 | 类型 | 含义 |
| ---: | --- | --- | --- |
| 0 | `flags` | `uint8` | 状态位 |
| 1..2 | `position_error_mm` | `int16` | 当前目标误差，单位 mm |
| 3..4 | `confidence_permille` | `uint16` | YOLO 置信度 × 1000 |

若没有有效钢球，K230 发送 `position_error_mm=-32768` 并清除 `VALID`；
MSPM0 必须忽略该位置值。

| 位 | 掩码 | 含义 | MSPM0 行为 |
| ---: | ---: | --- | --- |
| bit0 | `0x01` | `VALID` | 与 `IN_ROD` 同时置位才采用位置 |
| bit1 | `0x02` | `STABLE` | 用于任务启动前的稳定判定 |
| bit2 | `0x04` | `IN_ROD` | 球位于已标定水管范围内 |
| bit3 | `0x08` | `REFERENCE_READY` | 任务 6 的启动参考已锁存 |
| bit4 | `0x10` | `STOP_REQUEST` | 任务 3 到达终点，立即锁存停机 |

MSPM0 仅在 `(flags & 0x05) == 0x05` 时更新视觉位置。无有效位置超过
120 ms 会触发执行器安全停机。

## 3. 各视觉模式的位置语义

- **任务 3 / `mode_1.py`**：`position_error_mm = ball_position_mm - current_target_mm`。
  K230 完成 `+50 mm` 后会切换 `-50 mm`；误差从约 0 跳至约 `+100 mm` 是正常现象。
  到达最终 `-50 mm` 时，K230 置位 `STOP_REQUEST`，MSPM0 立即结束任务。
- **任务 4、5 / `mode_2.py`**：误差相对于水管中点，目标是 0 mm。
- **任务 6 / `mode_3.py`**：误差相对于启动点。`REFERENCE_READY` 置位前，MSPM0 不会开始任务；
  置位后启动点就是 0 mm。

第一次联调应架空执行器，观察钢球偏移后的动作方向。若方向相反，只在一侧统一修改
符号约定，不能 K230 与 MSPM0 同时取反。

## 4. 联调清单

- [ ] 确认 IO28 接 PB18、两端共地，且不是 5 V 串口。
- [ ] 确认 K230 先发一帧 `0x13`，再发送任务 3–6 所需的 `0x12` 位置帧。
- [ ] 任务 1–6 依次触发时，OLED 分别显示 `TASK 1` 至 `TASK 6`。
- [ ] 遮挡钢球时，K230 清除 `VALID`，MSPM0 在超时后停执行器。
- [ ] 任务 3 到终点时，单次 `STOP_REQUEST` 即能让执行器停机。
- [ ] 任务 6 在 K230 置位 `REFERENCE_READY` 前不应起车。
