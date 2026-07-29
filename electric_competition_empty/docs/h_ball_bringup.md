# H 题联调清单

## K230 标定

1. 将控制相机固定后，使全长 25 cm 摆杆始终进入 640 x 360 画面。
2. 分别把钢球置于 `-100 mm`、`0 mm`、`+100 mm` 刻度，记录 YOLO 叠加框中心的 `pixel_x`。
3. 将三组数据写入 K230 的 `beam_position_calibration.json` 的 `points` 数组。未完成该步骤时，K230 会发送无效球位置，主控不会启动平衡任务。
4. 确认 K230 每约 33 ms 发送一次 `BALL_REPORT (0x12)`；有效帧包含位置、置信度、框尺寸和稳定帧数。

## MSPM0 SysConfig

不要手动编辑 `empty.syscfg`。使用 SysConfig 为以下输入分配未占用 GPIO/定时器捕获资源：

- `START`：内部上拉，按下接地；短按调用 `app_mission_start(tick_ms)`。
- `MODE`：内部上拉，按下接地；短按调用 `app_mission_next_mode()`。
- 步进编码器 PWM：捕获高电平计数与周期计数，并在每个有效周期调用 `app_ball_control_set_actuator_pwm(high_ticks, period_ticks)`。

`app_ball_control` 会在 PWM 编码器未接入、超范围或视觉超过 120 ms 未更新时停止摆杆步进并报告故障，避免开环失控。

## 调参顺序

1. 断开车轮，只测 PWM 编码器归一化位置是否在 `0.0..1.0` 单调变化。
2. 架空摆杆，确认 `APP_BALL_COMMAND_SIGN` 与实际钢球修正方向一致；反向时仅修改该符号。
3. 静止完成 `+50 mm` 到 `-50 mm`；先调 `g_ball_pid_cfg.kp`，再增加少量 `kd`，最后才调 `ki`。
4. 空车完成一圈、A 点启停线识别和编码器里程；再以 `0.25 m/s` 带球测试。
5. 最后启用 IMU 加速度前馈，按实车方向校验纵向加速度符号。
