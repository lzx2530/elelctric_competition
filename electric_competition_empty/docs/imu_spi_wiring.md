# IMU SPI 接线

IMU 已从 I2C1 迁移到 SPI1；OLED 继续独占 I2C1 的 `PB2/PB3`。

| SPI1 信号 | TI 引脚 / 排针 | IMU 模块引脚 |
| --- | --- | --- |
| SCLK | `PB9 / J1.7` | `SCL` 或 `SCK` |
| MOSI | `PB8 / J2.15` | `SDA` 或 `MOSI` |
| MISO | `PB7 / J2.14` | `AD0` 或 `SDO` |
| CS | `PB23 / J1.3` | `NCS` |
| 电源 | `3.3V` | `VCC` |
| 地 | `GND` | `GND` |

- `AD0` 在 SPI 模式是数据输出，必须接 `PB7`，不能再接地。
- `NCS` 必须接 `PB23`，不能再直接拉到 `3.3V`。
- `INT`、`EDA`、`ECL` 暂不使用，不接。
- 按键启动已由 K230 任务帧替代，`PB7/PB8` 不再用于按键。
