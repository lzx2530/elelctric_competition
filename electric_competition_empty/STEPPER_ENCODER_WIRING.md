# Stepper Encoder Wiring Reservation

This note reserves four header-accessible GPIOs for two stepper-motor encoders.
The goal is to avoid using pins that are not broken out to the LaunchPad headers.

## Assumption

- Each stepper encoder provides quadrature `A/B` outputs.
- Two axes are reserved: `yaw` and `pitch`.

## Recommended Reservation

| Axis | Encoder Signal | MCU Pin | LaunchPad Header Pin | Reason |
| --- | --- | --- | --- | --- |
| Yaw | A | `PB7` | 14 | Broken out, currently unused |
| Yaw | B | `PB8` | 15 | Broken out, currently unused |
| Pitch | A | `PA13` | 31 | Broken out, currently unused |
| Pitch | B | `PB4` | 40 | Broken out, currently unused |

## Why These Pins

- They are present on the LP-MSPM0G3507 BoosterPack headers.
- They do not conflict with the current project wiring for:
  - chassis motors
  - chassis encoders
  - line-tracking sensor
  - K230 UART
  - I2C IMU/OLED
  - stepper `STEP/DIR`
- They avoid pins that are only internal/onboard or require special mux handling.

## Do Not Use For Stepper Encoders

- `PA21`, `PA24`, `PA25`: currently used by the line sensor and not convenient header choices here
- `PB16`: currently conflicts with line-sensor `AD2` and turret laser
- `PA10`, `PA11`: reserved for debug UART backchannel
- `PB2`, `PB3`: reserved for `I2C1`
- `PA8`, `PA12`, `PB12`, `PB15`: already used by stepper `STEP/DIR`

## Electrical Note

- If the encoder `A/B` outputs are `3.3V` TTL, they can be wired directly.
- If the encoder outputs are `5V` push-pull, level shifting or voltage adaptation is required before connecting to the MSPM0 GPIOs.
