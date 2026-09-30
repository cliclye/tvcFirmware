# Hardware evidence register

This document separates observed facts from assumptions. Only entries marked
**confirmed** can later become board configuration.

Vendor I/O source: Brian Wong, 2026-09-29, ArduPilot `hwdef` for EasyTVC
(`board/easytvc.hwdef`). C pin macros: `Core/Inc/board_pins.h`.

| Item | Status | Evidence | Pin/net assignment |
| --- | --- | --- | --- |
| MCU | confirmed | STM32F405RGT6 stated by board specification, package, and hwdef `STM32F405xx` | No GPIO assignment implied |
| HSE | vendor hwdef | `OSCILLATOR_HZ 16000000` | 16 MHz external oscillator |
| USB | vendor hwdef | `PA11`/`PA12` OTG FS | `PA11` DM, `PA12` DP |
| USB recovery | confirmed | macOS enumerated ST DFU ROM bootloader, VID:PID `0483:df11` | Bootloader transport only |
| NOR flash | vendor hwdef | Front photo 25Q128-family SOIC-8; hwdef SPI2 + `PC5` CS | SPI2 `PB13` SCK, `PB14` MISO, `PB15` MOSI, `PC5` CS, Mode 3, 32 MHz |
| microSD socket | vendor hwdef | Rear photo socket; hwdef SPI3 + `PC6` CS | SPI3 `PC10` SCK, `PC11` MISO, `PC12` MOSI, `PC6` CS |
| BMI088 | vendor hwdef | Specified IMU; SPI1 devices `bmi088_a` / `bmi088_g` | SPI1 `PA5` SCK, `PA6` MISO, `PA7` MOSI; accel CS `PC8`; gyro CS `PC7`; Mode 3, 10 MHz; rotation `ROLL_180_YAW_90` (bench-check before use) |
| BME280 | vendor hwdef + vendor caveat | Onboard baro; hwdef probes BMP280 on I2C0 `0x76`/`0x77` and I2C1 `0x77`. Product is BME280. **Requires STM32F4 internal I2C pull-up on the associated pins.** | I2C1 `PB6` SCL / `PB7` SDA with internal pull-up. I2C2 `PB10`/`PB11` has no hwdef pull-up. Probe 7-bit `0x76` then `0x77` |
| RGB LED | vendor hwdef | Discrete GPIO LEDs, not a smart-LED protocol | Blue ACT `PB5`, green B/E `PB4`, red `PB3`; init LOW |
| P1/P2/P3 pyro | vendor hwdef, **untested** | Outputs LOW + pulldown. Vendor: do not use with pyrotechnics until tested | Out: `PB2`, `PA13`, `PC3`. Continuity in: `PB12`, `PC4`, `PC2`. `PA13` is SWDIO |
| SW | unconfirmed | Front photo terminal | Hardware function and any MCU sense net still unconfirmed |
| RC connector | vendor hwdef | Silk `RX4`/`TX4`; UART4 | `PA0` TX, `PA1` RX |
| GNSS/mag connector | vendor hwdef | Silk `TX2`/`RX2`/`SCL2`/`SDA2`; USART2 + I2C2 | USART2 `PA2` TX, `PA3` RX; I2C2 `PB10` SCL, `PB11` SDA |
| Servo connectors | vendor hwdef | Four PWM channels | `PB0` TIM3_CH3, `PB1` TIM3_CH4, `PB8` TIM4_CH3, `PB9` TIM4_CH4 |
| Battery ADC | vendor hwdef | `BATT_VOLTAGE_SENS` | `PA4` ADC1 IN4, scale 1 (divider still unmeasured) |
| Buzzer | vendor hwdef | GPIO out | `PA8` LOW pulldown |
| DFU pin | vendor hwdef | `PINIO10` | `PA14` (SWCLK). Do not claim this pin during SWD debug |

## Vendor caveats (2026-09-29)

- There is no established EasyTVC firmware on GitHub; this hwdef is the pin list.
- BME280 requires the STM32F4 internal pull-up on its I2C pins (enable on I2C1).
- Pyro channels and continuity detection have not been fully tested. Do not use
  them with pyrotechnic systems. This firmware keeps pyro GPIO compiled out.

## Prohibited inferences

- The hwdef is the vendor net list, not a continuity measurement on this unit.
  Bring-up should still identity-probe sensors one device at a time.
- hwdef `BMP280` vs product `BME280`: probe chip id `0x60` (BME280), not `0x58`.
- `SCALE(1)` on battery ADC is not a calibrated divider.
- `ROTATION_ROLL_180_YAW_90` is ArduPilot's IMU mount; confirm on the bench
  before using it as the body-frame transform.
- `PA13`/`PA14` overlap SWD. Early images must leave them at reset state.
- Presence of microSD and NOR flash does not mean they share an SPI bus (they
  do not: SPI3 vs SPI2).
- SW is still not an arm switch unless a sense net is identified.

## 2026-09-27 inspection

Front/back photos in `Pasted 2026-09-27 … .png` show EasyTVC **v0.1**, STM32F405,
USB-C, BOOT/RST, and four servo headers. Photographs alone could not establish
MCU-to-net mapping.

The [specification](https://mr-wongs-shenanigans.com/products/easytvc) lists
STM32F405RGT6, BMI088, BME280, W25Q128JVS, and four servo channels. All paths and
branches in [Brian-179/ardupilot](https://github.com/Brian-179/ardupilot) were checked
on 2026-09-27 without finding an EasyTVC target. The public
[Megadingus FC2.1](https://github.com/Brian-179/Megadingus-FC) uses RP2040 and is
not this pin map.

Needed before a flight adapter: measured battery divider, servo models/linkage,
sensor-to-body confirmation, SW electrical function, and pyro verification with
dummy loads only.
