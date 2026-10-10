# Firmware review: bugs found and fixed

Scope: everything added since the "Firmware Update" commit (register-level drivers, `main.c`,
SD logging). The portable flight core (`app`, `controller`, `flight_fsm`, `pid`, ...) already had
tests and was left alone. **Nothing here has run on a real board** - see "Not verified".

## Fixed

| Area | Bug | Effect |
|---|---|---|
| Build | Hardware drivers were in the host source list | `host-test.sh` failed to compile; no tests ran |
| Clock | `PPRE1=4<<10` is /2 (not /4); `PPRE2=2<<13` is /1 (not /2) | APB1 at 84 MHz and APB2 at 168 MHz, both above spec; every baud/bus/timer constant wrong |
| Clock | PLLQ left 0; HSE (specified 16 MHz crystal) unused | invalid PLL config; USB could never get 48 MHz |
| Header | `AHB2PERIPH_BASE = 0x40000000 + 0x50000000` | USB OTG address wrong |
| GPIO | PA14 (SWCLK) configured as a GPIO output | debugger access lost after boot |
| GPIO | I2C pins push-pull, not open-drain | I2C bus contention; BME280 unusable |
| GPIO | Pyro output nets left as floating inputs | floating gate on a pyro channel |
| GPIO | USB pins given pull-ups | wrong for USB FS |
| PWM | `OCxM_PWM1 << 4` applied twice (macros already shifted) | outputs not in PWM mode; CH4 configured as an input |
| PWM | No update event before start | first period used unloaded shadow registers |
| UART | BRR = 5833 (forgot the x16 oversampling) | ~7200 baud instead of 115200 |
| UART | Blocking 120-byte send in the control loop | ~10 ms stall per telemetry line |
| BMI088 | R/W bit inverted (reads sent as writes and vice versa) | no valid sensor read |
| BMI088 | Accelerometer dummy byte not skipped | accel data shifted by one byte |
| BMI088 | Wrote 0x00 to ACC_PWR_CTRL | accelerometer switched off |
| BMI088 | Gyro "config" wrote to INT_CTRL, range/bandwidth never set; no scaling | raw counts treated as physical units |
| BME280 | 7-byte read into a 6-byte slot | stack overflow by one byte |
| BME280 | `dig_H6` taken from wrong byte, signed values unsigned | humidity wrong |
| BME280 | Pressure formula was a mangled 64-bit variant on `int32_t` | overflow, garbage pressure/altitude |
| BME280 | `CTRL_HUM` never written; no skipped-sample / range check | invalid humidity; garbage accepted |
| I2C | No STOP / flag clear after NACK; 0x76 then 0x77 tried every call | one failed probe wedged the bus |
| I2C | 1-byte and multi-byte receive sequences wrong | extra byte clocked, race on ACK/STOP |
| SPI | SD card initialised at 21 MHz (must be <= 400 kHz) | card never initialises |
| SD | CS toggled during the 80 clocks, no OCR/CCS check, R7 ignored | SDHC vs SDSC addressing wrong |
| SD | `while (token != 0xFE)` and `while (busy)` with no timeout | one bad card hangs the vehicle |
| Logger | Periodic flush reset the sector offset without advancing | each second overwrote the last: almost the whole log lost |
| Logger | Blocking multi-ms SD write inside the loop | breaks the 20 ms control dt limit |
| Logger | `accel*1000` and `temperature*100` cast to `int16_t` | overflow / garbage values |
| main | "10 ms" timer was a `nop` loop (actually ~15 ms+) | timestamps wrong, rate unknown |
| main | Servos centred forever; flight stack never called | no estimator/controller |
| main | One failed sensor read disabled the sensor permanently | no recovery |
| main | Negative values in (-1,0) printed without the sign | wrong telemetry |
| Startup | Hard fault -> `wfi` forever; 16-entry vector table; no watchdog | frozen servos in flight |
| Viewer | `H` (unsigned) temperature, gyro /100, ignored sector padding | all data after sector 1 misaligned |

## Not verified (needs a bench)

- Anything on real hardware: no board was available. Clock, SPI, I2C, SD and PWM are checked only
  by compiling and by reading the code against RM0090 / datasheets. Do a first power-up with the
  servos disconnected and a scope or logic analyser on the PWM and SPI pins.
- BMI088 mounting rotation (`ROLL_180_YAW_90`) is **not applied**; sensor frame != body frame.
- PA13 (pyro 2) is SWDIO, which powers up as a pulled-up input. Firmware cannot hold that net low.
  Do not connect an e-match to channel 2.
- INSTALLATION.md says to ground PA14 for DFU; on an STM32F405 that is SWCLK and DFU needs BOOT0.
- `physical_arm_qualified` is hard-wired false: there is no known arm-sense input. Until that
  exists the image cannot arm, which is intentional.
- BME280 pressure uses Bosch's 32-bit integer formula (+3 Pa vs float on the datasheet example).
