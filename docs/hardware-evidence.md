# Hardware evidence register

This document separates observed facts from assumptions. Only entries marked
**confirmed** can later become board configuration.

| Item | Status | Evidence | Pin/net assignment |
| --- | --- | --- | --- |
| MCU | confirmed | STM32F405RGT6 stated by board specification and package | No GPIO assignment implied |
| USB recovery | confirmed | macOS enumerated ST DFU ROM bootloader, VID:PID `0483:df11` | Bootloader transport only |
| NOR flash | confirmed as a distinct device | Front photo shows a 25Q128-family SOIC-8 device | Bus and CS unconfirmed |
| microSD socket | confirmed as a distinct device | Rear photo shows a microSD socket | Bus and CS unconfirmed |
| BMI088 | component specified | No reliable net tracing performed | Bus, CS, interrupts unconfirmed |
| BME280 | component specified | No reliable net tracing performed | Bus, address, CS unconfirmed |
| RGB LED | package present | Six-pad RGB package visible on front photo | Driver type and pins unconfirmed |
| P1/P2/P3 | confirmed terminals with separate nearby driver circuits | Front photo | MCU inputs and active polarity unconfirmed |
| SW | confirmed terminal | Front photo | Hardware function and any MCU sense net unconfirmed |
| RC connector | confirmed labels `GND`, `5V`, `RX4`, `TX4` | Front silkscreen | MCU USART pins unconfirmed |
| GNSS/mag connector | confirmed labels `3.3V`, `5V`, `TX2`, `RX2`, `SCL2`, `SDA2`, `GND` | Front silkscreen | MCU UART/I2C pins unconfirmed |
| Servo connectors | connectors present | Board photos | Signal pins/timers unconfirmed |

## Prohibited inferences

- A silkscreen label such as `RX4` or `SCL2` does not prove the STM32 pin,
  USART/I2C peripheral, electrical level, or whether a level shifter exists.
- Presence of microSD and NOR flash does not prove they share an SPI bus.
- The RGB package must not be treated as WS2812-compatible without proving its
  part and net routing.
- A nearby pyro driver is not evidence of its logic polarity or boot-state
  behaviour.

## 2026-09-27 inspection

Front/back photos in `Pasted 2026-09-27 … .png` show EasyTVC **v0.1**, STM32F405,
USB-C, BOOT/RST, and four servo headers. No reliable MCU-to-net mapping can be
established from these photographs alone.

The [specification](https://mr-wongs-shenanigans.com/products/easytvc) lists
STM32F405RGT6, BMI088, BME280, W25Q128JVS, and four servo channels. All paths and
branches in [Brian-179/ardupilot](https://github.com/Brian-179/ardupilot) were checked
without finding an EasyTVC target. The public
[Megadingus FC2.1](https://github.com/Brian-179/Megadingus-FC) uses RP2040. No pin
assignments have been promoted to confirmed using those references.

Needed evidence: schematic/continuity map, oscillator specification,
sensor-to-body orientation, servo models/linkage/power, and SW's electrical
function. Record each net before implementing its hardware initialization.
