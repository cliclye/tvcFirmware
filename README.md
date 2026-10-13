# EasyTVC Studio + firmware core

Open **EasyTVC Studio.app** (keep it inside this folder). It is a native Mac window with:
**Rocket setup** (edits `Core/Inc/vehicle_config.h`, mass / thrust-to-weight / apogee / parachute
calculator), **Tuning lab** (simulation), **Build & flash** (builds both images, flashes over USB
DFU with read-back verification), **Live board** (telemetry via a USB-UART on TX2/RX2, tilt view,
automatic nose-axis detection), **Launch checklist** (bench → prep → pad, GO/NO-GO, launch-day kit)
and **Flight logs** (reads sessions off the SD card, timeline and charts, CSV export).
Rebuild the app with `./scripts/build-studio.sh`; it needs `brew install cmake dfu-util`.
No account, cloud service, npm, or Python packages are required. The app needs Python
3.9+; the Python installation already present on this Mac is supported.

**Flight firmware: see [docs/flight-guide.md](docs/flight-guide.md)** for the parts list,
wiring, bench checks and pad procedure. Two images are built: `easytvc_bench` (flash first; the
gimbal follows the board by hand) and `easytvc_flight` (arms on the pad, TVC during boost,
parachute servo at apogee). Your rocket's settings are in `Core/Inc/vehicle_config.h`.
Nothing has been run on hardware yet, so the bench checks are mandatory.

The sections below describe the Studio tuning/simulation app.

**Historical note:** this was originally a tuning/simulation application and tested portable
controller, not completed flight firmware for the physical board. EasyTVC v0.1 pin nets
from Brian Wong's hwdef are in `board/easytvc.hwdef` and `Core/Inc/board_pins.h`.
There is still no operational sensor/servo/USB board adapter. No board was flashed.
Do not upload the blank image expecting working TVC. BME280 needs STM32 I2C
pull-ups. Pyro is untested by the vendor and remains compiled out.

## What you can use now

- Separate PID, derivative filter, slew, and integral limits for the two gimbal axes.
- Servo channel selection, direction, center, and pulse endpoints.
- Wind/gust/temperature inputs and explicitly entered operating limits.
- An interactive response plot and CSV export using the **same C controller** as
  the portable firmware. This is a simplified rotational model, not a flight predictor.
- Validated JSON profiles you can save and import.
- USB serial settings/telemetry client with checksums, timeouts, state checks, and
  settings read-back. This becomes usable on hardware after a compatible USB CDC
  board adapter is implemented. The included blank image cannot connect to Studio.
- USB DFU image inspection, explicitly requested download, and byte-for-byte
  read-back verification through `scripts/dfu.py`.

No tuning can guarantee operation in any weather. Defaults are examples, not
validated gains for your rocket. Thrust, inertia, linkage, servo response, wind
torque, and environmental protection determine the usable operating envelope.

## Build and verify

```sh
./scripts/build-studio.sh
python3 -m unittest discover -s tests -p 'test_studio.py' -v
./scripts/build-firmware.sh
```

The first command builds the native controller, runs C tests, and packages the
Mac app. `python3 studio/studio.py` also starts the interface directly.
Use **Quit Studio** in the sidebar or quit the Mac app to stop its local service.
Save a profile before closing; unsaved edits are not automatically persisted.

The ARM build produces `build-arm/easytvc_bench.bin` and `build-arm/easytvc_flight.bin`
(see [docs/flight-guide.md](docs/flight-guide.md)).
The complete Arm GNU 14.3.Rel1 toolchain is installed locally under `.tools/` on
this Mac. For another installation, set `ARM_TOOLCHAIN_DIR` to a complete Arm GNU
toolchain containing newlib. The Homebrew compiler alone lacks those headers.

## What is needed to finish the physical firmware

1. A board adapter that programs the vendor pin map: 16 MHz HSE, SPI1 BMI088,
   I2C1 BME280 with internal pull-ups, USART2/UART4, USB FS, and TIM3/TIM4 PWM.
   Leave pyro and SWD pins (`PA13`/`PA14`) untouched until a dedicated gate.
2. Servo model, channels, measured pulse endpoints, linkage direction, gimbal
   geometry, and a bench check of `ROLL_180_YAW_90`.
3. USB CDC, watchdog, calibration, power-loss-safe settings storage, and
   powered bench tests. SW's electrical function is still unknown.

Pyro outputs are compiled out. This project is **not a validated recovery system**.

See [Mac and USB instructions](docs/usb-and-studio.md),
[architecture and board adapter contract](docs/software.md),
[protocol](docs/protocol.md), [hardware evidence](docs/hardware-evidence.md),
and [verification results](docs/verification.md).
