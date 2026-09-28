# EasyTVC Studio + firmware core

Open **EasyTVC Studio.app** in Finder, or double-click **Open EasyTVC Studio.command**.
Studio runs locally on your Mac and opens in your usual browser, including Safari.
No account, cloud service, npm, or Python packages are required. The app needs Python
3.9+; the Python installation already present on this Mac is supported.

**This is a working tuning/simulation application and tested portable controller,
not completed flight firmware for the physical board.** The photos identify
EasyTVC v0.1 / STM32F405RGT6, but do not establish its electrical pin map.
There is no operational sensor/servo/USB board adapter yet. No board was flashed.
Do not upload the blank image expecting working TVC.

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

The ARM build produces `build-arm/libeasytvc_core.a` and
`build-arm/easytvc_safe_blank.bin`. **The archive contains control/runtime code;
the executable blank image only sleeps.** There is no flight image to upload yet.
The complete Arm GNU 14.3.Rel1 toolchain is installed locally under `.tools/` on
this Mac. For another installation, set `ARM_TOOLCHAIN_DIR` to a complete Arm GNU
toolchain containing newlib. The Homebrew compiler alone lacks those headers.

## What is needed to finish the physical firmware

1. The EasyTVC v0.1 schematic or a continuity-verified map of sensor buses/chip
   selects, servo pins/timers, clock, USB, arm input, and output-off states.
2. Servo model, channels, measured pulse endpoints, linkage direction, gimbal
   geometry, and board orientation in the vehicle.
3. Board-specific sensor acquisition, USB CDC, PWM, watchdog, calibration, and
   power-loss-safe settings storage, followed by powered bench tests.

Photos alone cannot establish these nets. The manufacturer's public ArduPilot
fork did not contain an EasyTVC target when checked on 2026-09-27; the older
Megadingus FC2.1 uses an RP2040 and its pin map is not applicable.

Pyro outputs are compiled out. This project is **not a validated recovery system**.

See [Mac and USB instructions](docs/usb-and-studio.md),
[architecture and board adapter contract](docs/software.md),
[protocol](docs/protocol.md), [hardware evidence](docs/hardware-evidence.md),
and [verification results](docs/verification.md).
