# Open Studio and use USB

## On this Mac

Double-click `EasyTVC Studio.app` in the project folder. It opens a local page in
your usual browser. `Open EasyTVC Studio.command` is a terminal-based alternative.
No internet access is needed to edit, simulate, or save profiles.

1. Enter measured vehicle/actuator values under Model assumptions. Example values
   are supplied only to demonstrate the tools.
2. Set the two axes' PID parameters and actual servo channel/endpoints/direction.
3. Enter sustained wind, peak gust, temperature, and established operating limits.
   Do not mark limits validated without supporting tests.
4. Run simulation. Examine tilt, output saturation, and fault behavior. Hover over
   the plot to inspect samples. Export results as CSV.
5. Save profile downloads JSON; Import validates it in Python and C. Invalid
   profiles cannot replace the current one. Edits are not autosaved.

Quit through the sidebar or Command-Q on the launcher app. Closing the browser
tab leaves the service running until you quit the app. With the `.command`
launcher, Control-C also stops it. Rebuild after changes with:

```sh
./scripts/build-studio.sh
```

The app targets this Mac's CPU and needs Python 3.9+ on a destination Mac.
It is locally signed, not notarized for general distribution.

## Two USB modes

| Mode | Purpose | What you see |
| --- | --- | --- |
| STM32 ROM DFU | Program internal flash | ST `0483:df11`, shown by `dfu-util -l` |
| Application USB CDC | Settings and telemetry | A port such as `/dev/cu.usbmodem…` |

The **blank image has no USB CDC implementation**. Studio cannot connect to it
or ROM DFU. Its CDC client and protocol have been tested against the C host fixture;
hardware needs a board adapter. Other firmware, including ArduPilot, does not
speak this protocol and is rejected during the handshake.

On compatible firmware, read settings first, then edit/save while idle and
physically disarmed. Studio waits for persistence acknowledgement and reads back
the whole packet. A failed acknowledgement/read-back means the save is unverified;
reconnect and read before retrying. A disconnected UI or old telemetry does not
establish that the physical board is disarmed.

## DFU programming

The utility accepts reviewed raw binaries linked at `0x08000000`. It is not a
source of a working board image. Do not use a binary for another board or one
linked after an application bootloader.

Use a USB-C **data cable**. During bring-up, disconnect ignition/recovery loads
and start with USB power only. BOOT/RST are visible in the photos; verify their
bootloader-entry sequence with board documentation. This work did not connect to,
program, or reset the board.

Read-only operations:

```sh
python3 scripts/dfu.py list
python3 scripts/dfu.py inspect path/to/reviewed-firmware.bin
```

After reviewing the image and board compatibility, this explicit command writes
it. Replace the placeholders with the actual image and its inspected full hash:

```sh
python3 scripts/dfu.py flash path/to/reviewed-firmware.bin \
  --board easytvc-v0.1 --sha256 FULL_64_CHARACTER_REVIEWED_HASH
```

Add `--serial DEVICE_SERIAL` for multiple attached STM32 DFU devices. The utility
selects internal flash, stages the reviewed bytes, downloads without leaving DFU,
uploads exactly the written region, and compares every byte. It stops on failure
without starting the application. Reset manually when the bench setup is ready.
An interrupted write may leave a partial application; use ROM DFU recovery and
do not assume the previous application still exists.

`easytvc_safe_blank.bin` only sleeps. Uploading it requires `--allow-blank` and
replaces the existing application with one that has no TVC, sensors, USB serial,
or deployment system. Do not flash it to obtain Studio functionality.

Byte verification establishes transfer integrity, **not flight reliability**.
ROM's USB identity does not prove the PCB model. `--board` records the operator's
selection; it does not electronically identify EasyTVC.

## References

- [ST AN2606: system memory boot mode](https://www.st.com/resource/en/application_note/an2606-stm32-microcontroller-system-memory-boot-mode-stmicroelectronics.pdf)
- [dfu-util command reference](https://dfu-util.sourceforge.net/dfu-util.1.html)
- [EasyTVC specification](https://mr-wongs-shenanigans.com/products/easytvc)
- [ArduPilot PID](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AC_PID/AC_PID.cpp), inspected for control practices; no source copied.
- [Manufacturer's older FC2.1](https://github.com/Brian-179/Megadingus-FC): RP2040, not this board's pin map.
