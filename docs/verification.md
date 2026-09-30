# Verification record — 2026-09-27

## Completed checks

- C core/regression suite: flight states, sensor identity fixtures, attitude,
  PID, TVC bounds, RAM logging, telemetry, and the pyro compile lock.
- C reliability suite: settings round-trip, every-byte CRC corruption,
  invalid settings, axis direction, axial-spin exclusion, stale/repeated samples,
  uint32 timer wrap, non-finite inputs, excessive tilt, arm-loss faults,
  saturation anti-windup, protocol overflow/recovery, state/storage gates, and
  applying persisted runtime settings.
- Both C suites passed with Clang AddressSanitizer and UndefinedBehaviorSanitizer.
- Thirteen Python integration tests passed, covering profile validation, Python/C binary format
  compatibility, response to changed gains/wind, weather checks, telemetry parsing,
  serial checksums, and HTTP token/Origin/Host/path restrictions. A pseudo-terminal
  exchanges settings with the **actual C protocol executable**, including
  read-back, corrupt input recovery, unsupported commands, and timeout/disconnect.
- USB DFU transfer tests use a simulated device to check target filtering,
  hash mismatch rejection, reviewed-byte staging, read-back verification, and
  failure on corruption. No test writes to a physical board.
- Browser test passed in installed Chrome: initial C simulation, wind changes,
  limit warnings, reset, board navigation, and 390-pixel layout without horizontal
  overflow. No JavaScript runtime errors. Desktop screenshot was inspected.
- The `.app` compiles, its plist validates, and its ad-hoc signature verifies.
  It was opened successfully; its native launcher and bundled local service
  were confirmed running.
- All firmware core/runtime sources compile with Arm GNU 14.3.Rel1 for Cortex-M4F.
  The no-I/O MCU image builds and its raw vector table passes the DFU inspector.

## Artifacts and toolchain

- `EasyTVC Studio.app`: native Mac launcher + local UI + C simulation library.
- `build-host/libeasytvc_studio.dylib`: host controller/simulator.
- `build-arm/libeasytvc_core.a`: ARM control/runtime library, **not an executable**.
- `build-arm/easytvc_safe_blank.bin`: 176-byte blank image, **no TVC or USB CDC**.
  SHA-256: `cd0aaa0fd2f072dccac325dc91b8cc153426ab79b38ef7e11df0e0eedabf1954`.

The full Arm toolchain was downloaded from Arm's official HTTPS distribution and
its archive SHA-256 matched the separately downloaded checksum:

```
30f4d08b219190a37cded6aa796f4549504902c53cfc3c7e044a8490b6eba1f7
arm-gnu-toolchain-14.3.rel1-darwin-arm64-arm-none-eabi.tar.xz
```

It lives in `.tools/`, outside the source files. `scripts/build-firmware.sh` detects
it automatically. A plain Homebrew arm-none-eabi-gcc install did not include the
newlib headers needed by this project. On another platform use a complete Arm GNU
toolchain and set `ARM_TOOLCHAIN_DIR` accordingly.

## Not verified / not implemented

No physical DFU write/read-back, USB CDC enumeration, sensor acquisition,
stationary calibration, PWM output, watchdog reset, brownout behavior, power-loss
storage recovery, servo direction, loaded actuator response, flight, or weather
qualification has been performed. The board driver needed for these operations
does not exist. Pin assignments are recorded from the 2026-09-29 vendor hwdef
and have not been exercised on hardware.

Host tests and an ARM compile do not establish flightworthiness. A functioning
board image and measured bench results remain necessary before considering flight.

## Repeat commands

```sh
./scripts/build-studio.sh
python3 -m unittest discover -s tests -p 'test_studio.py' -v
./scripts/build-firmware.sh
cmake -S . -B build-sanitize -DCMAKE_BUILD_TYPE=Debug \
  '-DCMAKE_C_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer' \
  '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined'
cmake --build build-sanitize --parallel 4
ctest --test-dir build-sanitize --output-on-failure
```

Optional browser check: start `python3 studio/studio.py --no-browser`, then pass
its printed local session URL to `node tests/ui_smoke.mjs 'LOCAL_SESSION_URL'`.
This needs Node 22+, installed Chrome, and permission to run a local server/browser.
