# Wireless telemetry (XIAO ESP32C3 + DroneBridge)

Phase 0 findings and the plan. Phases 1–3 are implemented below this section as they land.

## Phase 0: what exists today (read from the code, 2026-11-02)

### USART2 (the TX2/RX2 pins on the 7-pin port)
- **Initialised already**: `EasyTVC_USART2_Init()` (`Core/Src/uart_driver.c`), PA2 TX / PA3 RX,
  **115200 8N1**, from `main.c` and `sensor_test.c`. UART4 (RC input) is set up but unused.
- **TX is already non-blocking**: a 512-byte ring drained by the USART2 interrupt
  (`EasyTVC_USART2_Queue`). Anything that doesn't fit is dropped, so a slow link can't stall the
  200 Hz loop.
- **RX is not used**: no image reads USART2. RX is "polled" in name only: nothing polls it, and
  the IRQ only drains overruns. At 115200 a byte arrives every 87 µs, so commands need an RX
  interrupt + ring.
- **What it sends** is **unframed text lines** (no sync, no sequence, no CRC):
  - flight image: a 12-field CSV line at 10 Hz (`telemetry.c`: ms, state, roll/pitch/yaw mrad,
    altitude mm, vspeed mm/s, command x/y milli, tvc, armed, pyro_locked) and `S,…` status at 1 Hz;
  - bench image: `B,…` at 10 Hz (tilt, pulses, raw accel) and `S,…`;
  - sensor-test image: `T,…` (10 Hz values) and `Q,…` (1 Hz checks).
  - It is **not MAVLink**.

### How Studio talks to the board today
- **There is no USB serial on the board.** USB-C is only the STM32's built-in DFU (ROM) bootloader.
  `usb_cdc.c` was an empty stub (removed in the cleanup).
- **Live board** (`studio/launch.py` → `Monitor`, `parse_line`) reads those text lines from a
  USB-UART adapter on TX2/RX2 (`/dev/cu.usbserial*`), **read-only**, and shows status, bench tilt,
  sensor-test values and the nose-axis suggestion.
- A **framed command protocol "E1"** exists but runs on no image yet: `Core/Src/protocol.c` +
  `runtime.c`, Studio `studio.py` → `frame()`, `parse_frame()`, `Device` (HELLO / STATUS / GET / SET
  of an 88-byte settings record). Spec: `docs/protocol.md`. ASCII, one line per frame:
  `E1 <sequence> <command> <payload> *<crc32>\n`, ≤ 255 bytes, CRC-32 (zlib), bad frames dropped,
  resync on LF. Tested over a pty with `tests/protocol_console.c`.
- **Config is compile-time.** Every setting lives in `vehicle_config.h`; Studio writes that file and
  rebuilds. The firmware has **no runtime settings storage**: `SET` has no persistence backend in
  any image.

### What Studio already does
Rocket setup (writes `vehicle_config.h`) · Tuning lab · 3D flight sim (firmware in the loop, live
mode with kicks/gusts/joystick, 40-flight Monte Carlo) · Board → Install firmware (build + DFU flash
with read-back verify; bench / flight / sensor-test images) · Live → Pad tools (USB-UART text) ·
Launch checklist · Flight logs (SD card → Terminal password prompt → saved sessions, sensor-test
graphs, 3D replay of flight logs and sensor tests, CSV).

### Flashing and the bootloader
- `scripts/dfu.py` + `dfu-util` flash `build-arm/*.bin` to **0x08000000** through the STM32F405's
  **ROM DFU** (hold BOOT, plug in USB-C). Every byte is read back and compared.
- **There is no ArduPilot bootloader on the board**: our image starts at 0x08000000
  (`ld/stm32f405rg.ld`), so if the board ever shipped with one it was overwritten by the first flash.
- The STM32F405 **ROM** bootloader listens on **USART1, USART3, CAN2 and USB DFU, not USART2**.
  So nothing on the board can take firmware over TX2/RX2 today. See Phase 4.

### Flight logic (`flight_fsm.c`, `pad_arm.c`, `main.c`)
- States: **IDLE → ARMED → BOOST → COAST → APOGEE → DESCENT → LANDED**, plus **FAULT**.
- **Arming**: automatic on the pad after `ARM_HOLD_MS` vertical and still (`pad_arm.c`); tipping past
  `DISARM_TILT_DEG` disarms. There is no arm command anywhere.
- **Launch**: |a| ≥ `LAUNCH_ACCEL_G` (now 1.8 g) and axial ≥ half of it for `LAUNCH_CONFIRM_MS`.
  TVC on after `TVC_ENABLE_DELAY_MS`; TVC only in BOOST; gives up past `MAX_TILT_DEG`.
- **Burnout** → COAST on low axial acceleration. **Apogee**: vertical speed ≤ 0 after the minimum
  time, or the failsafe time.
- **Recovery**: parachute **servo** on channel 3 opens at APOGEE/DESCENT/LANDED or `CHUTE_BACKUP_MS`
  after launch. **Pyro outputs are compiled out** (`EASYTVC_PYRO_OUTPUTS_COMPILED_IN 0`).
- Useful live: state, time since launch, attitude, rates, acceleration, baro altitude and vertical
  speed, tilt, TVC commands, servo pulses, chute flag, health flags.

### Hardware notes that change the request
- **Baro is a BME280** (chip id 0x60), not a BMP280.
- **Logs are on the microSD card**, not the W25Q128 (SPI2 is unused). "Flash free space" is
  reported as SD session space left.
- **Battery voltage**: PA4 / ADC1 IN4, but the board's divider ratio is **unknown** (hwdef says
  `SCALE(1)`, unverified). The flight image doesn't read it yet. Telemetry will send the pin voltage
  plus a ratio you calibrate once (measure the pack with a meter).
- **Pyro continuity**: inputs PB12 / PC4 / PC2 (vendor-untested). Reading them is input-only and
  doesn't change pyro behaviour. Pyro "armed/fired" are always 0 (compiled out).
- **DroneBridge ESP-NOW ground unit has no Wi-Fi AP.** In ESP-NOW LR mode the GND unit is a
  serial endpoint (its docs: "the web UI is unavailable in ESP-NOW mode"; the laptop connects
  through serial/USB). So with ESP-NOW (~1 km) the Mac sees the **ground XIAO as a USB serial port**,
  not a Wi-Fi network. Wi-Fi/UDP/TCP (14550 / 5760, AP 192.168.2.1, password `dronebridge`) is the
  **Wi-Fi AP mode** (~150 m). Studio supports **all three transports** (serial, UDP, TCP).
- **UDP**: DroneBridge only sends to UDP clients it has heard from, so Studio sends a small keep-alive
  frame once a second.
- **XIAO pins**: DroneBridge's docs say use **GPIO4/5/6/7** on the ESP32-C3 and, for the XIAO, *not
  D6/D7* (GPIO21/20 = the C3's boot console UART, which prints boot text into the EasyTVC's RX).
  Recommended: **XIAO D3 = GPIO5 → TX** to EasyTVC RX2, **XIAO D2 = GPIO4 → RX** from EasyTVC TX2.
- **Power**: the port's 5 V is very likely the board's 7805, shared with the servos. An ESP32-C3
  transmitting draws up to ~0.3 A. On 2S that adds ~1 W of regulator heat. Do the bench power test
  with the XIAO plugged in; if the 5 V sags, feed the XIAO from the UBEC.

## Protocol decision

**Reuse the E1 framing for everything on USART2** (one parser, one code path, USB-UART and wireless
identical). It already has what a lossy link needs: a sync marker (`E1 ` at line start), a length
bound (≤ 255 B, LF-terminated), a message ID (the command field), a sequence number and CRC-32,
and it resyncs on the next LF. MAVLink isn't in the firmware and would cost a generator plus
hand-mapped messages that QGC wouldn't show for TVC anyway.

- **Board → Studio, unsolicited**: `E1 <dev_seq> <ID> <csv> *<crc>`. `dev_seq` counts every
  unsolicited frame, so Studio derives packet loss from gaps. IDs: `TLM` (fast), `SYS` (1 Hz),
  `EVT` (events), `BEN` (bench), `SNS` / `SCK` (sensor test).
- **Studio → board, requests**: `E1 <host_seq> <CMD> <payload> *<crc>`. Responses echo `host_seq`
  with `OK` / `ERR`.
- **Read-only during flight**: the board ignores every request except `HELLO` and `PING` unless the
  state is **IDLE**. Studio refuses the same commands unless the last state it saw was IDLE.
  **There is no arm, fire, servo, chute, launch or bootloader command**, in any state.

## What's implemented

| Piece | Where |
| --- | --- |
| Frame encoder/decoder, IDLE gate, TLM/SYS payloads, base64 | `Core/Src/link.c`, `Core/Inc/link.h` |
| USART2 RX interrupt ring, whole-frame-or-nothing TX queue | `Core/Src/uart_driver.c` |
| Telemetry, events, request handlers, loop-timing counters | `Core/Src/main.c` (flight + bench), `Core/Src/sensor_test.c` |
| Battery pin ADC, continuity inputs (read-only) | `Core/Src/adc_driver.c`, `EasyTVC_GPIO_ContinuityBits` |
| Version + git hash in HELLO | `Core/Inc/version.h`, `CMakeLists.txt` |
| Transports, link stats, recorder, requests, resumable download | `studio/groundlink.py` |
| Live telemetry page | `studio/web/live.js` (Live → Pad tools) |
| Fake board for testing without hardware | `scripts/fake_link.py` |

### Messages (board → ground, unsolicited)

All integers. `seq` counts every unsolicited frame the board *tried* to send, so a frame the UART
ring had no room for still shows on the ground as a gap.

**TLM**: 10 Hz on the pad, 20 Hz from ARMED until LANDED (23 fields):
`ms, state, since_launch_ms (-1 before launch), qw, qx, qy, qz (×1e4), gx, gy, gz (mrad/s),
ax, ay, az (mg, body), alt_cm, vs_cm_s, tilt_x, tilt_y (centi-degrees), cmd_a, cmd_b (milli, −1000..1000
of servo travel), pulse_a, pulse_b, pulse_chute (µs), flags, deploy_eta_ms`.
Firmware 0.5 adds `pressure_pa, baro_temp_cdeg` (26 fields).
`deploy_eta_ms` (firmware 0.4+): until the parachute deploys (release servo or ejection charge), −1 before
launch or with no recovery set, 0 once deployed. Firmware 0.3 sent the first 23 fields only.
Flags bits 0–10: imu_ok, baro_ok, gyro_cal, accel_sat, gyro_sat, tvc, ctrl_fault, chute, armed, recovery,
servo_test (gimbal test or trim active).

**SYS**: 1 Hz (23 fields): `ms, image (0 flight, 1 bench, 2 sensor test), state, armed, chute,
batt_pin_mv, vdda_mv, continuity (bits P1..P3, raw levels), pyro_armed, pyro_fired (both 0 unless the pyro outputs are unlocked),
sd_ok, session, records, dropped, write_errors, sd_left_kb, loop_max_us, late_ticks, imu_errors,
baro_age_ms, link_dropped, link_ignored, errors`.
Error bits 0–9: imu_fail, baro_stale, sd_missing, config_bad, recovery, fault, sd_errors, sd_full,
link_drops, late_ticks.

**EVT**: `ms,NAME` on each state change (LAUNCH_DETECTED, BURNOUT, APOGEE, DESCENT, LANDED, FAULT,
ARMED, IDLE) and IMU_OK / IMU_LOST, PARACHUTE_RELEASED, CHARGE_FIRED (or CHARGE_DUE_LOCKED while the
pyro outputs are locked), RECOVERY_BOOT, SD_MISSING. Sent twice
(Studio drops the copy) so one lost packet doesn't lose the event.

**BEN** (bench image, 10 Hz), **SNS** / **SCK** (sensor test, 10 Hz / 1 Hz): the same values as the
old `B`, `T`, `Q` text lines, now framed.

### Requests (ground → board)

| Command | When | Reply (`OK`) |
| --- | --- | --- |
| `HELLO` | always | `EASYTVC,2,<flight\|bench\|sensortest>,<version>,<git hash>` |
| `PING` | always | `PONG,<ms>` |
| `STATUS` | IDLE only | `state,armed,recovery,imu_ok,baro_ok,sd_ok` |
| `CFG` | IDLE only | the settings in use, in `groundlink.CFG_FIELDS` order: 32, then (0.4+) the pyro channel, delay, on-time and master lock, then (0.5+) deploy height, charge backup and whether radio settings are in use |
| `LOGINFO` | IDLE only | `sd_ready,session,record_bytes (32),session_sectors (16384)` |
| `LOGREAD` | IDLE only | `session,offset` → `session,offset,128,<crc32 hex>,<base64 of 128 bytes>` |
| `SETCFG` | IDLE, not armed | the 14 tunables (`groundlink.TUNABLE_FIELDS`, wire units) → stored on the SD card, read back, applied; reply = the set now in use |
| `CFGRESET` | IDLE, not armed | clears the stored set: back to the installed values |
| `PYROARM` | IDLE, not armed | `channel` (1 or 3) → `channel,code,locked,continuity` |
| `PYROFIRE` | IDLE, not armed | `channel,code,ms` (20–1000 ms, the code from PYROARM within 10 s, once) → `channel,ms,locked,continuity` |
| `BEEP` | IDLE only | ~2 s of red/green/blue LEDs and beeps |
| `SERVO` | IDLE only | `a,b` in milli of travel (−1000..1000) → `mode,trim_a,trim_b`. Held for 1 s; repeat to keep holding |
| `SWEEP` | IDLE only | servo A centre → max → min → centre, then B (300 ms steps) |
| `CENTER` | IDLE only | ends a hold or sweep |
| `TRIM` | IDLE only | `a,b` in µs (−100..100); the centre must stay ≥ 50 µs inside each endpoint |

In any other state the board **ignores** the request (no reply; counted in SYS `link_ignored`).
Errors: `ERR COMMAND`, `PAYLOAD`, `RANGE`, `BUSY` (an SD write is in flight: ask again).
The board takes at most one request per 5 ms loop tick, and only when its transmit ring has room for
the whole reply, so a fast ground slows down rather than losing replies. Studio keeps up to 4 LOGREADs
in flight, checks the CRC of every chunk, asks again for lost or damaged ones, and can resume a
cancelled or interrupted download from where it stopped.

There is **no** arm, launch, parachute-release or bootloader command, and nothing on the link works
outside IDLE except HELLO/PING. The pyro test can only pulse a channel on the pad, disarmed, with a
one-off code, and only moves the pin if the outputs are unlocked in the firmware.

### Settings over the radio (`Core/Src/tunables.c`)
Live → Pad tools → Board settings. What can be sent: the PID gains, integrator limit, slew, D filter,
give-up tilt, the TVC X/Y centres, the servo backup time, and the charge's deploy height, delay,
on-time and backup. What can't: wiring (channels, directions, endpoints, nose axis), which still
changes by installing over USB-C.
- The board checks the ranges, writes the set to its SD card (sectors 1000/1001, alternating, with a
  sequence number and CRC so a power cut mid-write keeps the other copy), reads it back, applies it,
  and echoes it. Studio compares every value and then reads the config again.
- Each stored set carries a CRC of the values compiled into the firmware it was made for. Installing
  new firmware therefore wins; until then the radio set wins. Studio can copy what it sent into
  Rocket setup so the next install keeps it.
- No SD card: nothing is stored (`ERR NO_SD`).
- The SD session header still lists the compiled values; Read config shows the ones in use.

### Pyro channel ground test (`Core/Src/pyro_test.c`)
Live → Pad tools → Pyro channel test, with a test load (LED + 1 kΩ or a bulb), never a charge.
IDLE and not armed only; PYROARM gives a one-off code, PYROFIRE must repeat it within 10 s; one
pulse of 20–1000 ms; it stops at once if the pad arms. P2 is refused. With the outputs locked
(`EASYTVC_PYRO_OUTPUTS_COMPILED_IN 0`) the test runs but the pin stays off, and the event log shows
`PYRO_TEST_Pn_LOCKED`. PYROFIRE is never retried: a lost reply means "check Events", not "try again".

### Altitude-triggered charge (`Core/Src/recovery_charge.c`)
`PYRO_DEPLOY_ALT_M` > 0: after apogee is detected and the rocket is coming down, the charge fires when
it passes that height above the pad (+ `PYRO_CHUTE_DELAY_MS`). If the barometer goes stale, altitude
stops counting and the timer (`PYRO_BACKUP_MS` after launch) fires it. Apogee lower than the deploy
height: it fires at apogee. 0 = at apogee. Set the backup from the simulated time of reaching that
height on the way down plus ~1 s. (The 3D sim still opens the parachute at apogee, so it doesn't
model a low deploy height yet.)

### TVC gimbal test (`Core/Src/servo_test.c`)
For checking and aligning the mount without a cable: Live → Pad tools → Board tools → TVC gimbal test.
- **Servos A and B only.** The parachute servo can't be moved from the link.
- **IDLE only**, enforced on both sides. The moment the state leaves IDLE (the pad auto-arming, for
  example) the test ends and the trim is cleared, so the flight always uses the compiled-in calibration.
- **Hold to move**: a position lasts 1 s after the last `SERVO`. Studio repeats it every 200 ms while
  you hold a slider, so a lost link returns the gimbal to centre within a second.
- **Trim is temporary** (RAM only). It shifts the centre (endpoints and direction stay as compiled),
  and on the bench image it shifts the follow-the-tilt output too. "Save trim to Rocket setup" writes
  board centre + trim into your setup. Build and install over USB-C to make it permanent.

## Wiring the XIAO ESP32-C3

```
EasyTVC 7-pin port          XIAO ESP32-C3
  5V   ───────────────────  5V
  GND  ───────────────────  GND
  TX2  ───────────────────  D2 (GPIO4)   XIAO RX
  RX2  ───────────────────  D3 (GPIO5)   XIAO TX
```

- Both sides are 3.3 V logic: wire directly, no level shifter.
- **Don't use D6/D7** (GPIO21/20): that's the C3's boot console. It prints boot text that the EasyTVC
  would receive. GPIO4/5 are the pins DroneBridge documents for the C3.
- Use the XIAO's **external-antenna** version (the u.FL antenna), pointed away from the motor tube and
  wiring, for range.
- **Power**: the port's 5 V is most likely the board's 7805, shared with the servos. The XIAO can draw
  ~0.3 A while transmitting. Do the bench power test with the XIAO fitted and the servos moving. If 5 V
  sags below ~4.8 V or the regulator gets hot, power the XIAO from the UBEC instead (GND still common).

## DroneBridge setup

Flash DroneBridge for ESP32 with its web flasher. Pick the **ESP32-C3** build. The XIAO isn't on its
official board list, but it's a plain ESP32-C3 with the pins above. Then open its web UI
(Wi-Fi `DroneBridge ESP32`, password `dronebridge`, http://192.168.2.1).

**Air unit (in the rocket)**

| Setting | Value |
| --- | --- |
| Mode | **ESP-NOW LR Mode AIR** (long range), or **Wi-Fi Access Point** for short range |
| UART TX pin / RX pin | **5 / 4** |
| UART baud | **115200** |
| RTS / CTS (flow control) | **off** (RTS 0, CTS 0) |
| Serial protocol | **Transparent** |
| Serial read timeout | 20 ms (default is fine: frames are LF-terminated) |
| Packet size | default |

**Ground unit (ESP-NOW only)**: a second ESP32 in **ESP-NOW LR Mode GND**, same channel and
encryption key as the air unit. In this mode it has **no Wi-Fi network**: plug it into the Mac by
USB and choose it under **USB serial** in Studio.

## Bluetooth (the default link)

DroneBridge for ESP32 has a **Bluetooth LE mode**: service `0000db32-0000-1000-8000-00805f9b34fb`,
the board's bytes arrive as notifications on `…db34`, and bytes written to `…db33` go to the board.
Studio talks to it through a small CoreBluetooth relay built into the Studio app
(`studio/BleRelay.swift`), because Python's standard library has no Bluetooth. The relay passes bytes
through unchanged on `127.0.0.1:14570`, so the frames, CRCs, loss stats, recording and the IDLE-only
rules are exactly the same as on every other link.

- DroneBridge settings: mode **Bluetooth LE**, UART TX pin 5 / RX pin 4, 115200 baud, flow control
  off, protocol Transparent. Its Wi-Fi access point keeps running alongside, for its settings page.
- No pairing and no network to join: Studio scans for the DroneBridge and connects by itself. The
  first time, macOS asks to let EasyTVC Studio use Bluetooth (System Settings → Privacy & Security →
  Bluetooth).
- Bluetooth needs the **Studio app**: running `python3 studio/studio.py` in a browser has no relay,
  and the page says so.
- **Range is short.** DroneBridge's own docs say BLE range is "greatly reduced compared to Wi-Fi":
  expect tens of metres line of sight, less through a body tube. That's ideal on the pad (status,
  config check, log download, gimbal test) and fine for the start of a flight; expect gaps near
  apogee. The SD card logs every sample regardless. For live data over the whole flight use
  ESP-NOW LR (about 1 km, a second ESP32 on the Mac's USB).
- Throughput is not a concern: ~4 KB/s of telemetry is well inside BLE's capacity.

## Connecting Studio

Live → Pad tools, then pick a link:

| Link | Use it with | Settings |
| --- | --- | --- |
| **Bluetooth** (default) | DroneBridge in Bluetooth LE mode, from the Studio app | nothing to fill in |
| **USB serial** | a USB-UART on TX2/RX2, or the DroneBridge ESP-NOW ground unit | pick the port |
| **Wi-Fi · UDP** | DroneBridge in Wi-Fi AP mode (join its network first) | bridge `192.168.2.1`, port `14550`, listen on `14550` |
| **Wi-Fi · TCP** | the same, over TCP | bridge `192.168.2.1`, port `5760` |

UDP: DroneBridge only sends to clients it has heard from, so Studio sends a `PING` once a second
(this also measures round-trip time). The page shows frames/s, loss % (from sequence gaps), time since
the last packet (**STALE** after 1 s), damaged frames and ping. Every frame is recorded to
`Documents/EasyTVC Flights/live_<date>-<time>.etlog`. Open it in **Flight logs** for the charts and
**Replay in 3D**, the same as an SD session. The SD card still has every sample; the radio log has
gaps where packets were lost.

The **Board tools** (Status, Read config, Download log, TVC gimbal test) are enabled only while the
board reports IDLE, and the board enforces the same rule itself. Read config compares the board's compiled-in settings
with your saved Rocket setup. Changing settings is still "edit → build → install over USB-C".

## Bandwidth budget (115200 baud = 11 520 B/s)

| Traffic | Size (worst case) | Rate | B/s |
| --- | --- | --- | --- |
| TLM in flight | 200 B | 20 Hz | 4 000 |
| SYS | 101 B | 1 Hz | 101 |
| EVT (×2) | ~60 B | ~2 /s at most | 120 |
| **Flight total** | | | **~4 220 (37 %)** |

On the pad TLM is 10 Hz (~17 %). A log download uses whatever is left: ~5 KB/s measured over a
simulated link with 10 % loss (`tests/test_studio.py`). The radio itself (ESP-NOW LR / Wi-Fi) carries
far more than the UART, so 115200 baud is the limit.

## Effect on the control loop

- Nothing on the link can block: frames are queued whole into the TX ring or dropped (and counted).
  Received bytes go into an interrupt ring; the loop reads at most 64 per tick.
- Flight logic never reads the link: losing it, or a flood of garbage, can't change state, arming,
  TVC or the parachute. Apart from the IDLE-only gimbal test (which ends on its own when the link
  goes quiet), requests can only read.
- Per-tick work, measured on the host (`tests/test_link.c`, section 9): **before** (old text line
  formatted every tick + 10 Hz serial line) vs **after** (no per-tick text, TLM at 20 Hz + SYS):
  after ≤ before, on the pad and in flight. Two changes paid for the new frames: the app's unused
  per-tick telemetry text is skipped on the board (`skip_text_telemetry`), and CRC-32 uses a 1 KB table.
- On the board, SYS reports `loop_max_us` (longest loop body in the last second) and `late_ticks`.
  Check them on the bench with the XIAO connected: `loop_max_us` should stay well under 5000.

## Testing without hardware

```bash
python3 scripts/fake_link.py --loss 0.1 --corrupt 0.01          # a simulated flight of your rocket
python3 scripts/fake_link.py --idle --session path/to/session_N.bin   # on the pad: try the board tools
```

In Studio: Wi-Fi · UDP, bridge `127.0.0.1`, port `14551`, listen on `14550`. The fake board flies the
rocket from Rocket setup in the 3D sim and answers requests with the firmware's IDLE-only rule.

## Not verified on hardware

- DroneBridge on the XIAO ESP32-C3 (unofficial board) with GPIO4/5, and real ESP-NOW range.
- On-target loop timing with the link running (read `loop_max_us` / `late_ticks` in SYS).
- The battery divider ratio (calibrate once with a meter: Live data → Battery divider…).
- What the continuity inputs read with and without an igniter (shown as raw levels only).
- 5 V rail headroom with the XIAO transmitting (and with the gimbal test sweeping the servos).
- The gimbal test on the real board (it's tested on the host and against the fake board).

### Open decisions
- **Writing config over the link.** There's nowhere on the board to store it: settings are compiled
  in. Options: (a) keep config as "edit → build → install over USB-C" (current), (b) add a settings
  record on the SD card (or internal flash) that the firmware loads at boot and Studio writes over
  the link when IDLE, with read-back. (b) is a real change to how the rocket gets its settings.
- **Phase 4, wireless firmware update: dropped** (not needed). USB-C stays the only way to install
  firmware; the bootloader is untouched.
