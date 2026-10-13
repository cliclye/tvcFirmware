# EasyTVC SD Card Logging

Raw-sector logging to the microSD card (SPI3), no file system. Format: `Core/Inc/data_logger.h`.

> **Use a dedicated card (1–32 GB).** Everything from sector 1024 on is overwritten.

## Layout

| Sector | Contents |
|---|---|
| 1024 | Directory: `ETVCDIR1`, next session number (+ its complement as a check) |
| 2048 + (n % 32) × 16384 | Session *n* header: `ETVCLOG2`, session, record size, settings text |
| … + 1 onward | 32-byte records, 16 per sector |

Every power-up starts a new session in its own 8 MB region, so **turning the board back on after
a flight does not overwrite it**. The 32 most recent sessions are kept.

## Record (32 bytes, little-endian, body frame: +Z = nose)

| Field | Type | Unit |
|---|---|---|
| timestamp | u32 | ms since power-up |
| accel x/y/z | i16 ×3 | mg |
| gyro x/y/z | i16 ×3 | 0.1 deg/s (bias-corrected) |
| tilt x/y | i16 ×2 | 0.01 deg (what the controller corrects) |
| servo A/B | u16 ×2 | µs sent to the TVC servos |
| baro altitude | i16 | 0.1 m above the pad |
| vertical speed | i16 | cm/s (estimate) |
| session | u8 | low byte of the session number |
| state | u8 | IDLE, ARMED, BOOST, COAST, APOGEE, DESCENT, LANDED, FAULT |
| flags | u8 | imu_ok, baro_ok, gyro_cal_ok, accel_sat, gyro_sat, tvc_active, ctrl_fault, chute_open |
| marker | u8 | 0xA5 |

Rate: **100 Hz** from arming until landing, **10 Hz** on the pad and on the bench. The partial
sector is written every second (at most ~1 s lost on power loss). Writes never block the control
loop: an 8-sector RAM queue absorbs slow cards, and anything beyond that is counted as `dropped`
in the `S,...` serial status line.

## Reading a flight

```bash
diskutil list                                   # find the card, e.g. /dev/disk4
./scripts/read-sd-data.sh /dev/disk4            # latest session -> session_N.bin
./scripts/read-sd-data.sh /dev/disk4 5          # a specific session
python3 scripts/telemetry_viewer.py session_N.bin          # timeline summary + plots (matplotlib)
python3 scripts/telemetry_viewer.py session_N.bin --csv flight.csv
```

The viewer prints the state timeline, when the parachute servo opened, max altitude, worst tilt
during boost, and any controller fault.
