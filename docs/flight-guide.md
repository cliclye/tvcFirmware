# EasyTVC: from bench to first flight

This is the procedure for the `easytvc_bench` and `easytvc_flight` images. **EasyTVC Studio.app**
does every step below from one window (Rocket setup → Build & flash → Live board → Launch
checklist → Flight logs); the commands are here for reference. Everything specific
to your rocket is in [`Core/Inc/vehicle_config.h`](../Core/Inc/vehicle_config.h). After changing
it, rebuild with `./scripts/build-firmware.sh` and repeat the bench checks.

> **Status (2026-10-12):** the firmware builds, and every host test passes, including the
> end-to-end flight simulation in `tests/test_flight.c` (pad arm → launch → closed-loop TVC →
> apogee → parachute). **None of it has run on a real board yet.** The bench steps below are how
> you find what simulation can't: servo direction, IMU mounting, power and real gains. Don't
> skip them.

## 1. Parts list (Estes E12 build)

| Part | Recommendation | Notes |
|---|---|---|
| Flight computer | EasyTVC v0.1 (you have it) | XT30 battery input; servos run from its onboard 5 V linear regulator |
| TVC servos (2) | **TowerPro MG90D** (metal gear, digital, ~13 g) | Alternative: Emax ES08MD II. Avoid plastic-gear SG90s, which are sloppy around centre. Buy one or two spares |
| Parachute servo (1) | Any 9 g micro servo (MG90D or SG90) | Only needs to pull a pin or release a latch |
| Battery | **2S (7.4 V) LiPo, 300–450 mAh, XT30 plug**, e.g. a small FPV pack | **Not 3S.** See the power note below. Around 20–30 g |
| LiPo charger | Any balance charger that does 2S | Charge before every flight; don't fly below 7.6 V resting |
| microSD | 1–32 GB SDHC, a **dedicated** card | The logger writes raw sectors and wipes any file system after sector 1024 |
| USB-UART adapter | 3.3 V CP2102 or FT232 module | Wire it to the `TX2/RX2/GND` header to see live telemetry. USB-C on the board is only for flashing (no USB serial yet) |
| Motors | **Estes E12-0** (no ejection charge) | The firmware releases the parachute with a servo. With E12-4/-6 the ejection charge also fires through the gimbal, so that needs a gas path and a retained motor |
| Igniters + launch controller | Estes starters and a standard controller | |
| Recovery | 18" (45 cm) parachute, Kevlar shock cord, swivel, Nomex blanket | An 18" chute gives roughly 5 m/s descent at 350 g |
| Launch stand | A vertical stand or tower that holds the rocket upright and lets go freely | A finless TVC rocket can't stabilise itself off a long rod. It has to be vertical before ignition |
| Hardware | M2/M2.5 nylon standoffs, servo pushrods (1 mm wire with Z-bends or ball links), heat-set inserts | Use PETG/ASA, not PLA, for anything near the motor |

**Mass budget:** the E12 averages about 12 N. The firmware detects launch above 2.5 g, so liftoff
mass must stay **under about 0.4 kg**, motor included. Weigh the finished rocket.

**Power warning:** the servo headers' 5 V comes from the board's 7805, a linear regulator. It
burns off (battery − 5 V) × current as heat, and two loaded servos can pull peaks near 1 A.
On 2S that's fine for short bursts, but on 3S it overheats. A sag can also reset the MCU. The
firmware now detects a reboot in the air and fails safe (gimbal centred, parachute released),
but don't rely on that: do the power test in step 4. If it fails, power the servo `+` wires
from a separate 5–6 V, ≥3 A switching UBEC on the same 2S pack. Keep the signal wires and a
common ground on the board.

Don't connect anything to pyro channel P2. Its MOSFET gate is on PA13 (SWDIO), which the MCU
pulls *up* at reset. Pyro outputs are compiled out of this firmware altogether.

## 2. Wiring

- Servo A to header **1**, servo B to header **2**, parachute servo to header **3**. Header 4 is
  unused and held at 1500 µs. You can change the channels in `vehicle_config.h`.
- 2S LiPo to the XT30. Plugging in the battery is the power switch.
- Optional live telemetry: adapter `RX` to board `TX2`, adapter `TX` to board `RX2`, plus GND.
  Then run `screen /dev/tty.usbserial-XXXX 115200`.
- Dedicated microSD card in the slot.

## 3. Flash

Enter DFU: hold **boot**, press and release **RST** (or plug in USB-C), then release boot.

```bash
python3 scripts/dfu.py inspect build-arm/easytvc_bench.bin
```

```bash
python3 scripts/dfu.py flash build-arm/easytvc_bench.bin --board easytvc-v0.1 --sha256 <hash printed by inspect>
```

Then **unplug USB and the battery** (a full power cycle). If the board comes up with the red LED
on and the parachute servo open, it either saw a non-power-on reset or was moving when it
powered up. That's the in-air safety mode, and a power cycle with the board at rest clears it.

## 4. Bench checks (`easytvc_bench` image). Do all of them.

The bench image never arms. The gimbal always tries to hold the motor so that it would push the
rocket back to vertical.

1. **Power-up self test.** A short beep, then servo A sweeps centre → max → min → centre, then
   servo B does the same, then the parachute servo opens and closes. Make sure the right servo
   moves at each step. Watch for binding: if the gimbal hits a stop, shrink `MIN_US`/`MAX_US`
   for that servo. If you hear no beep at all, the buzzer is not working; note it.
2. **Nose axis.** Stand the rocket nose-up and keep it still. A **solid green LED** means the
   board thinks it's vertical, so `EASYTVC_NOSE_AXIS` is right. If the green LED is off, read
   the `B,...` telemetry line. One of the three `raw` accel numbers is near +1000 (or −1000):
   set `EASYTVC_NOSE_AXIS` to that axis (`POS_`/`NEG_`), rebuild and reflash.
3. **Channel and direction (the tilt test).** Hold the rocket vertical, then lean the nose about
   10° toward one side. **The motor nozzle must swing to the opposite side** of the lean,
   because that pushes the tail back under the nose. Repeat for all four directions.
   - If the wrong servo responds to a lean, swap `SERVO_A_CHANNEL` and `SERVO_B_CHANNEL`.
   - If the right servo moves the wrong way, flip that servo's `DIRECTION` to `-1`.
   - Past 25° of lean the gimbal centres. That's the in-flight give-up limit (`MAX_TILT_DEG`).
4. **Power test.** With the battery connected, push against the gimbal by hand while tilting the
   rocket around for 30 s. The blue LED must keep blinking steadily, and the power-up sweep
   must **not** run again (that would be a brownout reset). Then feel the 7805 on the back of
   the board: if it's too hot to touch, use the UBEC described above.
5. **Logging.** Power-cycle twice, then put the card in your Mac and run
   `./scripts/read-sd-data.sh /dev/diskN`. It should report the latest session, and
   `python3 scripts/telemetry_viewer.py session_N.bin` should show your tilt test.

## 5. Flight image

Flash `build-arm/easytvc_flight.bin` the same way as in step 3. The power-up self test only
moves the TVC servos. The parachute servo stays where it is, so it can't drop a packed chute.

### Pad procedure

1. Pack the parachute and close the release. Fit the E12-0 and the igniter. Put the rocket on
   the stand.
2. Connect the battery. You get a beep and the gimbal sweep, then the green LED blinks slowly.
3. **Hands off.** Once the rocket has stood vertical (within 10°) and still for 3 s, the green
   LED goes **solid** and the buzzer chirps once a second. It is now **ARMED**. It also takes the
   final gyro calibration from those 3 s.
   - If it gets bumped or tipped more than 20° it disarms, and arms again once it's still.
   - **Red LED with a long beep every 2 s: do not launch.** That means the IMU failed, there's
     no SD card, the configuration is invalid, or this is a recovery boot. Power-cycle and
     check.
4. Clear the area and launch. Launch is detected after 40 ms above 2.5 g along the nose, and
   TVC starts 80 ms later. It runs until burnout, then centres the gimbal.
5. The parachute servo opens at detected apogee, or at the latest `EASYTVC_CHUTE_BACKUP_MS`
   (7 s) after launch. **Set that to your OpenRocket apogee time + about 0.5 s.**
6. 20 s after launch the buzzer starts a locator double-beep.

### After the flight

Switching the board back on is safe: every power-up is a new session, and the 32 most recent
are kept. Read the flight with `./scripts/read-sd-data.sh /dev/diskN <session>` and the viewer.
It prints the state timeline, the worst tilt during boost, and any controller fault. Logging
runs at 100 Hz from arming to landing and 10 Hz on the bench or pad.

## 6. Tuning before the first flight

The gains in `vehicle_config.h` (`KP` 1.2, `KD` 0.2 in full-deflection per radian) are starting
values. The host simulation is stable with them for an assumed 31 rad/s² of control authority:
E12 thrust, about 0.15 m from gimbal to CG, ±5° of gimbal travel. Your rocket will differ.
Measure its mass, CG and moment of inertia, then use **EasyTVC Studio**'s simulation with your
numbers before flying. If the bench tilt test looks twitchy or oscillates, lower `KD` first. If
it's sluggish, raise `KP`. For the first flight, pick a day with almost no wind.

## 7. What changed in this pass (review findings)

| Area | Problem | Effect | Fix |
|---|---|---|---|
| Controller | Any invalid sample, even on the pad, latched the controller fault forever | One slow barometer retry or one missed IMU read before launch made the rocket go to FAULT at liftoff with a centred gimbal | Faults only latch while TVC is active. Regression test `pad_glitch_does_not_disable_tvc_in_flight` |
| Flight FSM | A bad sample while ARMED latched FAULT (power cycle needed) | Silent "never flies" state on the pad | ARMED disarms and re-qualifies; in flight it's still FAULT |
| Arming | `physical_arm_qualified` hard-wired false | Could never arm, so TVC could never run | Pad auto-arm: vertical + still for 3 s, disarms if tipped (`pad_arm.c`, tested) |
| IMU mounting | Sensor-to-body rotation was the identity | Wrong axes = TVC steering the wrong way | `EASYTVC_NOSE_AXIS` with all six right-handed mappings, plus a bench LED check |
| Gyro bias | Only calibrated at power-up, often while the rocket is in someone's hand | Tilt drift during boost | Re-taken from the 3 s still window on the pad |
| Recovery | Pyro compiled out and no other deployment path | With E12-0, no parachute at all | Servo release at apogee, with backup timer, fault fallback and in-air-reboot fallback |
| Resets | A watchdog reset or brownout in the air rebooted into normal start-up | The servo sweep would run in flight; a brownout looks like a power-on | Reset-cause and in-motion boot detection lead to a safe mode: gimbal centred, chute out, no arming |
| SD log | Restarted at sector 1024 on every boot | Powering the board on after a flight overwrote the flight log | Session directory: 32 sessions × 8 MB |
| SD log | 10 Hz with no attitude, servo or state data | Nothing to diagnose a TVC flight with | 100 Hz in flight, 32-byte records with tilt, servo pulses, state and flags; new viewer with CSV export |
| SD log | One sector of buffering | A slow card dropped data | 8-sector RAM queue |
| Main loop | Blocking sensor re-probes (60 ms or more) could run at any time | Broke the 20 ms timing limit, and in flight that's a FAULT | Re-probes only happen while IDLE |
| Buzzer | Driven as a DC level | A passive transducer stays silent, so there's no armed or error warning | 2.7 kHz tone from TIM1 on PA8 |
| Config | Servo channels, endpoints and gains could only be changed through an unimplemented USB path | Couldn't set up a rocket at all | `vehicle_config.h` |

Notes on the vendor `hwdef` (ArduPilot only, not used by this firmware): `#define HAL_BUZZER_PIN`
is commented out (`#` starts a comment in hwdef), so ArduPilot would have no buzzer. There's
no `HAL_BATT_VOLT_PIN`/scale, so battery monitoring isn't configured. The "H743 has 16 pages"
comment and the duplicate `STORAGE_FLASH_PAGE`/`HAL_STORAGE_SIZE` lines are copy-paste leftovers.
`PA13` (SWDIO) is used as the pyro 2 output.

## 8. Still not verified

- Nothing here has run on hardware. Servo direction, the IMU axis, gains, buzzer type,
  regulator heat and SD card compatibility are all things the bench steps exist to prove.
- No USB serial (CDC) yet. Use the UART header for live telemetry.
- Battery voltage (PA4) isn't measured, because the divider ratio is unknown. Check the pack
  with a meter before every flight.
- The `SW`/arming terminal's function isn't known, so the firmware doesn't read it.
