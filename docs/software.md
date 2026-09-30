# Software architecture and board adapter contract

## Portable runtime

`EasyTVCRuntime` owns the application and settings protocol. All calls must run
on one serialized main-loop task; interrupt handlers may enqueue samples/bytes
but must not call into this state concurrently.

- `EasyTVC_RuntimeInit` starts disarmed with defaults. `hardware_verified=false`
  prevents arming and settings writes. It must remain false for the current board.
- `EasyTVC_RuntimeLoadSettings` validates a versioned CRC packet and installs it
  only while idle and physically disarmed. Failed loads must inhibit hardware arming
  until the operator explicitly reviews/replaces the defaults.
- `EasyTVC_RuntimeStep` consumes a complete sample and returns calibrated pulse
  requests, controller status, flight state, and telemetry.
- `EasyTVC_RuntimeFeed` consumes one received byte. Pass a freshly read, qualified
  arm input. Settings are decoded into a candidate, stored through the adapter
  callback, applied to the active controller, and only then acknowledged. Failed
  storage leaves the active settings unchanged.

The hardware adapter, including its storage callback, **does not exist yet**.
Vendor pin macros are in `board_pins.h`; they are not wired to peripherals.
`tests/protocol_console.c` is only a host fixture with simulated storage.

## Coordinate convention and control

All samples are in a right-handed **body frame with +Z toward the nose**. The
driver must apply a verified sensor-to-body rotation to gyro and acceleration.
The quaternion rotates body into world coordinates. The two gimbal controllers
act on body X/Y rotations, using world-up expressed in body coordinates. Z-axis
spin is not controlled. The old app incorrectly used Euler pitch/yaw despite
treating Z as axial; `controller.c` replaces that path.

Use `EasyTVCControlOutput.pulse_us[0..1]` and `settings.servo[i].channel` for physical
outputs. Do not assume an opposed four-servo mixer. Legacy `tvc.c` remains for its
original tests; `app.c` and Studio both use `controller.c`.

Gains operate on radians and normalized command: P command/rad, I command/(rad·s),
D command·s/rad. Derivative is on measurement, optionally low-pass filtered.
Integration is withheld when it would worsen amplitude or slew saturation.
Servo reversal/endpoints are independent per axis. The model assumes correct
physical direction; a profile cannot establish that direction experimentally.

Invalid settings/data, repeated/backward/stale samples, excessive boost tilt, or
physical arm loss during boost latch a fault and return neutral pulse requests.
An MCU hang needs independent watchdog/PWM timeout protection in the adapter.
Fault reset must be an explicit disarmed operation.

`dt_s` must be positive and at most 20 ms. Sensor timestamps advance by 1–20 ms
and must be no more than 20 ms old. Unsigned subtraction handles millisecond
wrap. The intended complete IMU sampling rate is 200 Hz. The slower barometer
needs separate freshness checks included in the aggregate `sensors_healthy` flag.

Accelerometer attitude correction is only used before boost, near 1 g, when the
caller declares gravity trustworthy. Specific force is rotated into world
vertical before gravity subtraction. Altitude filtering and flight thresholds
are experimental defaults, not a qualified deployment or complete flight safety
policy. Pyro outputs remain compiled out.

## Required board adapter

Do not set `hardware_verified=true` merely because an image builds. Complete:

1. Vendor hwdef pin map is recorded. Still required: startup/off behavior for
   every pin an image actually drives, and I2C1 internal pull-ups for BME280.
2. Clock/FPU setup (16 MHz HSE per hwdef), complete interrupt vectors,
   brownout/reset handling, independent watchdog, timing budget, and fault-safe PWM.
3. BMI088 on SPI1 (accel CS `PC8`, gyro CS `PC7`, Mode 3), dummy cycles,
   data-ready sampling, timestamps, scaling/saturation detection, stationary gyro
   calibration, and a bench-checked sensor-axis transform (`ROLL_180_YAW_90` in hwdef).
4. BME280 on I2C1 with STM32 internal pull-ups, chip-id `0x60`, addresses `0x76`/`0x77`,
   calibration/compensation, timing/freshness/plausibility, and pad pressure reference.
5. USB CDC descriptors and bounded nonblocking queues. Drop telemetry when the
   host is slow. USB must not delay control or be required during flight.
6. Servo timers, power supply, endpoints, physical direction, and reset/stall/
   brownout/watchdog behavior, measured on the bench.
7. Qualified physical arm sensing. SW's electrical function is unknown; do not
   assume it is an MCU-readable arm switch.
8. Two independently erasable settings records with CRC/version/generation,
   commit-last semantics, and read-back. Power interruption must preserve the
   previous valid record. Return true from persistence only after verification;
   never erase/write while armed.

BMI088/BME280 files currently perform **identity probes only**. NOR (SPI2) and
microSD (SPI3) logging are absent. The overwriting 32-record RAM ring is
diagnostic, not a flight recorder. The only executable MCU image remains the
no-I/O blank image.

## Mac Studio

The Python standard-library server loads a native C library for simulation and
wire-packet validation. It binds only `127.0.0.1` on a random port, with a per-launch
token, Host/Origin checks, and content security policy. Three fixed frontend files
are served. The `.app` bundles the frontend, Python source, and native library;
Python 3.9+ is an external dependency. The app is locally signed, not notarized.

Only control/servo fields are uploaded; model/weather information stays in the
local profile. Simulation models constant thrust, independent rotational inertia,
first-order actuator lag, and wind torque using approximate density, side area,
drag coefficient, and CP arm. It does not model complete trajectory, recovery,
nonlinear aerodynamics, changing mass/thrust, vibration, power, or water ingress.
Weather limits must come from external evidence. Defaults are unvalidated.
