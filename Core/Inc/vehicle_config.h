#pragma once

/*
 * ============================================================================================
 *  EVERYTHING SPECIFIC TO YOUR ROCKET LIVES IN THIS FILE.
 *  Change a value, run ./scripts/build-firmware.sh, flash, and re-run the bench checks in
 *  docs/flight-guide.md. Never fly a value that has not been through the bench image.
 * ============================================================================================
 */

/* ---- IMU mounting ---------------------------------------------------------------------------
 * Which SENSOR axis points at the nose when the rocket stands on the pad. Find it with the bench
 * image: stand the rocket nose-up and read the `raw` accel columns; the one near +9800 is the nose
 * axis (+X/+Y/+Z). If it reads near -9800 use the NEG variant. Body X/Y are then derived so the
 * frame stays right-handed; the bench check decides which servo is which. */
#define EASYTVC_AXIS_POS_X 1
#define EASYTVC_AXIS_NEG_X 2
#define EASYTVC_AXIS_POS_Y 3
#define EASYTVC_AXIS_NEG_Y 4
#define EASYTVC_AXIS_POS_Z 5
#define EASYTVC_AXIS_NEG_Z 6

/* TVCRocketV3 (Fusion CAD, 2026-10-31): the EasyTVC stands VERTICALLY on the two-ring FC sled, pyro
 * terminals toward the nose, component side facing the front window (CAD +Y). The nose therefore lies
 * in the board plane, along the board's long edge, not through the board (the old flat-across mount
 * was POS_Z).
 * Evidence: the 2026-10-15 sensor test (board flat, parts up) read +1 g on sensor +Z, so +Z is out of
 * the component side. The vendor hwdef (IMU ROTATION_ROLL_180_YAW_90) puts the board's long "forward"
 * axis on sensor +Y, so terminals-up = +Y. This is the inferred axis, not yet measured nose-up: do the
 * bench check in docs/flight-guide.md (stand the rocket nose-up; Studio's sensor test names the axis).
 * A wrong value fails safe: the board won't arm unless it sees itself vertical. */
#define EASYTVC_NOSE_AXIS EASYTVC_AXIS_POS_Y

/* ---- Status LED -----------------------------------------------------------------------------
 * The RGB LED is driven by PB3 (red), PB4 (green), PB5 (blue). ArduPilot's convention for this
 * kind of board is active-low (common anode: pin LOW = on). At power-up the firmware shows red,
 * green, blue one at a time. If instead you see the LED mostly lit with one colour missing each
 * step, change this to 0. */
#define EASYTVC_LED_ACTIVE_LOW 0   /* measured on the real board 2026-10-14: HIGH = on */
/* Status LED brightness, 10..100 % in steps of 10 (software PWM at 100 Hz). */
#define EASYTVC_LED_BRIGHTNESS_PCT 10

/* ---- TVC servos -----------------------------------------------------------------------------
 * Servo A tilts the motor about body X, servo B about body Y. Channels are the board's servo
 * headers 1..4 (1 = PB0, 2 = PB1, 3 = PB8, 4 = PB9).
 * MIN/MAX are the pulses at the gimbal's mechanical limit: set them on the bench so the mount
 * never binds. DIRECTION is +1 or -1 and is decided by the bench "tilt test". */
/* With NOSE_AXIS = POS_Y the firmware body frame is: body X = sensor Z = CAD +Y (out of the board's
 * front), body Y = sensor X = CAD -X, body Z = nose. In the TVCRocketV3 CAD, "TVC Servo B" (outer gimbal)
 * tilts about CAD Y and "TVC Servo A" (inner gimbal) about CAD X. So firmware servo A (about body X) is
 * the CAD's Servo B, and firmware servo B is the CAD's Servo A. Wire CAD Servo A to header 1 and CAD
 * Servo B to header 2, as in the build notes. The DIRECTIONs still come from the bench tilt test. */
#define EASYTVC_SERVO_A_CHANNEL   2
#define EASYTVC_SERVO_A_MIN_US    1250
#define EASYTVC_SERVO_A_CENTER_US 1500
#define EASYTVC_SERVO_A_MAX_US    1750
#define EASYTVC_SERVO_A_DIRECTION 1

#define EASYTVC_SERVO_B_CHANNEL   1
#define EASYTVC_SERVO_B_MIN_US    1250
#define EASYTVC_SERVO_B_CENTER_US 1500
#define EASYTVC_SERVO_B_MAX_US    1750
#define EASYTVC_SERVO_B_DIRECTION 1

/* Servo frame rate. 50 Hz suits every hobby servo. Digital servos (e.g. MG90D, ES08MD II) can
 * usually take 100-200 Hz, which cuts up to 15 ms of actuator delay; raise it only after a bench
 * run at that rate with no buzzing or heat. Allowed: 50..333 and dividing 1 MHz (50/100/125/200/250). */
#define EASYTVC_SERVO_RATE_HZ 50

/* ---- Controller -----------------------------------------------------------------------------
 * Output is a fraction of full gimbal travel (+-1 = MIN/MAX above) per radian of tilt.
 * These are starting points only. Tune them in EasyTVC Studio with your rocket's measured
 * inertia, CG-to-gimbal arm and thrust before flying. */
/* Chosen by a gain search in Studio's 3D flight sim for the default rocket model (610 mm, 66 mm
 * BT-80, ogive nose, 80 g nose ballast, 380 g liftoff, -3.9 cal, 10.5 mm horn / 30 mm lever), on
 * the measured E12 curve with wind to 6 m/s, 1 deg misalignment, pad lean and mid-boost kicks.
 * On 60 unseen flights: mean boost tilt 0.89 deg (was 1.14 with 8/0.5/0.8/20/20), and it stayed
 * better than those gains with the model wrong by +-30 % inertia, +-40 % thrust authority, 2 deg
 * servo slop, a 350 deg/s servo, 3x sensor noise and the CP 3 cm forward. More aggressive gains
 * did better on paper but oscillated when the rocket was lighter than modelled.
 * Without the nose ballast this finless rocket is too unstable for any gains. Re-run the sim
 * with YOUR measured part positions before flying. */
/* Re-checked 2026-10-31 on the TVCRocketV3 CAD model (978 g, 79.35 mm Blue Tube, 1062 mm, CG 605 mm
 * from the tip, gimbal pivot 397 mm behind the CG, -6.6 cal, K-9 V8 gimbal 9 mm horn / 42 mm lever,
 * measured F15 curve): these gains hold boost tilt to 1.0 deg nominal and 40/40 Monte Carlo flights
 * (worst 3.2 deg) with no gimbal saturation. */
#define EASYTVC_KP                 10.4f
#define EASYTVC_KI                 1.2f
#define EASYTVC_KD                 0.92f
#define EASYTVC_INTEGRATOR_LIMIT   0.15f
#define EASYTVC_OUTPUT_RATE_LIMIT  30.0f   /* full travel per second */
#define EASYTVC_DERIVATIVE_CUTOFF  30.0f   /* Hz */
/* Tilt (from vertical) at which TVC gives up and centres the gimbal for the rest of the burn. */
#define EASYTVC_MAX_TILT_DEG       25.0f

/* ---- Launch detection -----------------------------------------------------------------------
 * Specific force along the nose (g) that counts as ignition. Liftoff mass must keep the motor's
 * thrust well above this (E12: under ~0.4 kg; F15: see the TVCRocketV3 note below). */
/* TVCRocketV3 + F15 (978 g): the F15 peaks at 26.8 N (2.8 g) for ~0.1 s, then sustains 13 N (1.35 g).
 * 2.5 g was only crossed at the peak: in Studio's sim, 11/40 Monte Carlo flights never detected launch
 * (TVC stayed off) once the rocket was 60 g heavy or the motor 5 % weak. 1.8 g: 40/40, detected at ~0.38 s. */
#define EASYTVC_LAUNCH_ACCEL_G     1.8f
#define EASYTVC_LAUNCH_CONFIRM_MS  40U
/* Delay after confirmed launch before the gimbal starts moving (clears the pad/tower). */
#define EASYTVC_TVC_ENABLE_DELAY_MS 80U

/* ---- Recovery: parachute release servo ------------------------------------------------------
 * With a no-ejection motor (E12-0) the parachute must be released by a servo. 0 disables it.
 * The servo opens at detected apogee, or BACKUP_MS after launch at the latest (set this to your
 * simulated apogee time + ~0.5 s), and stays open. It also opens immediately if the board ever
 * reboots from a watchdog/fault reset, because that reset may have happened in the air. */
#define EASYTVC_CHUTE_CHANNEL      0
#define EASYTVC_CHUTE_CLOSED_US    1500
#define EASYTVC_CHUTE_OPEN_US      2000
/* TVCRocketV3 + F15: simulated apogee 5.2 s after ignition (53 m) -> backup at apogee + 0.5 s. */
#define EASYTVC_CHUTE_BACKUP_MS    5800U

/* ---- Recovery: ejection charge on a pyro terminal -------------------------------------------
 * PYRO_CHUTE_CHANNEL: 0 = none, 1 = P1, 3 = P3 (P2 is the MCU's SWD pin and can't be used). The
 * charge fires once, PYRO_CHUTE_DELAY_MS after detected apogee, or CHUTE_BACKUP_MS after launch at
 * the latest, for PYRO_FIRE_MS. Never before launch is detected, and never after a reboot.
 * The pin is only driven when EASYTVC_PYRO_OUTPUTS_COMPILED_IN is 1 (easytvc_safety.h): it is 0
 * until the board's pyro circuit has been bench-tested (docs/flight-guide.md). */
#define EASYTVC_PYRO_CHUTE_CHANNEL  1
#define EASYTVC_PYRO_CHUTE_DELAY_MS 0U
#define EASYTVC_PYRO_FIRE_MS        1000U
/* PYRO_DEPLOY_ALT_M > 0: fire on the way DOWN (after apogee) when the rocket passes this height above
 * the pad (+ PYRO_CHUTE_DELAY_MS), or PYRO_BACKUP_MS after launch at the latest (the timer, for a
 * failed barometer). 0 = fire at apogee (+ delay). Set the backup from the simulated time of reaching
 * that height on the way down, plus ~1 s. */
#define EASYTVC_PYRO_DEPLOY_ALT_M   0.0f
#define EASYTVC_PYRO_BACKUP_MS      5800U

/* ---- Pad arming -----------------------------------------------------------------------------
 * The flight image arms itself when the rocket has stood still and vertical for ARM_HOLD_MS.
 * Armed is shown by a solid green LED and a chirp every second. If it is tipped past
 * DISARM_TILT_DEG it disarms and starts over. */
#define EASYTVC_ARM_HOLD_MS        3000U
#define EASYTVC_ARM_MAX_TILT_DEG   10.0f
#define EASYTVC_DISARM_TILT_DEG    20.0f
/* Refuse to arm without a working SD card (no data = nothing learned from the flight). */
#define EASYTVC_REQUIRE_SD         1
