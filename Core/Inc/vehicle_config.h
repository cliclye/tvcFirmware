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

#define EASYTVC_NOSE_AXIS EASYTVC_AXIS_POS_Z

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
#define EASYTVC_SERVO_A_CHANNEL   1
#define EASYTVC_SERVO_A_MIN_US    1250
#define EASYTVC_SERVO_A_CENTER_US 1500
#define EASYTVC_SERVO_A_MAX_US    1750
#define EASYTVC_SERVO_A_DIRECTION 1

#define EASYTVC_SERVO_B_CHANNEL   2
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
 * thrust well above this: Estes E12 (~12 N average) needs a liftoff mass under ~0.4 kg. */
#define EASYTVC_LAUNCH_ACCEL_G     2.5f
#define EASYTVC_LAUNCH_CONFIRM_MS  40U
/* Delay after confirmed launch before the gimbal starts moving (clears the pad/tower). */
#define EASYTVC_TVC_ENABLE_DELAY_MS 80U

/* ---- Recovery: parachute release servo ------------------------------------------------------
 * With a no-ejection motor (E12-0) the parachute must be released by a servo. 0 disables it.
 * The servo opens at detected apogee, or BACKUP_MS after launch at the latest (set this to your
 * simulated apogee time + ~0.5 s), and stays open. It also opens immediately if the board ever
 * reboots from a watchdog/fault reset, because that reset may have happened in the air. */
#define EASYTVC_CHUTE_CHANNEL      3
#define EASYTVC_CHUTE_CLOSED_US    1500
#define EASYTVC_CHUTE_OPEN_US      2000
#define EASYTVC_CHUTE_BACKUP_MS    7000U

/* ---- Pad arming -----------------------------------------------------------------------------
 * The flight image arms itself when the rocket has stood still and vertical for ARM_HOLD_MS.
 * Armed is shown by a solid green LED and a chirp every second. If it is tipped past
 * DISARM_TILT_DEG it disarms and starts over. */
#define EASYTVC_ARM_HOLD_MS        3000U
#define EASYTVC_ARM_MAX_TILT_DEG   10.0f
#define EASYTVC_DISARM_TILT_DEG    20.0f
/* Refuse to arm without a working SD card (no data = nothing learned from the flight). */
#define EASYTVC_REQUIRE_SD         1
