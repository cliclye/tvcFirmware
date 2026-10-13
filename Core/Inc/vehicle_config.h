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
#define EASYTVC_KP                 1.2f
#define EASYTVC_KI                 0.0f
#define EASYTVC_KD                 0.2f
#define EASYTVC_INTEGRATOR_LIMIT   0.15f
#define EASYTVC_OUTPUT_RATE_LIMIT  4.0f    /* full travel per second */
#define EASYTVC_DERIVATIVE_CUTOFF  20.0f   /* Hz */
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
