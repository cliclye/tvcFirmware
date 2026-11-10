#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "easytvc_safety.h"
#include "settings.h"

/*
 * TVC gimbal test from the ground link, on the pad only: move servo A/B, sweep them, and try a
 * centre trim, so the mount can be checked and aligned without a cable.
 *
 * - Only the two TVC servos. The parachute servo is never touched.
 * - Only while the flight state is IDLE (the link refuses these commands in any other state as
 *   well). On any other state the test ends and the trim is cleared, so what flies is always the
 *   compiled-in calibration.
 * - Hold-to-move: a position lasts EASYTVC_SERVO_TEST_HOLD_MS after the last SERVO command, then
 *   the gimbal returns to centre. A lost link can only end a test, never start or extend one.
 * - Trim is RAM-only, limited to +/-EASYTVC_SERVO_TEST_MAX_TRIM_US, and must keep the centre
 *   inside the servo's endpoints. Make it permanent in Rocket setup and reinstall over USB-C.
 */

#define EASYTVC_SERVO_TEST_HOLD_MS      1000U
#define EASYTVC_SERVO_TEST_STEP_MS      300U    /* sweep: centre, max, min, centre per servo */
#define EASYTVC_SERVO_TEST_MAX_TRIM_US  100

typedef enum { EASYTVC_SERVO_TEST_OFF, EASYTVC_SERVO_TEST_HOLD, EASYTVC_SERVO_TEST_SWEEP } EasyTVCServoTestMode;

typedef struct {
    EasyTVCServoTestMode mode;
    float command[2];          /* -1..1 of travel, as the controller commands */
    int16_t trim_us[2];
    uint32_t since_ms;         /* last SERVO command, or the sweep's start */
} EasyTVCServoTest;

void EasyTVC_ServoTestInit(EasyTVCServoTest *test);

/* Handle SERVO "a_milli,b_milli" | SWEEP | CENTER | TRIM "a_us,b_us". Returns false (with an error
 * token in `error`) for anything invalid or when `state` is not IDLE. */
bool EasyTVC_ServoTestCommand(EasyTVCServoTest *test, const EasyTVCSettings *settings, EasyTVCFlightState state,
                              const char *command, const char *payload, uint32_t now_ms, const char **error);

/* Every loop tick, after the controller. Overrides `pulse` (servo A, B) while a test runs, applies
 * the trim while IDLE, and ends everything outside IDLE. Returns true when it changed the pulses. */
bool EasyTVC_ServoTestApply(EasyTVCServoTest *test, const EasyTVCSettings *settings, EasyTVCFlightState state,
                            uint32_t now_ms, uint16_t pulse[2]);
