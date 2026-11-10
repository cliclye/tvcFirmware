#include "servo_test.h"

#include <string.h>

void EasyTVC_ServoTestInit(EasyTVCServoTest *test)
{
    memset(test, 0, sizeof(*test));
}

/* "a,b": two signed integers within +/-limit */
static bool parse_pair(const char *s, int32_t limit, int32_t out[2])
{
    for (unsigned i = 0; i < 2U; ++i) {
        bool negative = false;
        if (*s == '-') { negative = true; ++s; }
        if (*s < '0' || *s > '9') return false;
        int32_t v = 0;
        for (; *s >= '0' && *s <= '9'; ++s) {
            v = v * 10 + (*s - '0');
            if (v > limit) return false;
        }
        out[i] = negative ? -v : v;
        if (i == 0U && *s++ != ',') return false;
    }
    return *s == '\0';
}

static EasyTVCServoSettings trimmed(const EasyTVCServoSettings *servo, int16_t trim_us)
{
    EasyTVCServoSettings s = *servo;
    s.center_us = (uint16_t)((int32_t)s.center_us + trim_us);
    return s;
}

static bool trim_fits(const EasyTVCServoSettings *servo, int32_t trim_us)
{
    /* the centre stays at least 50 us inside each endpoint, so the servo keeps travel both ways */
    const int32_t c = (int32_t)servo->center_us + trim_us;
    return c >= (int32_t)servo->minimum_us + 50 && c <= (int32_t)servo->maximum_us - 50;
}

bool EasyTVC_ServoTestCommand(EasyTVCServoTest *test, const EasyTVCSettings *settings, EasyTVCFlightState state,
                              const char *command, const char *payload, uint32_t now_ms, const char **error)
{
    *error = "COMMAND";
    if (state != EASYTVC_FLIGHT_IDLE) {
        *error = "NOT_IDLE";
        return false;
    }
    int32_t v[2];
    if (!strcmp(command, "SERVO")) {
        if (!parse_pair(payload, 1000, v)) { *error = "PAYLOAD"; return false; }
        test->mode = EASYTVC_SERVO_TEST_HOLD;
        test->command[0] = (float)v[0] / 1000.0f;
        test->command[1] = (float)v[1] / 1000.0f;
        test->since_ms = now_ms;
        return true;
    }
    if (!strcmp(command, "SWEEP")) {
        test->mode = EASYTVC_SERVO_TEST_SWEEP;
        test->since_ms = now_ms;
        return true;
    }
    if (!strcmp(command, "CENTER")) {
        test->mode = EASYTVC_SERVO_TEST_OFF;
        return true;
    }
    if (!strcmp(command, "TRIM")) {
        if (!parse_pair(payload, EASYTVC_SERVO_TEST_MAX_TRIM_US, v)) { *error = "PAYLOAD"; return false; }
        if (!trim_fits(&settings->servo[0], v[0]) || !trim_fits(&settings->servo[1], v[1])) {
            *error = "RANGE";
            return false;
        }
        test->trim_us[0] = (int16_t)v[0];
        test->trim_us[1] = (int16_t)v[1];
        return true;
    }
    return false;
}

bool EasyTVC_ServoTestApply(EasyTVCServoTest *test, const EasyTVCSettings *settings, EasyTVCFlightState state,
                            uint32_t now_ms, uint16_t pulse[2])
{
    if (state != EASYTVC_FLIGHT_IDLE) {
        EasyTVC_ServoTestInit(test);          /* leaving the pad ends the test and drops the trim */
        return false;
    }
    const uint32_t elapsed = now_ms - test->since_ms;
    float command[2] = {0.0f, 0.0f};
    switch (test->mode) {
    case EASYTVC_SERVO_TEST_HOLD:
        if (elapsed > EASYTVC_SERVO_TEST_HOLD_MS) {
            test->mode = EASYTVC_SERVO_TEST_OFF;   /* no refresh from the ground: back to centre */
            break;
        }
        command[0] = test->command[0];
        command[1] = test->command[1];
        break;
    case EASYTVC_SERVO_TEST_SWEEP: {
        static const float steps[4] = {0.0f, 1.0f, -1.0f, 0.0f};
        const uint32_t step = elapsed / EASYTVC_SERVO_TEST_STEP_MS;
        if (step >= 8U) {
            test->mode = EASYTVC_SERVO_TEST_OFF;
            break;
        }
        command[step / 4U] = steps[step % 4U];   /* servo A first, then B */
        break;
    }
    case EASYTVC_SERVO_TEST_OFF:
        break;
    }
    if (test->mode == EASYTVC_SERVO_TEST_OFF && test->trim_us[0] == 0 && test->trim_us[1] == 0) {
        return false;                         /* nothing to do: the normal output stands */
    }
    for (unsigned i = 0; i < 2U; ++i) {
        const EasyTVCServoSettings s = trimmed(&settings->servo[i], test->trim_us[i]);
        if (test->mode != EASYTVC_SERVO_TEST_OFF) {
            /* the trim only moves the centre: endpoints and direction stay as compiled */
            pulse[i] = EasyTVC_ServoPulse(&s, command[i]);
        } else {
            /* no test running: shift the normal output (centred on the pad, follows the tilt on
             * the bench image) by the trim, inside the endpoints */
            int32_t p = (int32_t)pulse[i] + test->trim_us[i];
            if (p < (int32_t)s.minimum_us) p = s.minimum_us;
            if (p > (int32_t)s.maximum_us) p = s.maximum_us;
            pulse[i] = (uint16_t)p;
        }
    }
    return true;
}
