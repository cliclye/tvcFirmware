#include "pyro_test.h"

#include <string.h>

void EasyTVC_PyroTestInit(EasyTVCPyroTest *test)
{
    memset(test, 0, sizeof(*test));
}

static bool allowed(EasyTVCFlightState state, bool rocket_armed, const char **error)
{
    if (state != EASYTVC_FLIGHT_IDLE) { *error = "NOT_IDLE"; return false; }
    if (rocket_armed) { *error = "ARMED"; return false; }
    return true;
}

bool EasyTVC_PyroTestArm(EasyTVCPyroTest *test, uint8_t channel, EasyTVCFlightState state, bool rocket_armed,
                         uint32_t now_ms, uint32_t seed, const char **error)
{
    if (!allowed(state, rocket_armed, error)) return false;
    if (channel != 1U && channel != 3U) { *error = "CHANNEL"; return false; }
    /* a code the ground can only know from this reply: 1000..9999, never the previous one */
    uint32_t x = seed * 2654435761u ^ now_ms * 40503u ^ 0x9E37u;
    uint16_t code = (uint16_t)(1000U + (x >> 7) % 9000U);
    if (code == test->code) code = (uint16_t)(code == 9999U ? 1000U : code + 1U);
    test->channel = channel;
    test->code = code;
    test->armed_ms = now_ms;
    return true;
}

bool EasyTVC_PyroTestFire(EasyTVCPyroTest *test, uint8_t channel, uint16_t code, uint32_t pulse_ms,
                          EasyTVCFlightState state, bool rocket_armed, uint32_t now_ms, const char **error)
{
    if (!allowed(state, rocket_armed, error)) return false;
    if (test->channel == 0U || test->channel != channel) { *error = "NOT_ARMED_FOR_TEST"; return false; }
    if ((uint32_t)(now_ms - test->armed_ms) > EASYTVC_PYRO_TEST_WINDOW_MS) {
        test->channel = 0;
        *error = "EXPIRED";
        return false;
    }
    if (code != test->code) { test->channel = 0; *error = "CODE"; return false; }   /* one try per code */
    if (pulse_ms < 20U || pulse_ms > EASYTVC_PYRO_TEST_MAX_MS) { *error = "LENGTH"; return false; }
    test->channel = 0;                           /* used up */
    test->pulse_channel = channel;
    test->pulse_start_ms = now_ms;
    test->pulse_ms = pulse_ms;
    return true;
}

uint8_t EasyTVC_PyroTestStep(EasyTVCPyroTest *test, EasyTVCFlightState state, bool rocket_armed, uint32_t now_ms)
{
    if (state != EASYTVC_FLIGHT_IDLE || rocket_armed) {
        const uint16_t code = test->code;
        EasyTVC_PyroTestInit(test);              /* the pad arming or a launch ends any test at once */
        test->code = code;
        return 0;
    }
    if (test->pulse_channel && (uint32_t)(now_ms - test->pulse_start_ms) >= test->pulse_ms) test->pulse_channel = 0;
    return test->pulse_channel;
}
