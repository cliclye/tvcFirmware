#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "easytvc_safety.h"

/*
 * Ground test of a pyro channel from Studio, for a test load (an LED or bulb), never a charge.
 *
 * - IDLE only and only while the rocket is NOT armed (the link refuses it in every other state too).
 * - Two steps: PYROARM picks the channel and gets a one-off code; PYROFIRE must repeat channel and
 *   code within EASYTVC_PYRO_TEST_WINDOW_MS. One code, one pulse.
 * - The pulse is short (EASYTVC_PYRO_TEST_MAX_MS at most) and stops at once if the state changes or
 *   the rocket arms.
 * - The pin is only driven when EASYTVC_PYRO_OUTPUTS_COMPILED_IN is 1; locked, the test still runs
 *   and says so, but the output stays off.
 * - P1 and P3 only (P2 is the SWD pin).
 */

#define EASYTVC_PYRO_TEST_WINDOW_MS 10000U
#define EASYTVC_PYRO_TEST_MAX_MS    1000U

typedef struct {
    uint8_t channel;            /* armed for this channel (0 = not armed) */
    uint16_t code;
    uint32_t armed_ms;
    uint8_t pulse_channel;      /* pulsing this channel (0 = none) */
    uint32_t pulse_start_ms, pulse_ms;
} EasyTVCPyroTest;

void EasyTVC_PyroTestInit(EasyTVCPyroTest *test);

/* Returns false with an error token: NOT_IDLE, ARMED, CHANNEL. `seed` varies the code. */
bool EasyTVC_PyroTestArm(EasyTVCPyroTest *test, uint8_t channel, EasyTVCFlightState state, bool rocket_armed,
                         uint32_t now_ms, uint32_t seed, const char **error);
/* NOT_IDLE, ARMED, NOT_ARMED_FOR_TEST, CODE, EXPIRED, LENGTH. */
bool EasyTVC_PyroTestFire(EasyTVCPyroTest *test, uint8_t channel, uint16_t code, uint32_t pulse_ms,
                          EasyTVCFlightState state, bool rocket_armed, uint32_t now_ms, const char **error);
/* Every tick: which channel the test drives now (0 = none). Ends everything outside IDLE / when armed. */
uint8_t EasyTVC_PyroTestStep(EasyTVCPyroTest *test, EasyTVCFlightState state, bool rocket_armed, uint32_t now_ms);
