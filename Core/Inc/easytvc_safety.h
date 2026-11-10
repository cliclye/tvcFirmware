#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Policy only. Pin nets exist in board_pins.h; this file still does not
 * configure or write any physical GPIO. Vendor pyro nets are untested.
 */
/* The master pyro lock: 0 = no pyro pin is ever driven. It is 0 for every image except
 * easytvc_pyrobench (CMakeLists.txt), the bench-only image for testing the pyro channels with a
 * test load: it never arms or flies, so only Studio's pyro ground test can switch a channel. The
 * flight image unlocks only by changing this line after that test has passed. */
#ifndef EASYTVC_PYRO_OUTPUTS_COMPILED_IN
#define EASYTVC_PYRO_OUTPUTS_COMPILED_IN 0
#endif

typedef enum {
    EASYTVC_FLIGHT_IDLE = 0,
    EASYTVC_FLIGHT_ARMED,
    EASYTVC_FLIGHT_BOOST,
    EASYTVC_FLIGHT_COAST,
    EASYTVC_FLIGHT_APOGEE,
    EASYTVC_FLIGHT_DESCENT,
    EASYTVC_FLIGHT_LANDED,
    EASYTVC_FLIGHT_FAULT,
} EasyTVCFlightState;

typedef enum {
    EASYTVC_PYRO_EVENT_DROGUE = 0,
    EASYTVC_PYRO_EVENT_MAIN,
} EasyTVCPyroEvent;

/*
 * Policy-only gate. It never writes a GPIO. A hardware driver may call it only
 * after its pin, output-off polarity, and physical-arm sense net are confirmed.
 */
bool EasyTVC_PyroPolicyAllows(EasyTVCFlightState state,
                               EasyTVCPyroEvent event,
                               bool physical_arm_qualified);
