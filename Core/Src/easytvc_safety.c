#include "easytvc_safety.h"

bool EasyTVC_PyroPolicyAllows(EasyTVCFlightState state,
                               EasyTVCPyroEvent event,
                               bool physical_arm_qualified)
{
    /* A build without an explicitly reviewed output layer can never fire. */
#if !EASYTVC_PYRO_OUTPUTS_COMPILED_IN
    (void)state;
    (void)event;
    (void)physical_arm_qualified;
    return false;
#else
    if (!physical_arm_qualified) {
        return false;
    }

    switch (event) {
    case EASYTVC_PYRO_EVENT_DROGUE:
        return state == EASYTVC_FLIGHT_APOGEE;
    case EASYTVC_PYRO_EVENT_MAIN:
        return state == EASYTVC_FLIGHT_DESCENT;
    default:
        return false;
    }
#endif
}
