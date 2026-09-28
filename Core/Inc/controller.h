#pragma once

#include "settings.h"
#include "attitude.h"
#include <stdint.h>

typedef struct {
    EasyTVCPid axis[2];
    bool fault_latched;
    uint32_t last_sample_ms;
    bool sample_seen;
} EasyTVCController;

typedef struct {
    float command[2];
    uint16_t pulse_us[2];
    bool enabled;
    bool fault;
} EasyTVCControlOutput;

void EasyTVC_ControllerReset(EasyTVCController *controller);
/* A new sensor sample is required for every call. Reset only while disarmed.
 * Positive controller command assumes positive body-axis torque; prove direction on a bench. */
EasyTVCControlOutput EasyTVC_ControllerStep(EasyTVCController *controller,
    const EasyTVCSettings *settings, const EasyTVCQuaternion *orientation,
    uint32_t now_ms, uint32_t sample_ms, float dt_s, bool healthy,
    bool physical_arm, bool boost_enabled);
