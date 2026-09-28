#pragma once

#include <stddef.h>
#include <stdint.h>

#include "easytvc_safety.h"

typedef struct {
    uint32_t now_ms;
    EasyTVCFlightState state;
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float altitude_m;
    float vertical_speed_m_s;
    float pitch_command;
    float yaw_command;
    bool tvc_enabled;
    bool physical_arm_qualified;
    bool pyro_lock_active;
} EasyTVCTelemetrySample;

size_t EasyTVC_TelemetryFormat(const EasyTVCTelemetrySample *sample,
                               char *buffer,
                               size_t capacity);
