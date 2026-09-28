#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "easytvc_safety.h"

typedef struct {
    float launch_accel_g;
    uint32_t launch_confirm_ms;
    uint32_t minimum_boost_ms;
    float burnout_axial_accel_g;
    uint32_t minimum_apogee_ms;
    uint32_t failsafe_apogee_ms;
    float main_deploy_altitude_m;
    float landed_speed_m_s;
    uint32_t landed_confirm_ms;
    uint32_t tvc_enable_delay_ms;
} EasyTVCFlightConfig;

typedef struct {
    uint32_t now_ms;
    bool sensors_healthy;
    bool physical_arm_qualified;
    bool software_arm_request;
    float acceleration_magnitude_g;
    float axial_acceleration_g;
    float altitude_m;
    float vertical_speed_m_s;
} EasyTVCFlightInput;

typedef struct {
    EasyTVCFlightState state;
    bool log_active;
    bool tvc_enabled;
    /* One-step events. They are not persistent output levels. */
    bool drogue_deploy_requested;
    bool main_deploy_requested;
    bool fault;
} EasyTVCFlightOutput;

typedef struct {
    EasyTVCFlightConfig config;
    EasyTVCFlightState state;
    uint32_t state_entered_ms;
    uint32_t launch_candidate_ms;
    uint32_t launch_ms;
    uint32_t landed_candidate_ms;
    bool launch_candidate_active;
    bool landed_candidate_active;
    bool drogue_requested;
    bool main_requested;
} EasyTVCFlightMachine;

void EasyTVC_FlightInit(EasyTVCFlightMachine *machine,
                        const EasyTVCFlightConfig *config,
                        uint32_t now_ms);
EasyTVCFlightOutput EasyTVC_FlightStep(EasyTVCFlightMachine *machine,
                                       const EasyTVCFlightInput *input);
