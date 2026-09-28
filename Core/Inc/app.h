#pragma once

#include "attitude.h"
#include "easytvc_safety.h"
#include "flight_fsm.h"
#include "logger.h"
#include "nav.h"
#include "pyro_seq.h"
#include "telemetry.h"
#include "tvc.h"
#include "controller.h"

typedef struct {
    float gyro_rad_s[3];
    float accel_m_s2[3];
    float baro_altitude_m;
    bool sensors_healthy;
    bool physical_arm_qualified;
    bool software_arm_request;
    bool accel_is_gravity_reference;
    uint32_t now_ms;
    /* Age of the last complete, valid sensor sample, supplied by the driver. */
    uint32_t sensor_age_ms;
    float dt_s;
} EasyTVCAppInput;

typedef struct {
    EasyTVCFlightOutput flight;
    EasyTVCTvcOutput tvc;
    EasyTVCControlOutput control;
    EasyTVCPyroCommand pyro;
    EasyTVCEulerAngles euler;
    EasyTVCNavFilter nav;
    char telemetry[160];
} EasyTVCAppOutput;

typedef struct {
    EasyTVCFlightMachine flight;
    EasyTVCAttitudeFilter attitude;
    EasyTVCNavFilter nav;
    EasyTVCController controller;
    EasyTVCSettings settings;
    EasyTVCPyroSequencer pyro;
    EasyTVCLogRing log;
    EasyTVCLogRecord log_storage[32];
    EasyTVCFlightConfig flight_config;
    EasyTVCPyroConfig pyro_config;
} EasyTVCApp;

void EasyTVC_AppInit(EasyTVCApp *app, uint32_t now_ms);
bool EasyTVC_AppApplySettings(EasyTVCApp *app, const EasyTVCSettings *settings, bool physical_arm);
EasyTVCAppOutput EasyTVC_AppStep(EasyTVCApp *app, const EasyTVCAppInput *input);
