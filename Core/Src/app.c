#include "app.h"

#include <math.h>

static float vector_norm(const float value[3])
{
    return sqrtf(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
}

void EasyTVC_AppInit(EasyTVCApp *app, uint32_t now_ms)
{
    const EasyTVCFlightConfig flight_config = {
        .launch_accel_g = 2.5f,
        .launch_confirm_ms = 40U,
        .minimum_boost_ms = 200U,
        .burnout_axial_accel_g = 0.4f,
        .minimum_apogee_ms = 800U,
        .failsafe_apogee_ms = 8000U,
        .main_deploy_altitude_m = 200.0f,
        .landed_speed_m_s = 1.0f,
        .landed_confirm_ms = 400U,
        .tvc_enable_delay_ms = 80U,
    };
    app->flight_config = flight_config;
    app->pyro_config.pulse_ms = 500U;
    EasyTVC_FlightInit(&app->flight, &app->flight_config, now_ms);
    EasyTVC_AttitudeInit(&app->attitude, 2.0f, 0.01f);
    EasyTVC_NavInit(&app->nav, 0.0f);
    EasyTVC_ControllerReset(&app->controller);
    EasyTVC_SettingsDefault(&app->settings);
    EasyTVC_PyroSequencerInit(&app->pyro);
    EasyTVC_LogRingInit(&app->log, app->log_storage, 32U);
}

bool EasyTVC_AppApplySettings(EasyTVCApp *app, const EasyTVCSettings *settings, bool physical_arm)
{
    if (physical_arm || app->flight.state != EASYTVC_FLIGHT_IDLE ||
        !EasyTVC_SettingsValid(settings)) return false;
    app->settings = *settings;
    EasyTVC_ControllerReset(&app->controller);
    return true;
}

EasyTVCAppOutput EasyTVC_AppStep(EasyTVCApp *app, const EasyTVCAppInput *input)
{
    EasyTVCAppOutput output;
    const float gravity = 9.80665f;

    bool healthy = input->sensors_healthy && input->sensor_age_ms <= 20U &&
        isfinite(input->dt_s) && input->dt_s > 0.0f && input->dt_s <= 0.02f &&
        isfinite(input->baro_altitude_m) && fabsf(input->baro_altitude_m) < 100000.0f;
    for (unsigned i = 0; i < 3; ++i) {
        healthy = healthy && isfinite(input->gyro_rad_s[i]) &&
            fabsf(input->gyro_rad_s[i]) <= 35.0f && isfinite(input->accel_m_s2[i]) &&
            fabsf(input->accel_m_s2[i]) <= 235.36f;
    }

    if (healthy) {
        EasyTVC_AttitudeUpdate(&app->attitude, input->gyro_rad_s, input->accel_m_s2,
            input->accel_is_gravity_reference && app->flight.state <= EASYTVC_FLIGHT_ARMED &&
            fabsf(vector_norm(input->accel_m_s2) - gravity) < gravity * 0.15f, input->dt_s);
        const EasyTVCQuaternion q = app->attitude.orientation;
        /* Body +Z is noseward; rotate specific force into world vertical. */
        const float vertical_accel =
            2.0f * (q.x * q.z - q.w * q.y) * input->accel_m_s2[0] +
            2.0f * (q.w * q.x + q.y * q.z) * input->accel_m_s2[1] +
            (q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z) * input->accel_m_s2[2];
        EasyTVC_NavUpdate(&app->nav, input->baro_altitude_m,
                         vertical_accel - gravity, input->dt_s);
    }

    const EasyTVCFlightInput flight_input = {
        .now_ms = input->now_ms,
        .sensors_healthy = healthy,
        .physical_arm_qualified = input->physical_arm_qualified,
        .software_arm_request = input->software_arm_request,
        .acceleration_magnitude_g = healthy ? vector_norm(input->accel_m_s2) / gravity : 0.0f,
        .axial_acceleration_g = healthy ? input->accel_m_s2[2] / gravity : 0.0f,
        .altitude_m = app->nav.altitude_m,
        .vertical_speed_m_s = app->nav.vertical_speed_m_s,
    };

    output.flight = EasyTVC_FlightStep(&app->flight, &flight_input);
    output.euler = EasyTVC_AttitudeEuler(&app->attitude);
    output.nav = app->nav;
    output.control = EasyTVC_ControllerStep(&app->controller, &app->settings,
        &app->attitude.orientation, input->now_ms, input->now_ms - input->sensor_age_ms,
        input->dt_s, healthy, input->physical_arm_qualified, output.flight.tvc_enabled);
    if (output.control.fault && output.flight.state != EASYTVC_FLIGHT_IDLE &&
        output.flight.state != EASYTVC_FLIGHT_LANDED) {
        app->flight.state = EASYTVC_FLIGHT_FAULT;
        output.flight.state = EASYTVC_FLIGHT_FAULT;
        output.flight.fault = true;
        output.flight.drogue_deploy_requested = false;
        output.flight.main_deploy_requested = false;
    }
    output.flight.tvc_enabled = output.control.enabled;
    output.tvc.pitch_command = output.control.command[0];
    output.tvc.yaw_command = output.control.command[1];
    for (unsigned i = 0; i < 4; ++i) {
        output.tvc.servo_normalized[i] = 0.5f;
    }
    for (unsigned i = 0; i < 2 && EasyTVC_SettingsValid(&app->settings); ++i) {
        output.tvc.servo_normalized[app->settings.servo[i].channel - 1] =
            fminf(1.0f, fmaxf(0.0f, ((float)output.control.pulse_us[i] - 1000.0f) / 1000.0f));
    }
    output.pyro = EasyTVC_PyroSequencerStep(&app->pyro,
                                            &app->pyro_config,
                                            output.flight.state,
                                            input->physical_arm_qualified,
                                            output.flight.drogue_deploy_requested,
                                            output.flight.main_deploy_requested,
                                            input->now_ms);

    if (output.flight.log_active) {
        EasyTVCLogRecord record = {
            .timestamp_ms = input->now_ms,
            .acceleration_m_s2 = {
                input->accel_m_s2[0],
                input->accel_m_s2[1],
                input->accel_m_s2[2],
            },
            .gyro_rad_s = {
                input->gyro_rad_s[0],
                input->gyro_rad_s[1],
                input->gyro_rad_s[2],
            },
            .altitude_m = app->nav.altitude_m,
            .vertical_speed_m_s = app->nav.vertical_speed_m_s,
            .state = (uint8_t)output.flight.state,
            .flags = (uint8_t)((output.flight.tvc_enabled ? 1U : 0U) |
                               (output.pyro.request_drogue_output ? 2U : 0U) |
                               (output.pyro.request_main_output ? 4U : 0U)),
        };
        EasyTVC_LogRingAppend(&app->log, &record);
    }

    const EasyTVCTelemetrySample sample = {
        .now_ms = input->now_ms,
        .state = output.flight.state,
        .roll_rad = output.euler.roll_rad,
        .pitch_rad = output.euler.pitch_rad,
        .yaw_rad = output.euler.yaw_rad,
        .altitude_m = app->nav.altitude_m,
        .vertical_speed_m_s = app->nav.vertical_speed_m_s,
        .pitch_command = output.tvc.pitch_command,
        .yaw_command = output.tvc.yaw_command,
        .tvc_enabled = output.flight.tvc_enabled,
        .physical_arm_qualified = input->physical_arm_qualified,
        .pyro_lock_active = !EASYTVC_PYRO_OUTPUTS_COMPILED_IN,
    };
    EasyTVC_TelemetryFormat(&sample, output.telemetry, sizeof(output.telemetry));
    return output;
}
