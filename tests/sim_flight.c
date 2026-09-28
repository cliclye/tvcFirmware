#include <stdio.h>

#include "app.h"

int main(void)
{
    EasyTVCApp app;
    EasyTVC_AppInit(&app, 0U);

    EasyTVCAppInput input = {
        .gyro_rad_s = {0.0f, 0.02f, 0.0f},
        .accel_m_s2 = {0.0f, 0.0f, 9.80665f},
        .baro_altitude_m = 0.0f,
        .sensors_healthy = true,
        .physical_arm_qualified = true,
        .software_arm_request = true,
        .accel_is_gravity_reference = true,
        .now_ms = 0U,
        .dt_s = 0.01f,
    };

    EasyTVCFlightState previous = EASYTVC_FLIGHT_IDLE;
    for (uint32_t step = 0U; step < 2500U; ++step) {
        const float time_s = (float)step * 0.01f;
        input.now_ms = step * 10U;

        if (time_s < 0.5f) {
            input.accel_m_s2[2] = 9.80665f;
            input.baro_altitude_m = 0.0f;
            input.accel_is_gravity_reference = true;
        } else if (time_s < 2.5f) {
            input.accel_m_s2[2] = 40.0f;
            input.baro_altitude_m = (time_s - 0.5f) * 80.0f;
            input.accel_is_gravity_reference = false;
        } else if (time_s < 8.0f) {
            const float coast_t = time_s - 2.5f;
            input.accel_m_s2[2] = 1.0f;
            input.baro_altitude_m = 160.0f + coast_t * 40.0f - 4.9f * coast_t * coast_t;
            input.accel_is_gravity_reference = false;
        } else {
            input.accel_m_s2[2] = 1.0f;
            input.baro_altitude_m = 20.0f;
            input.accel_is_gravity_reference = false;
        }

        const EasyTVCAppOutput output = EasyTVC_AppStep(&app, &input);
        if (output.flight.state != previous) {
            printf("t=%u state=%u tvc=%d pyro_d=%d pyro_m=%d\n",
                   input.now_ms,
                   (unsigned)output.flight.state,
                   output.flight.tvc_enabled ? 1 : 0,
                   output.pyro.request_drogue_output ? 1 : 0,
                   output.pyro.request_main_output ? 1 : 0);
            fputs(output.telemetry, stdout);
            previous = output.flight.state;
        }
        if (step + 1U == 2500U) {
            printf("final state=%u pyro_lock=%d log_count=%zu\n",
                   (unsigned)output.flight.state,
                   output.pyro.request_drogue_output || output.pyro.request_main_output,
                   app.log.count);
        }
    }

    return 0;
}
