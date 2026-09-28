#include "studio_sim.h"
#include "controller.h"

#include <math.h>

int EasyTVC_StudioValidate(const uint8_t *packet, size_t length)
{
    EasyTVCSettings settings;
    return EasyTVC_SettingsDecode(&settings, packet, length) ? 1 : 0;
}

int EasyTVC_StudioSimulate(const uint8_t *packet, size_t length,
    const float p[EASYTVC_SIM_PLANT_FIELDS], float *rows, unsigned count)
{
    EasyTVCSettings settings;
    if (!EasyTVC_SettingsDecode(&settings, packet, length) || !p || !rows || count < 2 || count > 6001) return -1;
    const float lower[] = {0.0001f, 0.0001f, 0.1f, 0.01f, 0.1f, 0.005f, 0, 0, -30, 0.0001f, 0.01f, 0.001f, -20};
    const float upper[] = {10, 10, 1000, 2, 15, 1, 30, 40, 60, 1, 3, 2, 20};
    for (unsigned i = 0; i < EASYTVC_SIM_PLANT_FIELDS; ++i)
        if (!isfinite(p[i]) || p[i] < lower[i] || p[i] > upper[i]) return -2;
    if (p[7] < p[6]) return -2;
    const float radians = 0.01745329252f, dt = 0.005f;
    const float density = 101325.0f / (287.05f * (273.15f + p[8]));
    EasyTVCController controller;
    EasyTVC_ControllerReset(&controller);
    float angle[2] = {p[12] * radians, p[12] * -0.5f * radians};
    float rate[2] = {0}, gimbal[2] = {0};
    /* Fixed step 200 Hz; publish 100 Hz. This is a rotational bench model,
     * not a prediction of a complete rocket flight. */
    for (unsigned step = 0; step < count * 2; ++step) {
        const float time = step * dt;
        const float cx = cosf(angle[0] / 2), sx = sinf(angle[0] / 2);
        const float cy = cosf(angle[1] / 2), sy = sinf(angle[1] / 2);
        const EasyTVCQuaternion q = {.w = cx*cy, .x = sx*cy, .y = cx*sy, .z = -sx*sy};
        const EasyTVCControlOutput out = EasyTVC_ControllerStep(&controller, &settings, &q,
            step * 5U, step * 5U, dt, true, true, true);
        const float gust = fmaxf(0.0f, sinf(time * 3.1f));
        const float wind = p[6] + (p[7] - p[6]) * gust;
        if (step % 2 == 0) {
            float *row = rows + (step / 2) * EASYTVC_SIM_COLUMNS;
            row[0] = time;
            row[1] = angle[0] / radians; row[2] = angle[1] / radians;
            row[3] = out.command[0]; row[4] = out.command[1];
            row[5] = out.pulse_us[0]; row[6] = out.pulse_us[1];
            row[7] = wind; row[8] = out.fault ? 1.0f : 0.0f;
        }
        for (unsigned i = 0; i < 2; ++i) {
            gimbal[i] += dt / (p[5] + dt) * (out.command[i] * p[4] * radians - gimbal[i]);
            const float aerodynamic_torque = 0.5f * density * wind*wind * p[9] * p[10] * p[11] *
                (i == 0 ? 1.0f : sinf(time * 1.7f) * 0.5f);
            const float torque = p[2] * p[3] * sinf(gimbal[i]) + aerodynamic_torque;
            rate[i] += (torque / p[i]) * dt;
            angle[i] += rate[i] * dt;
            /* Stop numeric divergence after a fault; report faulted rows. */
            if (!isfinite(angle[i]) || fabsf(angle[i]) > 3.141592654f) {
                angle[i] = copysignf(3.141592654f, angle[i]); rate[i] = 0.0f;
            }
        }
    }
    return (int)count;
}
