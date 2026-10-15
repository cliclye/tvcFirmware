#include "controller.h"

#include <math.h>
#include <string.h>

static void rotate_pair(float *x, float *y, float angle)
{
    const float c = cosf(angle), s = sinf(angle), nx = c * *x - s * *y;
    *y = s * *x + c * *y;
    *x = nx;
}

void EasyTVC_ControllerReset(EasyTVCController *c)
{
    memset(c, 0, sizeof(*c));
}

void EasyTVC_ControllerTilt(const EasyTVCQuaternion *q, float tilt_rad[2])
{
    /* World-up expressed in body coordinates, as rotations about body X and Y. */
    const float vertical = 1.0f - 2.0f * (q->x*q->x + q->y*q->y);
    tilt_rad[0] = atan2f(2.0f * (q->w*q->x + q->y*q->z), vertical);
    tilt_rad[1] = atan2f(2.0f * (q->w*q->y - q->x*q->z), vertical);
}

EasyTVCControlOutput EasyTVC_ControllerStep(EasyTVCController *c,
    const EasyTVCSettings *s, const EasyTVCQuaternion *q,
    uint32_t now_ms, uint32_t sample_ms, float dt_s, bool healthy,
    bool physical_arm, bool boost_enabled)
{
    EasyTVCControlOutput out = {0};
    const bool config_ok = EasyTVC_SettingsValid(s);
    float norm = q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z;
    const bool valid = config_ok && isfinite(norm) && fabsf(norm - 1.0f) < 0.02f &&
        isfinite(dt_s) && dt_s > 0.0f && dt_s <= 0.02f && healthy &&
        (uint32_t)(now_ms - sample_ms) <= 20U &&
        (!c->sample_seen || (uint32_t)(sample_ms - c->last_sample_ms) > 0U) &&
        (!c->sample_seen || (uint32_t)(sample_ms - c->last_sample_ms) <= 20U);
    const float vertical = 1.0f - 2.0f * (q->x*q->x + q->y*q->y);
    /* Only latch while TVC is actually commanded. A glitch while sitting on the pad (a slow
     * barometer retry, one missed IMU read) used to latch here forever, and the flight then went
     * straight to FAULT with a centred gimbal at the moment of launch. */
    if (boost_enabled && (!valid || !physical_arm ||
        vertical < cosf(s->maximum_tilt_deg * 0.01745329252f))) c->fault_latched = true;
    c->sample_seen = true;
    c->last_sample_ms = sample_ms;
    out.enabled = valid && physical_arm && boost_enabled && !c->fault_latched;
    out.fault = c->fault_latched;
    /* Desired world-up vector in body coordinates. These errors do not command axial spin. */
    float error_angles[2];
    EasyTVC_ControllerTilt(q, error_angles);
    /* Roll since the last sample: z of the body-frame increment conj(q_prev) * q. */
    float roll_step = 0.0f;
    if (valid && c->roll_rate_valid) {
        const EasyTVCQuaternion a = c->previous_q;
        const float rw = a.w*q->w + a.x*q->x + a.y*q->y + a.z*q->z;
        const float rz = a.w*q->z - a.z*q->w + a.y*q->x - a.x*q->y;
        roll_step = 2.0f * atan2f(rz, rw);
        if (fabsf(roll_step) > 3.0f) roll_step = 0.0f;   /* > 170 deg in one sample: not credible */
        c->roll_rate_rad_s = roll_step / dt_s;
    } else {
        c->roll_rate_rad_s = 0.0f;
    }
    c->previous_q = *q;
    c->roll_rate_valid = valid;
    if (out.enabled && roll_step != 0.0f && c->axis[0].initialized && c->axis[1].initialized) {
        /* A world-fixed tilt seen from a body that rolled by +roll_step turns by -roll_step. */
        rotate_pair(&c->axis[0].previous_measurement, &c->axis[1].previous_measurement, -roll_step);
        rotate_pair(&c->axis[0].filtered_derivative, &c->axis[1].filtered_derivative, -roll_step);
        rotate_pair(&c->axis[0].previous_output, &c->axis[1].previous_output, -roll_step);
    }
    for (unsigned i = 0; i < 2; ++i) {
        if (out.enabled) out.command[i] = EasyTVC_PidUpdate(&c->axis[i], &s->axis[i], 0.0f, error_angles[i], dt_s);
        else EasyTVC_PidReset(&c->axis[i]);
    }
    if (out.enabled) {
        /* The body keeps rolling while the servos move: aim the torque where the body will be. */
        rotate_pair(&out.command[0], &out.command[1], -c->roll_rate_rad_s * EASYTVC_ROLL_LEAD_S);
        for (unsigned i = 0; i < 2; ++i) {
            const float limit = s->axis[i].output_limit;
            out.command[i] = fminf(limit, fmaxf(-limit, out.command[i]));
        }
    }
    for (unsigned i = 0; i < 2; ++i)
        out.pulse_us[i] = config_ok ? EasyTVC_ServoPulse(&s->servo[i], out.command[i]) : 1500;
    return out;
}
