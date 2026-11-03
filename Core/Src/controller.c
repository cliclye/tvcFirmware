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
    const bool attitude_ok = isfinite(norm) && fabsf(norm - 1.0f) < 0.02f;
    /* soft: this tick's sample is missing, old or untrusted (it comes back on its own) */
    const bool sample_ok = isfinite(dt_s) && dt_s > 0.0f && dt_s <= 0.02f && healthy &&
        (uint32_t)(now_ms - sample_ms) <= 20U &&
        (!c->sample_seen || (uint32_t)(sample_ms - c->last_sample_ms) > 0U) &&
        (!c->sample_seen || (uint32_t)(sample_ms - c->last_sample_ms) <= 20U + EASYTVC_CONTROL_HOLD_MS);
    const bool valid = config_ok && attitude_ok && sample_ok;
    const float vertical = attitude_ok ? 1.0f - 2.0f * (q->x*q->x + q->y*q->y) : -1.0f;
    /* Only latch while TVC is actually commanded. A glitch while sitting on the pad (a slow
     * barometer retry, one missed IMU read) used to latch here forever, and the flight then went
     * straight to FAULT with a centred gimbal at the moment of launch. */
    if (boost_enabled) {
        if (!config_ok || !attitude_ok || !physical_arm ||
            vertical < cosf(s->maximum_tilt_deg * 0.01745329252f)) c->fault_latched = true;
        if (!sample_ok) {
            if (!c->holding) { c->holding = true; c->hold_since_ms = now_ms; c->held_s = 0.0f; }
            if ((uint32_t)(now_ms - c->hold_since_ms) >= EASYTVC_CONTROL_HOLD_MS) c->fault_latched = true;
        }
    }
    if (sample_ok) {
        c->sample_seen = true;
        c->last_sample_ms = sample_ms;
    }
    out.enabled = config_ok && physical_arm && boost_enabled && !c->fault_latched;
    out.fault = c->fault_latched;
    if (out.enabled && !valid) {
        /* soft fault inside the hold window: keep steering on the last command */
        if (isfinite(dt_s) && dt_s > 0.0f) c->held_s += fminf(dt_s, 0.05f);
        out.command[0] = c->last_command[0];
        out.command[1] = c->last_command[1];
        for (unsigned i = 0; i < 2; ++i) out.pulse_us[i] = EasyTVC_ServoPulse(&s->servo[i], out.command[i]);
        return out;
    }
    /* the PID sees the real time since its last step, including a hold */
    const float pid_dt = fminf(dt_s + (c->holding ? c->held_s : 0.0f), 0.05f);
    c->holding = false;
    c->held_s = 0.0f;
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
        c->roll_rate_rad_s = roll_step / pid_dt;   /* previous_q is the last good sample, also across a hold */
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
        if (out.enabled) out.command[i] = EasyTVC_PidUpdate(&c->axis[i], &s->axis[i], 0.0f, error_angles[i], pid_dt);
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
    c->last_command[0] = out.command[0];
    c->last_command[1] = out.command[1];
    for (unsigned i = 0; i < 2; ++i)
        out.pulse_us[i] = config_ok ? EasyTVC_ServoPulse(&s->servo[i], out.command[i]) : 1500;
    return out;
}
