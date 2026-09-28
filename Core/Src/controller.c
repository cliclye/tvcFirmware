#include "controller.h"

#include <math.h>
#include <string.h>

void EasyTVC_ControllerReset(EasyTVCController *c)
{
    memset(c, 0, sizeof(*c));
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
    if (!valid || (boost_enabled && (!physical_arm ||
        vertical < cosf(s->maximum_tilt_deg * 0.01745329252f)))) c->fault_latched = true;
    c->sample_seen = true;
    c->last_sample_ms = sample_ms;
    out.enabled = valid && physical_arm && boost_enabled && !c->fault_latched;
    out.fault = c->fault_latched;
    /* Desired world-up vector in body coordinates. These errors do not command axial spin. */
    const float error_angles[2] = {
        atan2f(2.0f * (q->w*q->x + q->y*q->z), vertical),
        atan2f(2.0f * (q->w*q->y - q->x*q->z), vertical)
    };
    for (unsigned i = 0; i < 2; ++i) {
        if (out.enabled) out.command[i] = EasyTVC_PidUpdate(&c->axis[i], &s->axis[i], 0.0f, error_angles[i], dt_s);
        else EasyTVC_PidReset(&c->axis[i]);
        out.pulse_us[i] = config_ok ? EasyTVC_ServoPulse(&s->servo[i], out.command[i]) : 1500;
    }
    return out;
}
