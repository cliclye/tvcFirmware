#include "pid.h"

#include <math.h>

bool EasyTVC_PidConfigValid(const EasyTVCPidConfig *c)
{
    return c && isfinite(c->kp) && c->kp >= 0.0f && c->kp <= 100.0f &&
        isfinite(c->ki) && c->ki >= 0.0f && c->ki <= 100.0f &&
        isfinite(c->kd) && c->kd >= 0.0f && c->kd <= 100.0f &&
        isfinite(c->integrator_limit) && c->integrator_limit >= 0.0f &&
        c->integrator_limit <= 1.0f &&
        isfinite(c->output_limit) && c->output_limit > 0.0f && c->output_limit <= 10.0f &&
        isfinite(c->output_rate_limit) && c->output_rate_limit >= 0.0f &&
        c->output_rate_limit <= 100.0f &&
        isfinite(c->derivative_cutoff_hz) && c->derivative_cutoff_hz >= 0.0f &&
        c->derivative_cutoff_hz <= 100.0f;
}

static float clamp(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

void EasyTVC_PidReset(EasyTVCPid *pid)
{
    pid->integrator = 0.0f;
    pid->previous_measurement = 0.0f;
    pid->previous_output = 0.0f;
    pid->filtered_derivative = 0.0f;
    pid->initialized = false;
}

float EasyTVC_PidUpdate(EasyTVCPid *pid,
                        const EasyTVCPidConfig *config,
                        float target,
                        float measurement,
                        float dt_s)
{
    if (!EasyTVC_PidConfigValid(config) || !isfinite(target) || !isfinite(measurement) ||
        !isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.05f) {
        EasyTVC_PidReset(pid);
        return 0.0f;
    }

    const float error = target - measurement;
    if (!pid->initialized) {
        pid->previous_measurement = measurement;
        pid->initialized = true;
    }

    const float derivative = -(measurement - pid->previous_measurement) / dt_s;
    const float alpha = config->derivative_cutoff_hz > 0.0f ?
        dt_s / (dt_s + 1.0f / (6.283185307f * config->derivative_cutoff_hz)) : 1.0f;
    pid->filtered_derivative += alpha * (derivative - pid->filtered_derivative);
    const float proposed_i = clamp(pid->integrator + error * config->ki * dt_s,
                                   config->integrator_limit);
    const float pd = config->kp * error + config->kd * pid->filtered_derivative;
    const float requested = pd + proposed_i;
    float output = requested;
    output = clamp(output, config->output_limit);

    if (config->output_rate_limit > 0.0f) {
        const float allowed_delta = config->output_rate_limit * dt_s;
        output = pid->previous_output + clamp(output - pid->previous_output, allowed_delta);
    }

    /* Do not accumulate error into either amplitude or slew saturation.
     * An increment which unwinds existing saturation is still permitted. */
    if ((requested - output) * (proposed_i - pid->integrator) <= 0.0f) {
        pid->integrator = proposed_i;
    }

    if (!isfinite(output) || !isfinite(pid->filtered_derivative)) {
        EasyTVC_PidReset(pid);
        return 0.0f;
    }

    pid->previous_measurement = measurement;
    pid->previous_output = output;
    return output;
}
