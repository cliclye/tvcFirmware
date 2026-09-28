#pragma once

#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator_limit;
    float output_limit;
    float output_rate_limit;
    /* Zero disables filtering. Otherwise a first-order derivative LPF in Hz. */
    float derivative_cutoff_hz;
} EasyTVCPidConfig;

typedef struct {
    float integrator;
    float previous_measurement;
    float previous_output;
    float filtered_derivative;
    bool initialized;
} EasyTVCPid;

void EasyTVC_PidReset(EasyTVCPid *pid);
bool EasyTVC_PidConfigValid(const EasyTVCPidConfig *config);
float EasyTVC_PidUpdate(EasyTVCPid *pid,
                        const EasyTVCPidConfig *config,
                        float target,
                        float measurement,
                        float dt_s);
