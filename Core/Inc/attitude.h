#pragma once

#include <stdbool.h>

typedef struct {
    float w;
    float x;
    float y;
    float z;
} EasyTVCQuaternion;

typedef struct {
    EasyTVCQuaternion orientation;
    float integral_error[3];
    float proportional_gain;
    float integral_gain;
} EasyTVCAttitudeFilter;

typedef struct {
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
} EasyTVCEulerAngles;

void EasyTVC_AttitudeInit(EasyTVCAttitudeFilter *filter,
                          float proportional_gain,
                          float integral_gain);

/* gyro is radians/second. accel is m/s^2 and is used only when trustworthy. */
void EasyTVC_AttitudeUpdate(EasyTVCAttitudeFilter *filter,
                            const float gyro_rad_s[3],
                            const float accel_m_s2[3],
                            bool accel_is_gravity_reference,
                            float dt_s);

EasyTVCEulerAngles EasyTVC_AttitudeEuler(const EasyTVCAttitudeFilter *filter);
