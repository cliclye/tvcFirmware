#include "attitude.h"

#include <math.h>

static float clamp_unit(float value)
{
    if (value > 1.0f) {
        return 1.0f;
    }
    if (value < -1.0f) {
        return -1.0f;
    }
    return value;
}

static void normalize_quaternion(EasyTVCQuaternion *quaternion)
{
    const float norm = sqrtf(quaternion->w * quaternion->w +
                             quaternion->x * quaternion->x +
                             quaternion->y * quaternion->y +
                             quaternion->z * quaternion->z);
    if (norm > 0.0f) {
        quaternion->w /= norm;
        quaternion->x /= norm;
        quaternion->y /= norm;
        quaternion->z /= norm;
    }
}

void EasyTVC_AttitudeInit(EasyTVCAttitudeFilter *filter,
                          float proportional_gain,
                          float integral_gain)
{
    filter->orientation.w = 1.0f;
    filter->orientation.x = 0.0f;
    filter->orientation.y = 0.0f;
    filter->orientation.z = 0.0f;
    filter->integral_error[0] = 0.0f;
    filter->integral_error[1] = 0.0f;
    filter->integral_error[2] = 0.0f;
    filter->proportional_gain = proportional_gain;
    filter->integral_gain = integral_gain;
}

void EasyTVC_AttitudeUpdate(EasyTVCAttitudeFilter *filter,
                            const float gyro_rad_s[3],
                            const float accel_m_s2[3],
                            bool accel_is_gravity_reference,
                            float dt_s)
{
    if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.05f) {
        return;
    }
    for (unsigned i = 0; i < 3; ++i) {
        if (!isfinite(gyro_rad_s[i]) || !isfinite(accel_m_s2[i])) {
            return;
        }
    }

    float gx = gyro_rad_s[0];
    float gy = gyro_rad_s[1];
    float gz = gyro_rad_s[2];
    EasyTVCQuaternion q = filter->orientation;

    if (accel_is_gravity_reference) {
        const float acceleration_norm = sqrtf(accel_m_s2[0] * accel_m_s2[0] +
                                              accel_m_s2[1] * accel_m_s2[1] +
                                              accel_m_s2[2] * accel_m_s2[2]);
        if (acceleration_norm > 0.01f) {
            const float ax = accel_m_s2[0] / acceleration_norm;
            const float ay = accel_m_s2[1] / acceleration_norm;
            const float az = accel_m_s2[2] / acceleration_norm;

            const float vx = 2.0f * (q.x * q.z - q.w * q.y);
            const float vy = 2.0f * (q.w * q.x + q.y * q.z);
            const float vz = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;

            const float error_x = ay * vz - az * vy;
            const float error_y = az * vx - ax * vz;
            const float error_z = ax * vy - ay * vx;

            filter->integral_error[0] += error_x * filter->integral_gain * dt_s;
            filter->integral_error[1] += error_y * filter->integral_gain * dt_s;
            filter->integral_error[2] += error_z * filter->integral_gain * dt_s;

            gx += filter->proportional_gain * error_x + filter->integral_error[0];
            gy += filter->proportional_gain * error_y + filter->integral_error[1];
            gz += filter->proportional_gain * error_z + filter->integral_error[2];
        }
    }

    const float half_dt = 0.5f * dt_s;
    const float qw = q.w;
    const float qx = q.x;
    const float qy = q.y;
    const float qz = q.z;
    q.w += (-qx * gx - qy * gy - qz * gz) * half_dt;
    q.x += (qw * gx + qy * gz - qz * gy) * half_dt;
    q.y += (qw * gy - qx * gz + qz * gx) * half_dt;
    q.z += (qw * gz + qx * gy - qy * gx) * half_dt;
    normalize_quaternion(&q);
    filter->orientation = q;
}

EasyTVCEulerAngles EasyTVC_AttitudeEuler(const EasyTVCAttitudeFilter *filter)
{
    const EasyTVCQuaternion q = filter->orientation;
    EasyTVCEulerAngles euler;
    euler.roll_rad = atan2f(2.0f * (q.w * q.x + q.y * q.z),
                            1.0f - 2.0f * (q.x * q.x + q.y * q.y));
    euler.pitch_rad = asinf(clamp_unit(2.0f * (q.w * q.y - q.z * q.x)));
    euler.yaw_rad = atan2f(2.0f * (q.w * q.z + q.x * q.y),
                           1.0f - 2.0f * (q.y * q.y + q.z * q.z));
    return euler;
}
