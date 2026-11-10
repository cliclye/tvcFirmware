#include "nav.h"

#include <math.h>

void EasyTVC_NavInit(EasyTVCNavFilter *filter, float initial_altitude_m)
{
    filter->altitude_m = initial_altitude_m;
    filter->vertical_speed_m_s = 0.0f;
    filter->accel_gain = 1.0f;
    filter->altitude_rate = 10.0f;
    filter->speed_rate = 200.0f;
}

void EasyTVC_NavUpdate(EasyTVCNavFilter *filter,
                       float baro_altitude_m,
                       bool use_baro,
                       float axial_accel_m_s2,
                       float dt_s)
{
    if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.05f || !isfinite(axial_accel_m_s2)) {
        return;
    }

    filter->vertical_speed_m_s += axial_accel_m_s2 * dt_s * filter->accel_gain;
    filter->altitude_m += filter->vertical_speed_m_s * dt_s;

    if (use_baro && isfinite(baro_altitude_m)) {
        const float altitude_error = baro_altitude_m - filter->altitude_m;
        filter->altitude_m += filter->altitude_rate * dt_s * altitude_error;
        filter->vertical_speed_m_s += filter->speed_rate * dt_s * altitude_error;
    }
}
