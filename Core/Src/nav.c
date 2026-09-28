#include "nav.h"

#include <math.h>

void EasyTVC_NavInit(EasyTVCNavFilter *filter, float initial_altitude_m)
{
    filter->altitude_m = initial_altitude_m;
    filter->vertical_speed_m_s = 0.0f;
    filter->accel_gain = 1.0f;
    filter->baro_gain = 0.05f;
}

void EasyTVC_NavUpdate(EasyTVCNavFilter *filter,
                       float baro_altitude_m,
                       float axial_accel_m_s2,
                       float dt_s)
{
    if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.05f ||
        !isfinite(baro_altitude_m) || !isfinite(axial_accel_m_s2)) {
        return;
    }

    filter->vertical_speed_m_s += axial_accel_m_s2 * dt_s * filter->accel_gain;
    filter->altitude_m += filter->vertical_speed_m_s * dt_s;

    const float altitude_error = baro_altitude_m - filter->altitude_m;
    filter->altitude_m += filter->baro_gain * altitude_error;
    filter->vertical_speed_m_s += filter->baro_gain * altitude_error / dt_s * 0.1f;
}
