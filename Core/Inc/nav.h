#pragma once

typedef struct {
    float altitude_m;
    float vertical_speed_m_s;
    float accel_gain;
    float baro_gain;
} EasyTVCNavFilter;

void EasyTVC_NavInit(EasyTVCNavFilter *filter, float initial_altitude_m);
void EasyTVC_NavUpdate(EasyTVCNavFilter *filter,
                       float baro_altitude_m,
                       float axial_accel_m_s2,
                       float dt_s);
