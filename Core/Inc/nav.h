#pragma once

#include <stdbool.h>

typedef struct {
    float altitude_m;
    float vertical_speed_m_s;
    float accel_gain;
    /* Barometer correction as rates, so it does not depend on the loop period: the altitude is
     * pulled toward the barometer at altitude_rate (1/s) and the speed at speed_rate (1/s^2) times
     * the altitude error. 10 and 200 are what the old per-tick gains were at 200 Hz. */
    float altitude_rate;
    float speed_rate;
} EasyTVCNavFilter;

void EasyTVC_NavInit(EasyTVCNavFilter *filter, float initial_altitude_m);
/* use_baro false (barometer stale or missing): inertial only. A frozen barometer reading must not
 * be followed: it drags the speed to zero while the rocket climbs, which reads as apogee. */
void EasyTVC_NavUpdate(EasyTVCNavFilter *filter,
                       float baro_altitude_m,
                       bool use_baro,
                       float axial_accel_m_s2,
                       float dt_s);
