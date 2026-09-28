#pragma once

#include <stdint.h>
#include <stddef.h>

/* Stable, scalar-only FFI. Simulation uses the same controller as app.c.
 * plant: inertia X/Y, thrust N, thrust arm m, max gimbal deg, servo tau s,
 * wind m/s, peak gust m/s, temperature C, area m2, Cd, CP arm m, initial tilt deg.
 * row: seconds, tilt X/Y deg, command X/Y, pulse X/Y us, wind m/s, fault. */
#define EASYTVC_SIM_PLANT_FIELDS 13U
#define EASYTVC_SIM_COLUMNS 9U
int EasyTVC_StudioValidate(const uint8_t *packet, size_t length);
int EasyTVC_StudioSimulate(const uint8_t *packet, size_t length,
    const float plant[EASYTVC_SIM_PLANT_FIELDS], float *rows, unsigned count);
