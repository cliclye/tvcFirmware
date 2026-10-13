#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "easytvc_safety.h"

/*
 * Pad arming without an arm switch: the rocket arms once it has stood vertical and still for
 * hold_ms, and disarms if it is tipped over before launch. Once launched the arm stays latched.
 * The still window is also used to re-take the gyro bias right before flight.
 */
typedef struct {
    uint32_t hold_ms;
    float cos_arm_tilt;          /* cos(max tilt from vertical allowed to arm) */
    float cos_disarm_tilt;       /* cos(tilt that disarms) */
    float still_gyro_rad_s;      /* |gyro| below this counts as still */
    uint32_t disarm_confirm_ms;
} EasyTVCPadArmConfig;

typedef struct {
    bool armed;
    bool still_active;
    bool tip_active;
    uint32_t still_since_ms;
    uint32_t tip_since_ms;
    uint32_t samples;
    float gyro_sum[3];
    /* ~0.1 s low-pass of accel and |gyro| for the upright/still checks, so one noisy sample or a
     * gust shaking the stand does not restart the hold timer. */
    bool filter_ready;
    float accel_lp[3];
    float gyro_lp;
} EasyTVCPadArm;

void EasyTVC_PadArmInit(EasyTVCPadArm *pad);

/* Call once per loop BEFORE the flight state machine step, with the state it is currently in.
 * `ready` = everything else needed to fly is OK; `fresh` = this loop read a new IMU sample.
 * Accel/gyro are body frame (gyro bias-corrected); `gyro_raw` is the uncorrected sensor reading.
 * Returns true on the step it arms, with the new gyro bias (sensor frame) in bias_out. */
bool EasyTVC_PadArmStep(EasyTVCPadArm *pad, const EasyTVCPadArmConfig *config,
                        EasyTVCFlightState state, bool ready, bool fresh,
                        const float accel_body_m_s2[3], const float gyro_body_rad_s[3],
                        const float gyro_raw_rad_s[3], uint32_t now_ms, float bias_out[3]);

/* True while still qualifying (for a fast-blinking LED). */
bool EasyTVC_PadArmQualifying(const EasyTVCPadArm *pad);

/* Parachute release: at detected apogee (or later states), or backup_ms after launch at the
 * latest. Never before launch. */
bool EasyTVC_ChuteShouldOpen(bool launched, uint32_t ms_since_launch, EasyTVCFlightState state,
                             bool apogee_event, uint32_t backup_ms);
