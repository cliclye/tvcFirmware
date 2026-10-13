#include "pad_arm.h"

#include <math.h>

#define GRAVITY 9.80665f

static float norm3(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

void EasyTVC_PadArmInit(EasyTVCPadArm *pad)
{
    *pad = (EasyTVCPadArm){0};
}

bool EasyTVC_PadArmStep(EasyTVCPadArm *pad, const EasyTVCPadArmConfig *config,
                        EasyTVCFlightState state, bool ready, bool fresh,
                        const float accel[3], const float gyro[3],
                        const float gyro_raw[3], uint32_t now_ms, float bias_out[3])
{
    const float accel_norm = norm3(accel);

    if (state == EASYTVC_FLIGHT_IDLE) {
        pad->armed = false;
        pad->tip_active = false;
        const bool upright = isfinite(accel_norm) && accel_norm > 0.92f * GRAVITY &&
                             accel_norm < 1.08f * GRAVITY && accel[2] > accel_norm * config->cos_arm_tilt;
        const bool quiet = norm3(gyro) < config->still_gyro_rad_s;
        if (!(ready && fresh && upright && quiet)) {
            pad->still_active = false;
            return false;
        }
        if (!pad->still_active) {
            pad->still_active = true;
            pad->still_since_ms = now_ms;
            pad->samples = 0;
            pad->gyro_sum[0] = pad->gyro_sum[1] = pad->gyro_sum[2] = 0.0f;
        }
        for (unsigned axis = 0; axis < 3U; ++axis) {
            pad->gyro_sum[axis] += gyro_raw[axis];
        }
        ++pad->samples;
        if ((uint32_t)(now_ms - pad->still_since_ms) < config->hold_ms) {
            return false;
        }
        for (unsigned axis = 0; axis < 3U; ++axis) {
            bias_out[axis] = pad->gyro_sum[axis] / (float)pad->samples;
        }
        pad->armed = true;
        pad->still_active = false;
        return true;
    }

    if (state == EASYTVC_FLIGHT_ARMED) {
        pad->still_active = false;
        if (!ready) {
            pad->armed = false;
            return false;
        }
        const bool tipped = accel_norm > 0.3f * GRAVITY && accel[2] < accel_norm * config->cos_disarm_tilt;
        if (!tipped || !fresh) {
            pad->tip_active = false;
        } else if (!pad->tip_active) {
            pad->tip_active = true;
            pad->tip_since_ms = now_ms;
        } else if ((uint32_t)(now_ms - pad->tip_since_ms) >= config->disarm_confirm_ms) {
            pad->armed = false;
        }
    }
    /* In flight (BOOST onwards) the arm stays latched. */
    return false;
}

bool EasyTVC_PadArmQualifying(const EasyTVCPadArm *pad)
{
    return pad->still_active;
}

bool EasyTVC_ChuteShouldOpen(bool launched, uint32_t ms_since_launch, EasyTVCFlightState state,
                             bool apogee_event, uint32_t backup_ms)
{
    if (!launched) {
        return false;
    }
    return apogee_event || state == EASYTVC_FLIGHT_APOGEE || state == EASYTVC_FLIGHT_DESCENT ||
           state == EASYTVC_FLIGHT_LANDED || ms_since_launch >= backup_ms;
}
