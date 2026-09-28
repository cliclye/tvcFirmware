#include "tvc.h"

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

void EasyTVC_TvcInit(EasyTVCTvc *tvc)
{
    EasyTVC_PidReset(&tvc->pitch);
    EasyTVC_PidReset(&tvc->yaw);
}

EasyTVCTvcOutput EasyTVC_TvcUpdate(EasyTVCTvc *tvc,
                                  const EasyTVCTvcConfig *config,
                                  float pitch_rad,
                                  float yaw_rad,
                                  bool enabled,
                                  float dt_s)
{
    EasyTVCTvcOutput output;
    float pitch_command = 0.0f;
    float yaw_command = 0.0f;

    const bool valid = isfinite(pitch_rad) && isfinite(yaw_rad) &&
        isfinite(dt_s) && dt_s > 0.0f && dt_s <= 0.05f &&
        EasyTVC_PidConfigValid(&config->pitch) && EasyTVC_PidConfigValid(&config->yaw) &&
        isfinite(config->servo_center) && config->servo_center >= 0.0f &&
        config->servo_center <= 1.0f && isfinite(config->servo_throw) &&
        config->servo_throw >= 0.0f && config->servo_throw <= 0.5f;
    if (enabled && valid) {
        pitch_command = EasyTVC_PidUpdate(&tvc->pitch, &config->pitch, 0.0f, pitch_rad, dt_s);
        yaw_command = EasyTVC_PidUpdate(&tvc->yaw, &config->yaw, 0.0f, yaw_rad, dt_s);
    } else {
        EasyTVC_TvcInit(tvc);
    }

    pitch_command = clamp_unit(pitch_command);
    yaw_command = clamp_unit(yaw_command);

    output.pitch_command = pitch_command;
    output.yaw_command = yaw_command;
    const float center = valid ? config->servo_center : 0.5f;
    const float travel = valid ? config->servo_throw : 0.0f;
    const float commands[4] = {pitch_command, -pitch_command, yaw_command, -yaw_command};
    for (unsigned i = 0; i < 4; ++i) {
        const float value = center + travel * commands[i];
        output.servo_normalized[i] = fminf(1.0f, fmaxf(0.0f, value));
    }
    return output;
}
