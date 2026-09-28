#pragma once

#include <stdbool.h>

#include "pid.h"

typedef struct {
    EasyTVCPidConfig pitch;
    EasyTVCPidConfig yaw;
    float servo_center;
    float servo_throw;
} EasyTVCTvcConfig;

typedef struct {
    EasyTVCPid pitch;
    EasyTVCPid yaw;
} EasyTVCTvc;

typedef struct {
    float pitch_command;
    float yaw_command;
    float servo_normalized[4];
} EasyTVCTvcOutput;

void EasyTVC_TvcInit(EasyTVCTvc *tvc);
EasyTVCTvcOutput EasyTVC_TvcUpdate(EasyTVCTvc *tvc,
                                  const EasyTVCTvcConfig *config,
                                  float pitch_rad,
                                  float yaw_rad,
                                  bool enabled,
                                  float dt_s);
