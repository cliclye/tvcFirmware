#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bus.h"

/* Datasheet identity registers. These are not board pin assignments. */
#define EASYTVC_BMI088_ACCEL_CHIP_ID_REG 0x00U
#define EASYTVC_BMI088_ACCEL_CHIP_ID     0x1EU
#define EASYTVC_BMI088_GYRO_CHIP_ID_REG  0x00U
#define EASYTVC_BMI088_GYRO_CHIP_ID      0x0FU

typedef struct {
    EasyTVCBus *accel;
    EasyTVCBus *gyro;
    bool identity_confirmed;
} EasyTVCBmi088;

void EasyTVC_Bmi088Init(EasyTVCBmi088 *device, EasyTVCBus *accel, EasyTVCBus *gyro);
bool EasyTVC_Bmi088Probe(EasyTVCBmi088 *device);
