#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bus.h"

/* Datasheet identity registers. These are not board pin assignments. */
#define EASYTVC_BMI088_ACCEL_CHIP_ID_REG 0x00U
#define EASYTVC_BMI088_ACCEL_CHIP_ID     0x1EU
#define EASYTVC_BMI088_GYRO_CHIP_ID_REG  0x00U
#define EASYTVC_BMI088_GYRO_CHIP_ID      0x0FU

/* BMI088 data registers */
#define EASYTVC_BMI088_ACCEL_X_LSB_REG  0x12U
#define EASYTVC_BMI088_ACCEL_CTRL_REG   0x7DU
#define EASYTVC_BMI088_GYRO_RATE_X_LSB_REG 0x02U
#define EASYTVC_BMI088_GYRO_CTRL_REG    0x15U

typedef struct {
    EasyTVCBus *accel;
    EasyTVCBus *gyro;
    bool identity_confirmed;
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
} EasyTVCBmi088;

void EasyTVC_Bmi088Init(EasyTVCBmi088 *device, EasyTVCBus *accel, EasyTVCBus *gyro);
bool EasyTVC_Bmi088Probe(EasyTVCBmi088 *device);
bool EasyTVC_Bmi088Configure(EasyTVCBmi088 *device);
bool EasyTVC_Bmi088Read(EasyTVCBmi088 *device);
