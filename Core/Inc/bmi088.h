#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bus.h"

/* Datasheet identity registers. These are not board pin assignments. */
#define EASYTVC_BMI088_ACCEL_CHIP_ID_REG 0x00U
#define EASYTVC_BMI088_ACCEL_CHIP_ID     0x1EU
#define EASYTVC_BMI088_GYRO_CHIP_ID_REG  0x00U
#define EASYTVC_BMI088_GYRO_CHIP_ID      0x0FU

/* Accelerometer registers */
#define EASYTVC_BMI088_ACCEL_X_LSB_REG    0x12U
#define EASYTVC_BMI088_ACCEL_CONF_REG     0x40U
#define EASYTVC_BMI088_ACCEL_RANGE_REG    0x41U
#define EASYTVC_BMI088_ACCEL_PWR_CONF_REG 0x7CU
#define EASYTVC_BMI088_ACCEL_PWR_CTRL_REG 0x7DU
#define EASYTVC_BMI088_ACCEL_SOFTRESET_REG 0x7EU

#define EASYTVC_BMI088_ACCEL_CONF_400HZ_NORMAL 0xAAU
#define EASYTVC_BMI088_ACCEL_RANGE_24G         0x03U
#define EASYTVC_BMI088_ACCEL_ENABLE            0x04U

/* Gyroscope registers */
#define EASYTVC_BMI088_GYRO_RATE_X_LSB_REG 0x02U
#define EASYTVC_BMI088_GYRO_RANGE_REG      0x0FU
#define EASYTVC_BMI088_GYRO_BANDWIDTH_REG  0x10U
#define EASYTVC_BMI088_GYRO_LPM1_REG       0x11U

#define EASYTVC_BMI088_GYRO_RANGE_2000DPS  0x00U
#define EASYTVC_BMI088_GYRO_ODR400_BW47    0x03U

/* Raw LSB -> SI. Accel +-24 g: 24 * 9.80665 / 32768. Gyro +-2000 dps: 2000 * pi/180 / 32768. */
#define EASYTVC_BMI088_ACCEL_M_S2_PER_LSB  (24.0f * 9.80665f / 32768.0f)
#define EASYTVC_BMI088_GYRO_RAD_S_PER_LSB  (2000.0f * 0.017453292519943f / 32768.0f)

typedef struct {
    EasyTVCBus *accel;
    EasyTVCBus *gyro;
    /* Optional blocking delay used by Configure for the datasheet power-up times. */
    void (*delay_ms)(uint32_t ms);
    bool identity_confirmed;
    bool accel_saturated;
    bool gyro_saturated;
    /* Raw sensor counts, sensor frame. */
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    /* Scaled, sensor frame (no mounting rotation applied). */
    float accel_m_s2[3];
    float gyro_rad_s[3];
} EasyTVCBmi088;

void EasyTVC_Bmi088Init(EasyTVCBmi088 *device, EasyTVCBus *accel, EasyTVCBus *gyro);
bool EasyTVC_Bmi088Probe(EasyTVCBmi088 *device);
/* Powers up the accelerometer (it resets to suspend), selects ranges/bandwidth and reads
 * every written setting back. Returns false on any bus error or mismatch. */
bool EasyTVC_Bmi088Configure(EasyTVCBmi088 *device);
bool EasyTVC_Bmi088Read(EasyTVCBmi088 *device);
