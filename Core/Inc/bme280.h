#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bus.h"

#define EASYTVC_BME280_CHIP_ID_REG 0xD0U
#define EASYTVC_BME280_CHIP_ID     0x60U
/* On EasyTVC, I2C1 must enable STM32 internal pull-ups (vendor requirement). */

/* BME280 data registers */
#define EASYTVC_BME280_CTRL_MEAS_REG 0xF4U
#define EASYTVC_BME280_PRESS_MSB_REG 0xF7U
#define EASYTVC_BME280_TEMP_MSB_REG  0xFAU
#define EASYTVC_BME280_HUM_MSB_REG   0xFDU

/* BME280 calibration registers */
#define EASYTVC_BME280_DIG_T1_REG 0x88U
#define EASYTVC_BME280_DIG_P1_REG 0x8EU
#define EASYTVC_BME280_DIG_H1_REG 0xA1U

typedef struct {
    EasyTVCBus *bus;
    bool identity_confirmed;
    int32_t temperature;
    uint32_t pressure;
    uint32_t humidity;
    /* Calibration data (simplified) */
    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2, dig_H3, dig_H4, dig_H5, dig_H6;
} EasyTVCBme280;

void EasyTVC_Bme280Init(EasyTVCBme280 *device, EasyTVCBus *bus);
bool EasyTVC_Bme280Probe(EasyTVCBme280 *device);
bool EasyTVC_Bme280Configure(EasyTVCBme280 *device);
bool EasyTVC_Bme280Read(EasyTVCBme280 *device);
