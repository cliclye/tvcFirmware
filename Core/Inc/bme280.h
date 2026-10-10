#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bus.h"

#define EASYTVC_BME280_CHIP_ID_REG 0xD0U
#define EASYTVC_BME280_CHIP_ID     0x60U
/* On EasyTVC, I2C1 must enable STM32 internal pull-ups (vendor requirement). */

#define EASYTVC_BME280_RESET_REG     0xE0U
#define EASYTVC_BME280_CTRL_HUM_REG  0xF2U
#define EASYTVC_BME280_CTRL_MEAS_REG 0xF4U
#define EASYTVC_BME280_CONFIG_REG    0xF5U
#define EASYTVC_BME280_PRESS_MSB_REG 0xF7U   /* P[3] T[3] H[2] are contiguous from here */
#define EASYTVC_BME280_TEMP_MSB_REG  0xFAU
#define EASYTVC_BME280_HUM_MSB_REG   0xFDU

/* Calibration registers */
#define EASYTVC_BME280_DIG_T1_REG 0x88U   /* 24 bytes: T1..T3, P1..P9 */
#define EASYTVC_BME280_DIG_P1_REG 0x8EU
#define EASYTVC_BME280_DIG_H1_REG 0xA1U
#define EASYTVC_BME280_DIG_H2_REG 0xE1U   /* 7 bytes: H2, H3, H4, H5, H6 */

/* Plausible range for a rocket on or above the pad, used to reject garbage reads. */
#define EASYTVC_BME280_PRESSURE_MIN_PA 30000U
#define EASYTVC_BME280_PRESSURE_MAX_PA 110000U

typedef struct {
    EasyTVCBus *bus;
    void (*delay_ms)(uint32_t ms);   /* optional; used by Configure */
    bool identity_confirmed;
    int32_t temperature;     /* 0.01 degC */
    uint32_t pressure;       /* Pa */
    uint32_t humidity;       /* Q22.10 %RH (divide by 1024) */
    int32_t t_fine;
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

/* Bosch datasheet integer compensation, exposed so it can be tested against known values. */
int32_t EasyTVC_Bme280CompensateTemperature(const EasyTVCBme280 *device, int32_t adc_T, int32_t *t_fine);
uint32_t EasyTVC_Bme280CompensatePressure(const EasyTVCBme280 *device, int32_t adc_P, int32_t t_fine);
