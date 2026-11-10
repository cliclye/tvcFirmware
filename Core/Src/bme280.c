#include "bme280.h"

void EasyTVC_Bme280Init(EasyTVCBme280 *device, EasyTVCBus *bus)
{
    device->bus = bus;
    device->delay_ms = 0;
    device->identity_confirmed = false;
    device->temperature = 0;
    device->pressure = 0;
    device->humidity = 0;
    device->t_fine = 0;
}

bool EasyTVC_Bme280Probe(EasyTVCBme280 *device)
{
    uint8_t chip_id = 0U;
    if (EasyTVC_BusReadU8(device->bus, EASYTVC_BME280_CHIP_ID_REG, &chip_id) != 0) {
        device->identity_confirmed = false;
        return false;
    }
    device->identity_confirmed = (chip_id == EASYTVC_BME280_CHIP_ID);
    return device->identity_confirmed;
}

static uint16_t u16le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t s16le(const uint8_t *p)
{
    return (int16_t)u16le(p);
}

bool EasyTVC_Bme280Configure(EasyTVCBme280 *device)
{
    const uint8_t reset = 0xB6U;
    if (EasyTVC_BusWrite(device->bus, EASYTVC_BME280_RESET_REG, &reset, 1U) != 0) {
        return false;
    }
    if (device->delay_ms != 0) {
        device->delay_ms(10U);   /* NVM calibration copy after reset */
    }

    uint8_t cal[24];
    uint8_t h1 = 0U;
    uint8_t hcal[7];   /* 0xE1..0xE7 */
    if (EasyTVC_BusRead(device->bus, EASYTVC_BME280_DIG_T1_REG, cal, sizeof(cal)) != 0 ||
        EasyTVC_BusRead(device->bus, EASYTVC_BME280_DIG_H1_REG, &h1, 1U) != 0 ||
        EasyTVC_BusRead(device->bus, EASYTVC_BME280_DIG_H2_REG, hcal, sizeof(hcal)) != 0) {
        return false;
    }

    device->dig_T1 = u16le(&cal[0]);
    device->dig_T2 = s16le(&cal[2]);
    device->dig_T3 = s16le(&cal[4]);
    device->dig_P1 = u16le(&cal[6]);
    device->dig_P2 = s16le(&cal[8]);
    device->dig_P3 = s16le(&cal[10]);
    device->dig_P4 = s16le(&cal[12]);
    device->dig_P5 = s16le(&cal[14]);
    device->dig_P6 = s16le(&cal[16]);
    device->dig_P7 = s16le(&cal[18]);
    device->dig_P8 = s16le(&cal[20]);
    device->dig_P9 = s16le(&cal[22]);

    device->dig_H1 = h1;
    device->dig_H2 = s16le(&hcal[0]);
    device->dig_H3 = (int16_t)hcal[2];
    /* H4 = E4[7:0]<<4 | E5[3:0], H5 = E6[7:0]<<4 | E5[7:4]; E4/E6 are signed bytes. */
    device->dig_H4 = (int16_t)(((int16_t)(int8_t)hcal[3] * 16) | (hcal[4] & 0x0F));
    device->dig_H5 = (int16_t)(((int16_t)(int8_t)hcal[5] * 16) | (hcal[4] >> 4));
    device->dig_H6 = (int16_t)(int8_t)hcal[6];

    /* Sleep mode is required while writing CONFIG; CTRL_HUM only latches on the next CTRL_MEAS
     * write, so the order matters. IIR x4 smooths motor/airflow noise, standby 0.5 ms. */
    const uint8_t ctrl_hum = 0x01U;    /* humidity x1 */
    const uint8_t config = 0x08U;      /* t_sb 0.5 ms, filter 4 */
    const uint8_t ctrl_meas = 0x2FU;   /* temp x1, pressure x4, normal mode */
    if (EasyTVC_BusWrite(device->bus, EASYTVC_BME280_CTRL_HUM_REG, &ctrl_hum, 1U) != 0 ||
        EasyTVC_BusWrite(device->bus, EASYTVC_BME280_CONFIG_REG, &config, 1U) != 0 ||
        EasyTVC_BusWrite(device->bus, EASYTVC_BME280_CTRL_MEAS_REG, &ctrl_meas, 1U) != 0) {
        return false;
    }
    uint8_t readback = 0U;
    if (EasyTVC_BusReadU8(device->bus, EASYTVC_BME280_CTRL_MEAS_REG, &readback) != 0 ||
        readback != ctrl_meas) {
        return false;
    }
    return true;
}

int32_t EasyTVC_Bme280CompensateTemperature(const EasyTVCBme280 *d, int32_t adc_T, int32_t *t_fine)
{
    const int32_t var1 = ((((adc_T >> 3) - ((int32_t)d->dig_T1 * 2)) * (int32_t)d->dig_T2)) >> 11;
    const int32_t delta = (adc_T >> 4) - (int32_t)d->dig_T1;
    const int32_t var2 = (((delta * delta) >> 12) * (int32_t)d->dig_T3) >> 14;
    *t_fine = var1 + var2;
    return (*t_fine * 5 + 128) >> 8;
}

uint32_t EasyTVC_Bme280CompensatePressure(const EasyTVCBme280 *d, int32_t adc_P, int32_t t_fine)
{
    /* Bosch 32-bit integer formula (datasheet section 4.2.3). Left shifts are written as
     * multiplications to stay defined for negative operands. */
    int32_t var1 = (t_fine >> 1) - 64000;
    int32_t var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t)d->dig_P6;
    var2 = var2 + (var1 * (int32_t)d->dig_P5) * 2;
    var2 = (var2 >> 2) + (int32_t)d->dig_P4 * 65536;
    var1 = (((int32_t)d->dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
           ((((int32_t)d->dig_P2) * var1) >> 1);
    var1 = var1 >> 18;
    var1 = ((32768 + var1) * (int32_t)d->dig_P1) >> 15;
    if (var1 == 0) {
        return 0U;
    }
    uint32_t p = ((uint32_t)(1048576 - adc_P) - (uint32_t)(var2 >> 12)) * 3125U;
    if (p < 0x80000000UL) {
        p = (p << 1) / (uint32_t)var1;
    } else {
        p = (p / (uint32_t)var1) * 2U;
    }
    var1 = ((int32_t)d->dig_P9 * (int32_t)(((p >> 3) * (p >> 3)) >> 13)) >> 12;
    var2 = (((int32_t)(p >> 2)) * (int32_t)d->dig_P8) >> 13;
    return (uint32_t)((int32_t)p + ((var1 + var2 + (int32_t)d->dig_P7) >> 4));
}

static uint32_t compensate_humidity(const EasyTVCBme280 *d, int32_t adc_H, int32_t t_fine)
{
    int32_t v = t_fine - 76800;
    v = (((((adc_H * 16384) - ((int32_t)d->dig_H4 * 1048576) - ((int32_t)d->dig_H5 * v)) + 16384) >> 15) *
         (((((((v * (int32_t)d->dig_H6) >> 10) * (((v * (int32_t)d->dig_H3) >> 11) + 32768)) >> 10) +
            2097152) * (int32_t)d->dig_H2 + 8192) >> 14));
    v = v - (((((v >> 15) * (v >> 15)) >> 7) * (int32_t)d->dig_H1) >> 4);
    v = v < 0 ? 0 : v;
    v = v > 419430400 ? 419430400 : v;
    return (uint32_t)(v >> 12);
}

bool EasyTVC_Bme280Read(EasyTVCBme280 *device)
{
    /* One burst so pressure, temperature and humidity come from the same conversion. */
    uint8_t raw[8];
    if (EasyTVC_BusRead(device->bus, EASYTVC_BME280_PRESS_MSB_REG, raw, sizeof(raw)) != 0) {
        return false;
    }

    const int32_t adc_P = (int32_t)(((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) | (raw[2] >> 4));
    const int32_t adc_T = (int32_t)(((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) | (raw[5] >> 4));
    const int32_t adc_H = (int32_t)(((uint32_t)raw[6] << 8) | raw[7]);

    /* 0x80000 means "output disabled / not yet measured". */
    if (adc_P == 0x80000 || adc_T == 0x80000) {
        return false;
    }

    int32_t t_fine = 0;
    const int32_t temperature = EasyTVC_Bme280CompensateTemperature(device, adc_T, &t_fine);
    const uint32_t pressure = EasyTVC_Bme280CompensatePressure(device, adc_P, t_fine);
    if (pressure < EASYTVC_BME280_PRESSURE_MIN_PA || pressure > EASYTVC_BME280_PRESSURE_MAX_PA) {
        return false;
    }

    device->adc_p = (uint32_t)adc_P;
    device->adc_t = (uint32_t)adc_T;
    device->adc_h = (uint16_t)adc_H;
    device->t_fine = t_fine;
    device->temperature = temperature;
    device->pressure = pressure;
    device->humidity = adc_H == 0x8000 ? 0U : compensate_humidity(device, adc_H, t_fine);
    return true;
}
