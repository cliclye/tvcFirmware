#include "bme280.h"

void EasyTVC_Bme280Init(EasyTVCBme280 *device, EasyTVCBus *bus)
{
    device->bus = bus;
    device->identity_confirmed = false;
    device->temperature = 0;
    device->pressure = 0;
    device->humidity = 0;
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

bool EasyTVC_Bme280Configure(EasyTVCBme280 *device)
{
    /* Set to normal mode, oversampling x1 */
    uint8_t ctrl_meas = 0x27;  /* Normal mode, temp oversampling x1, pressure oversampling x1 */
    if (EasyTVC_BusWrite(device->bus, EASYTVC_BME280_CTRL_MEAS_REG, &ctrl_meas, 1) != 0) {
        return false;
    }

    /* Read calibration data (simplified - just read a few registers) */
    uint8_t cal_data[24];
    uint8_t hum_cal_data[7];
    if (EasyTVC_BusRead(device->bus, EASYTVC_BME280_DIG_T1_REG, cal_data, 24) != 0) {
        return false;
    }
    if (EasyTVC_BusRead(device->bus, EASYTVC_BME280_DIG_H1_REG, hum_cal_data, 1) != 0) {
        return false;
    }
    if (EasyTVC_BusRead(device->bus, 0xE1, hum_cal_data + 1, 7) != 0) {
        return false;
    }

    /* Parse temperature calibration data */
    device->dig_T1 = (uint16_t)((cal_data[1] << 8) | cal_data[0]);
    device->dig_T2 = (int16_t)((cal_data[3] << 8) | cal_data[2]);
    device->dig_T3 = (int16_t)((cal_data[5] << 8) | cal_data[4]);

    /* Parse pressure calibration data */
    device->dig_P1 = (uint16_t)((cal_data[7] << 8) | cal_data[6]);
    device->dig_P2 = (int16_t)((cal_data[9] << 8) | cal_data[8]);
    device->dig_P3 = (int16_t)((cal_data[11] << 8) | cal_data[10]);
    device->dig_P4 = (int16_t)((cal_data[13] << 8) | cal_data[12]);
    device->dig_P5 = (int16_t)((cal_data[15] << 8) | cal_data[14]);
    device->dig_P6 = (int16_t)((cal_data[17] << 8) | cal_data[16]);
    device->dig_P7 = (int16_t)((cal_data[19] << 8) | cal_data[18]);
    device->dig_P8 = (int16_t)((cal_data[21] << 8) | cal_data[20]);
    device->dig_P9 = (int16_t)((cal_data[23] << 8) | cal_data[22]);

    /* Parse humidity calibration data */
    device->dig_H1 = hum_cal_data[0];
    device->dig_H2 = (int16_t)((hum_cal_data[2] << 8) | hum_cal_data[1]);
    device->dig_H3 = (int16_t)hum_cal_data[3];
    device->dig_H4 = (int16_t)((hum_cal_data[4] << 4) | (hum_cal_data[5] & 0x0F));
    device->dig_H5 = (int16_t)((hum_cal_data[5] >> 4) | (hum_cal_data[6] << 4));
    device->dig_H6 = (int16_t)hum_cal_data[6];

    return true;
}

bool EasyTVC_Bme280Read(EasyTVCBme280 *device)
{
    uint8_t temp_data[3];
    uint8_t press_data[3];
    uint8_t hum_data[2];

    /* Read temperature data */
    if (EasyTVC_BusRead(device->bus, EASYTVC_BME280_TEMP_MSB_REG, temp_data, 3) != 0) {
        return false;
    }

    /* Read pressure data */
    if (EasyTVC_BusRead(device->bus, EASYTVC_BME280_PRESS_MSB_REG, press_data, 3) != 0) {
        return false;
    }

    /* Read humidity data */
    if (EasyTVC_BusRead(device->bus, EASYTVC_BME280_HUM_MSB_REG, hum_data, 2) != 0) {
        return false;
    }

    /* Parse raw values */
    int32_t adc_T = (int32_t)((temp_data[0] << 12) | (temp_data[1] << 4) | (temp_data[2] >> 4));
    int32_t adc_P = (int32_t)((press_data[0] << 12) | (press_data[1] << 4) | (press_data[2] >> 4));
    int32_t adc_H = (int32_t)((hum_data[0] << 8) | hum_data[1]);

    /* Temperature compensation (simplified BME280 formula) */
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)device->dig_T1 << 1))) * ((int32_t)device->dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)device->dig_T1)) * ((adc_T >> 4) - ((int32_t)device->dig_T1))) >> 12) * ((int32_t)device->dig_T3)) >> 14;
    int32_t t_fine = var1 + var2;
    device->temperature = (t_fine * 5 + 128) >> 8;  /* Temperature in 0.01°C */

    /* Pressure compensation (simplified) */
    var1 = (((int32_t)t_fine) >> 1) - 64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)device->dig_P6);
    var2 = var2 + ((var1 * ((int32_t)device->dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)device->dig_P4) << 16);
    var1 = (((device->dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)device->dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)device->dig_P1)) >> 15);
    if (var1 == 0) {
        return false;  /* Avoid division by zero */
    }
    int32_t p = 1048576 - adc_P;
    p = (((p << 31) / var1) << 1);
    var1 = (((int32_t)device->dig_P9) * ((p >> 13) * (p >> 13))) >> 25;
    var2 = (((int32_t)device->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int32_t)device->dig_P7) << 4);
    device->pressure = p;  /* Pressure in Pa */

    /* Humidity compensation (simplified) */
    int32_t v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)device->dig_H4) << 20) - (((int32_t)device->dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)device->dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)device->dig_H3)) >> 11) + 32768)) >> 10) + 2097152) * ((int32_t)device->dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)device->dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    device->humidity = (v_x1_u32r >> 12);  /* Humidity in 0.001% */

    return true;
}
