#include "bmi088.h"

void EasyTVC_Bmi088Init(EasyTVCBmi088 *device, EasyTVCBus *accel, EasyTVCBus *gyro)
{
    device->accel = accel;
    device->gyro = gyro;
    device->identity_confirmed = false;
    device->accel_x = 0;
    device->accel_y = 0;
    device->accel_z = 0;
    device->gyro_x = 0;
    device->gyro_y = 0;
    device->gyro_z = 0;
}

bool EasyTVC_Bmi088Probe(EasyTVCBmi088 *device)
{
    uint8_t accel_id = 0U;
    uint8_t gyro_id = 0U;

    if (EasyTVC_BusReadU8(device->accel, EASYTVC_BMI088_ACCEL_CHIP_ID_REG, &accel_id) != 0) {
        device->identity_confirmed = false;
        return false;
    }
    if (EasyTVC_BusReadU8(device->gyro, EASYTVC_BMI088_GYRO_CHIP_ID_REG, &gyro_id) != 0) {
        device->identity_confirmed = false;
        return false;
    }

    device->identity_confirmed =
        (accel_id == EASYTVC_BMI088_ACCEL_CHIP_ID) &&
        (gyro_id == EASYTVC_BMI088_GYRO_CHIP_ID);
    return device->identity_confirmed;
}

bool EasyTVC_Bmi088Configure(EasyTVCBmi088 *device)
{
    uint8_t accel_config = 0x00;  /* Normal mode, 100 Hz */
    uint8_t gyro_config = 0x00;   /* Normal mode, 200 Hz */

    if (EasyTVC_BusWrite(device->accel, EASYTVC_BMI088_ACCEL_CTRL_REG, &accel_config, 1) != 0) {
        return false;
    }
    if (EasyTVC_BusWrite(device->gyro, EASYTVC_BMI088_GYRO_CTRL_REG, &gyro_config, 1) != 0) {
        return false;
    }

    return true;
}

bool EasyTVC_Bmi088Read(EasyTVCBmi088 *device)
{
    uint8_t accel_data[6];
    uint8_t gyro_data[6];

    /* Read accelerometer data (X, Y, Z) */
    if (EasyTVC_BusRead(device->accel, EASYTVC_BMI088_ACCEL_X_LSB_REG, accel_data, 6) != 0) {
        return false;
    }

    /* Read gyroscope data (X, Y, Z) */
    if (EasyTVC_BusRead(device->gyro, EASYTVC_BMI088_GYRO_RATE_X_LSB_REG, gyro_data, 6) != 0) {
        return false;
    }

    /* Parse accelerometer data (little-endian) */
    device->accel_x = (int16_t)((accel_data[1] << 8) | accel_data[0]);
    device->accel_y = (int16_t)((accel_data[3] << 8) | accel_data[2]);
    device->accel_z = (int16_t)((accel_data[5] << 8) | accel_data[4]);

    /* Parse gyroscope data (little-endian) */
    device->gyro_x = (int16_t)((gyro_data[1] << 8) | gyro_data[0]);
    device->gyro_y = (int16_t)((gyro_data[3] << 8) | gyro_data[2]);
    device->gyro_z = (int16_t)((gyro_data[5] << 8) | gyro_data[4]);

    return true;
}
