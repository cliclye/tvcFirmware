#include "bmi088.h"

void EasyTVC_Bmi088Init(EasyTVCBmi088 *device, EasyTVCBus *accel, EasyTVCBus *gyro)
{
    device->accel = accel;
    device->gyro = gyro;
    device->identity_confirmed = false;
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
