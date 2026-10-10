#include "bmi088.h"

static void pause_ms(const EasyTVCBmi088 *device, uint32_t ms)
{
    if (device->delay_ms != 0) {
        device->delay_ms(ms);
    }
}

/* `mask` ignores reserved/read-only bits (e.g. GYRO_BANDWIDTH bit 7 always reads 1). */
static int write_verified(EasyTVCBus *bus, uint8_t reg, uint8_t value, uint8_t mask)
{
    uint8_t readback = 0U;
    if (EasyTVC_BusWrite(bus, reg, &value, 1U) != 0) {
        return -1;
    }
    if (EasyTVC_BusReadU8(bus, reg, &readback) != 0) {
        return -1;
    }
    return (readback & mask) == (value & mask) ? 0 : -1;
}

void EasyTVC_Bmi088Init(EasyTVCBmi088 *device, EasyTVCBus *accel, EasyTVCBus *gyro)
{
    device->accel = accel;
    device->gyro = gyro;
    device->delay_ms = 0;
    device->identity_confirmed = false;
    device->accel_saturated = false;
    device->gyro_saturated = false;
    device->accel_x = 0;
    device->accel_y = 0;
    device->accel_z = 0;
    device->gyro_x = 0;
    device->gyro_y = 0;
    device->gyro_z = 0;
    for (unsigned i = 0; i < 3; ++i) {
        device->accel_m_s2[i] = 0.0f;
        device->gyro_rad_s[i] = 0.0f;
    }
}

bool EasyTVC_Bmi088Probe(EasyTVCBmi088 *device)
{
    uint8_t accel_id = 0U;
    uint8_t gyro_id = 0U;

    /* The accelerometer powers up in I2C mode. A first read (whose result is meaningless) makes
     * the rising edge of CSB that switches it to SPI. */
    (void)EasyTVC_BusReadU8(device->accel, EASYTVC_BMI088_ACCEL_CHIP_ID_REG, &accel_id);

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
    const uint8_t reset = 0xB6U;
    uint8_t dummy = 0U;

    /* Accelerometer: soft reset, back into SPI mode, then wake it. Without ACC_PWR_CTRL = 0x04
     * the part stays in suspend and every sample reads as zero. */
    if (EasyTVC_BusWrite(device->accel, EASYTVC_BMI088_ACCEL_SOFTRESET_REG, &reset, 1U) != 0) {
        return false;
    }
    pause_ms(device, 2U);
    (void)EasyTVC_BusReadU8(device->accel, EASYTVC_BMI088_ACCEL_CHIP_ID_REG, &dummy);

    if (write_verified(device->accel, EASYTVC_BMI088_ACCEL_PWR_CONF_REG, 0x00U, 0xFFU) != 0) {
        return false;
    }
    pause_ms(device, 2U);
    if (write_verified(device->accel, EASYTVC_BMI088_ACCEL_PWR_CTRL_REG, EASYTVC_BMI088_ACCEL_ENABLE, 0xFFU) != 0) {
        return false;
    }
    pause_ms(device, 10U);
    if (write_verified(device->accel, EASYTVC_BMI088_ACCEL_CONF_REG, EASYTVC_BMI088_ACCEL_CONF_400HZ_NORMAL, 0xFFU) != 0 ||
        write_verified(device->accel, EASYTVC_BMI088_ACCEL_RANGE_REG, EASYTVC_BMI088_ACCEL_RANGE_24G, 0x03U) != 0) {
        return false;
    }

    /* Gyroscope: +-2000 dps, 400 Hz output rate / 47 Hz filter, normal power mode. */
    if (write_verified(device->gyro, EASYTVC_BMI088_GYRO_LPM1_REG, 0x00U, 0xFFU) != 0 ||
        write_verified(device->gyro, EASYTVC_BMI088_GYRO_RANGE_REG, EASYTVC_BMI088_GYRO_RANGE_2000DPS, 0x07U) != 0 ||
        write_verified(device->gyro, EASYTVC_BMI088_GYRO_BANDWIDTH_REG, EASYTVC_BMI088_GYRO_ODR400_BW47, 0x7FU) != 0) {
        return false;
    }
    pause_ms(device, 30U);
    return true;
}

static int16_t le16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static bool near_full_scale(int16_t value)
{
    return value >= 32700 || value <= -32700;
}

bool EasyTVC_Bmi088Read(EasyTVCBmi088 *device)
{
    uint8_t accel_data[6];
    uint8_t gyro_data[6];

    if (EasyTVC_BusRead(device->accel, EASYTVC_BMI088_ACCEL_X_LSB_REG, accel_data, 6) != 0) {
        return false;
    }
    if (EasyTVC_BusRead(device->gyro, EASYTVC_BMI088_GYRO_RATE_X_LSB_REG, gyro_data, 6) != 0) {
        return false;
    }

    device->accel_x = le16(&accel_data[0]);
    device->accel_y = le16(&accel_data[2]);
    device->accel_z = le16(&accel_data[4]);
    device->gyro_x = le16(&gyro_data[0]);
    device->gyro_y = le16(&gyro_data[2]);
    device->gyro_z = le16(&gyro_data[4]);

    device->accel_m_s2[0] = (float)device->accel_x * EASYTVC_BMI088_ACCEL_M_S2_PER_LSB;
    device->accel_m_s2[1] = (float)device->accel_y * EASYTVC_BMI088_ACCEL_M_S2_PER_LSB;
    device->accel_m_s2[2] = (float)device->accel_z * EASYTVC_BMI088_ACCEL_M_S2_PER_LSB;
    device->gyro_rad_s[0] = (float)device->gyro_x * EASYTVC_BMI088_GYRO_RAD_S_PER_LSB;
    device->gyro_rad_s[1] = (float)device->gyro_y * EASYTVC_BMI088_GYRO_RAD_S_PER_LSB;
    device->gyro_rad_s[2] = (float)device->gyro_z * EASYTVC_BMI088_GYRO_RAD_S_PER_LSB;

    device->accel_saturated = near_full_scale(device->accel_x) || near_full_scale(device->accel_y) ||
                              near_full_scale(device->accel_z);
    device->gyro_saturated = near_full_scale(device->gyro_x) || near_full_scale(device->gyro_y) ||
                             near_full_scale(device->gyro_z);
    return true;
}
