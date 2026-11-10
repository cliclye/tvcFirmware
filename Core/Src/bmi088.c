#include "bmi088.h"

static void pause_ms(const EasyTVCBmi088 *device, uint32_t ms)
{
    if (device->delay_ms != 0) {
        device->delay_ms(ms);
    }
}

/* Write a register up to `tries` times until it reads back as wanted. Never fatal on its own: the
 * caller judges the sensor by its data (Bosch's driver does not read settings back at all). */
static void set_register(EasyTVCBmi088 *device, EasyTVCBus *bus, uint8_t reg, uint8_t value,
                         uint8_t mask, uint32_t settle_ms, unsigned tries)
{
    uint8_t readback = 0U;
    if (EasyTVC_BusReadU8(bus, reg, &readback) == 0 && (readback & mask) == (value & mask)) {
        return;   /* already set (warm reboot) */
    }
    for (unsigned attempt = 0; attempt < tries; ++attempt) {
        (void)EasyTVC_BusWrite(bus, reg, &value, 1U);
        pause_ms(device, settle_ms);
        if (EasyTVC_BusReadU8(bus, reg, &readback) == 0 && (readback & mask) == (value & mask)) {
            return;
        }
    }
}

static uint8_t read_u8(EasyTVCBus *bus, uint8_t reg)
{
    uint8_t value = 0xEEU;   /* "could not read" marker in the diagnostics */
    (void)EasyTVC_BusReadU8(bus, reg, &value);
    return value;
}

static bool accel_awake(EasyTVCBmi088 *device)
{
    /* An asleep accelerometer reads exactly zero on every axis; nothing resting on Earth does. */
    for (unsigned sample = 0; sample < 3U; ++sample) {
        uint8_t raw[6] = {0};
        if (EasyTVC_BusRead(device->accel, EASYTVC_BMI088_ACCEL_X_LSB_REG, raw, sizeof(raw)) == 0 &&
            (raw[0] | raw[1] | raw[2] | raw[3] | raw[4] | raw[5]) != 0U) {
            return true;
        }
        pause_ms(device, 3U);
    }
    return false;
}

void EasyTVC_Bmi088Init(EasyTVCBmi088 *device, EasyTVCBus *accel, EasyTVCBus *gyro)
{
    device->accel = accel;
    device->gyro = gyro;
    device->delay_ms = 0;
    device->identity_confirmed = false;
    device->accel_ok = false;
    device->gyro_ok = false;
    device->accel_m_s2_per_lsb = EASYTVC_BMI088_ACCEL_M_S2_PER_LSB;
    device->gyro_rad_s_per_lsb = EASYTVC_BMI088_GYRO_RAD_S_PER_LSB;
    device->accel_passes = 0U;
    device->config_fail_step = 0U;
    for (unsigned i = 0; i < sizeof(device->readback); ++i) {
        device->readback[i] = 0U;
    }
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
    uint8_t dummy = 0U;
    device->config_fail_step = 0U;
    device->accel_passes = 0U;

    /* ---- accelerometer: up to 5 passes of Bosch's wake-up sequence until data comes out ----
     * PWR_CONF active, 5 ms, PWR_CTRL on, 5 ms, then rate and range, 50 ms settle. No soft reset
     * (as ArduPilot): it drops the part back into I2C mode and suspend. */
    device->accel_ok = false;
    for (uint8_t pass = 1; pass <= 5U && !device->accel_ok; ++pass) {
        (void)EasyTVC_BusReadU8(device->accel, EASYTVC_BMI088_ACCEL_CHIP_ID_REG, &dummy);   /* CSB edge: SPI mode */
        pause_ms(device, 1U);
        set_register(device, device->accel, EASYTVC_BMI088_ACCEL_PWR_CONF_REG, 0x00U, 0xFFU, 5U, 4U);
        set_register(device, device->accel, EASYTVC_BMI088_ACCEL_PWR_CTRL_REG, EASYTVC_BMI088_ACCEL_ENABLE, 0xFFU, 5U, 4U);
        set_register(device, device->accel, EASYTVC_BMI088_ACCEL_CONF_REG, EASYTVC_BMI088_ACCEL_CONF_400HZ_NORMAL, 0xFFU, 1U, 4U);
        set_register(device, device->accel, EASYTVC_BMI088_ACCEL_RANGE_REG, EASYTVC_BMI088_ACCEL_RANGE_24G, 0x03U, 1U, 4U);
        pause_ms(device, 50U);
        if (accel_awake(device)) {
            device->accel_ok = true;
            device->accel_passes = pass;
        }
    }
    device->readback[0] = read_u8(device->accel, EASYTVC_BMI088_ACCEL_PWR_CONF_REG);
    device->readback[1] = read_u8(device->accel, EASYTVC_BMI088_ACCEL_PWR_CTRL_REG);
    device->readback[2] = read_u8(device->accel, EASYTVC_BMI088_ACCEL_CONF_REG);
    device->readback[3] = read_u8(device->accel, EASYTVC_BMI088_ACCEL_RANGE_REG);
    /* Scale from the range the chip reports (0..3 = +-3/6/12/24 g), whatever we asked for. */
    if (device->readback[3] != 0xEEU) {
        device->accel_m_s2_per_lsb = (float)(3U << (device->readback[3] & 0x03U)) * 9.80665f / 32768.0f;
    }

    /* ---- gyroscope, on its own: it powers up in normal mode, so it works even if the
     * accelerometer does not. Soft reset (unchecked: it tends to NAK, per ArduPilot), 30 ms. ---- */
    const uint8_t reset = 0xB6U;
    (void)EasyTVC_BusWrite(device->gyro, EASYTVC_BMI088_GYRO_SOFTRESET_REG, &reset, 1U);
    pause_ms(device, 30U);
    set_register(device, device->gyro, EASYTVC_BMI088_GYRO_LPM1_REG, 0x00U, 0xFFU, 1U, 10U);
    set_register(device, device->gyro, EASYTVC_BMI088_GYRO_RANGE_REG, EASYTVC_BMI088_GYRO_RANGE_2000DPS, 0x07U, 1U, 10U);
    set_register(device, device->gyro, EASYTVC_BMI088_GYRO_BANDWIDTH_REG, EASYTVC_BMI088_GYRO_ODR400_BW47, 0x7FU, 1U, 10U);
    pause_ms(device, 30U);
    device->readback[4] = read_u8(device->gyro, EASYTVC_BMI088_GYRO_RANGE_REG);
    device->readback[5] = read_u8(device->gyro, EASYTVC_BMI088_GYRO_BANDWIDTH_REG);
    device->readback[6] = read_u8(device->gyro, EASYTVC_BMI088_GYRO_LPM1_REG);
    uint8_t gyro_id = 0U;
    const uint8_t range = device->readback[4] & 0x07U;
    /* 0..4 = +-2000/1000/500/250/125 dps */
    device->gyro_ok = EasyTVC_BusReadU8(device->gyro, EASYTVC_BMI088_GYRO_CHIP_ID_REG, &gyro_id) == 0 &&
                      gyro_id == EASYTVC_BMI088_GYRO_CHIP_ID && range <= 4U && device->readback[6] == 0x00U;
    if (range <= 4U) {
        device->gyro_rad_s_per_lsb = (2000.0f / (float)(1U << range)) * 0.017453292519943f / 32768.0f;
    }

    if (!device->accel_ok) {
        device->config_fail_step = 9U;
    } else if (!device->gyro_ok) {
        device->config_fail_step = 10U;
    }
    return device->accel_ok && device->gyro_ok;
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

    device->accel_m_s2[0] = (float)device->accel_x * device->accel_m_s2_per_lsb;
    device->accel_m_s2[1] = (float)device->accel_y * device->accel_m_s2_per_lsb;
    device->accel_m_s2[2] = (float)device->accel_z * device->accel_m_s2_per_lsb;
    device->gyro_rad_s[0] = (float)device->gyro_x * device->gyro_rad_s_per_lsb;
    device->gyro_rad_s[1] = (float)device->gyro_y * device->gyro_rad_s_per_lsb;
    device->gyro_rad_s[2] = (float)device->gyro_z * device->gyro_rad_s_per_lsb;

    device->accel_saturated = near_full_scale(device->accel_x) || near_full_scale(device->accel_y) ||
                              near_full_scale(device->accel_z);
    device->gyro_saturated = near_full_scale(device->gyro_x) || near_full_scale(device->gyro_y) ||
                             near_full_scale(device->gyro_z);
    return true;
}
