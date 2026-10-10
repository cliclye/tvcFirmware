#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "bme280.h"
#include "bmi088.h"

/* A byte-addressable register file that behaves like a sensor: writes stick, reads return the
 * stored byte, multi-byte reads auto-increment. */
typedef struct {
    uint8_t reg[256];
    bool writes_stick;
    unsigned write_count;
} FakeDevice;

static int fake_read(EasyTVCBus *bus, uint8_t reg, uint8_t *data, size_t length)
{
    const FakeDevice *dev = bus->context;
    for (size_t i = 0; i < length; ++i) {
        data[i] = dev->reg[(reg + i) & 0xFFU];
    }
    return 0;
}

static int fake_write(EasyTVCBus *bus, uint8_t reg, const uint8_t *data, size_t length)
{
    FakeDevice *dev = bus->context;
    for (size_t i = 0; i < length; ++i) {
        ++dev->write_count;
        if (dev->writes_stick) {
            dev->reg[(reg + i) & 0xFFU] = data[i];
        }
    }
    return 0;
}

static void put_le16(uint8_t *p, int v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

/* Bosch BMP280/BME280 datasheet worked example: T = 25.08 degC, P = 100653.27 Pa. */
static void load_bosch_example(FakeDevice *dev)
{
    memset(dev, 0, sizeof(*dev));
    dev->writes_stick = true;
    dev->reg[EASYTVC_BME280_CHIP_ID_REG] = EASYTVC_BME280_CHIP_ID;
    uint8_t *c = &dev->reg[EASYTVC_BME280_DIG_T1_REG];
    put_le16(c + 0, 27504);  put_le16(c + 2, 26435);  put_le16(c + 4, -1000);
    put_le16(c + 6, 36477);  put_le16(c + 8, -10685); put_le16(c + 10, 3024);
    put_le16(c + 12, 2855);  put_le16(c + 14, 140);   put_le16(c + 16, -7);
    put_le16(c + 18, 15500); put_le16(c + 20, -14600); put_le16(c + 22, 6000);
    /* adc_P = 415148 (0x65A2C), adc_T = 519888 (0x7EED0), humidity skipped (0x8000). */
    uint8_t *raw = &dev->reg[EASYTVC_BME280_PRESS_MSB_REG];
    raw[0] = 0x65; raw[1] = 0x5A; raw[2] = 0xC0;
    raw[3] = 0x7E; raw[4] = 0xED; raw[5] = 0x00;
    raw[6] = 0x80; raw[7] = 0x00;
}

static void test_bme280_matches_bosch_example(void)
{
    FakeDevice dev;
    load_bosch_example(&dev);
    EasyTVCBus bus = {.context = &dev, .write = fake_write, .read = fake_read};
    EasyTVCBme280 baro;
    EasyTVC_Bme280Init(&baro, &bus);

    assert(EasyTVC_Bme280Probe(&baro));
    assert(EasyTVC_Bme280Configure(&baro));
    assert(dev.reg[EASYTVC_BME280_CTRL_MEAS_REG] == 0x2F);
    assert(dev.reg[EASYTVC_BME280_CTRL_HUM_REG] == 0x01);   /* written before CTRL_MEAS */

    assert(EasyTVC_Bme280Read(&baro));
    printf("bme280: T=%d (0.01C) P=%u Pa\n", (int)baro.temperature, (unsigned)baro.pressure);
    assert(baro.temperature >= 2507 && baro.temperature <= 2509);
    assert(baro.pressure >= 100650U && baro.pressure <= 100656U);
    assert(baro.humidity == 0U);
}

static void test_bme280_rejects_uncalibrated_and_skipped(void)
{
    FakeDevice dev;
    load_bosch_example(&dev);
    EasyTVCBus bus = {.context = &dev, .write = fake_write, .read = fake_read};
    EasyTVCBme280 baro;
    EasyTVC_Bme280Init(&baro, &bus);

    /* No calibration loaded yet: dig_P1 == 0 must not divide by zero or return a number. */
    memset(&baro.dig_T1, 0, sizeof(baro) - offsetof(EasyTVCBme280, dig_T1));
    assert(!EasyTVC_Bme280Read(&baro));

    assert(EasyTVC_Bme280Configure(&baro));
    dev.reg[EASYTVC_BME280_PRESS_MSB_REG] = 0x80;   /* adc_P = 0x80000: measurement skipped */
    dev.reg[EASYTVC_BME280_PRESS_MSB_REG + 1] = 0x00;
    dev.reg[EASYTVC_BME280_PRESS_MSB_REG + 2] = 0x00;
    assert(!EasyTVC_Bme280Read(&baro));
}

static void test_bme280_config_must_stick(void)
{
    FakeDevice dev;
    load_bosch_example(&dev);
    dev.writes_stick = false;
    EasyTVCBus bus = {.context = &dev, .write = fake_write, .read = fake_read};
    EasyTVCBme280 baro;
    EasyTVC_Bme280Init(&baro, &bus);
    assert(!EasyTVC_Bme280Configure(&baro));
}

static void test_bmi088_wakes_accelerometer_and_scales(void)
{
    FakeDevice accel_dev, gyro_dev;
    memset(&accel_dev, 0, sizeof(accel_dev));
    memset(&gyro_dev, 0, sizeof(gyro_dev));
    accel_dev.writes_stick = gyro_dev.writes_stick = true;
    accel_dev.reg[EASYTVC_BMI088_ACCEL_CHIP_ID_REG] = EASYTVC_BMI088_ACCEL_CHIP_ID;
    gyro_dev.reg[EASYTVC_BMI088_GYRO_CHIP_ID_REG] = EASYTVC_BMI088_GYRO_CHIP_ID;
    gyro_dev.reg[EASYTVC_BMI088_GYRO_BANDWIDTH_REG] = 0x80;   /* bit 7 reads back as 1 */

    EasyTVCBus accel = {.context = &accel_dev, .write = fake_write, .read = fake_read};
    EasyTVCBus gyro = {.context = &gyro_dev, .write = fake_write, .read = fake_read};
    EasyTVCBmi088 imu;
    EasyTVC_Bmi088Init(&imu, &accel, &gyro);

    assert(EasyTVC_Bmi088Probe(&imu));
    assert(EasyTVC_Bmi088Configure(&imu));
    /* The old code wrote 0x00 to ACC_PWR_CTRL, which switches the accelerometer OFF. */
    assert(accel_dev.reg[EASYTVC_BMI088_ACCEL_PWR_CTRL_REG] == 0x04);
    assert(accel_dev.reg[EASYTVC_BMI088_ACCEL_RANGE_REG] == 0x03);
    assert(accel_dev.reg[EASYTVC_BMI088_ACCEL_CONF_REG] == 0xAA);
    assert(gyro_dev.reg[EASYTVC_BMI088_GYRO_RANGE_REG] == 0x00);
    assert((gyro_dev.reg[EASYTVC_BMI088_GYRO_BANDWIDTH_REG] & 0x7F) == 0x03);

    /* 1 g on Z at +-24 g is 32768/24 = 1365.33 counts. */
    put_le16(&accel_dev.reg[EASYTVC_BMI088_ACCEL_X_LSB_REG], -100);
    put_le16(&accel_dev.reg[EASYTVC_BMI088_ACCEL_X_LSB_REG + 2], 200);
    put_le16(&accel_dev.reg[EASYTVC_BMI088_ACCEL_X_LSB_REG + 4], 1365);
    /* 1000 counts at +-2000 dps = 61.04 dps = 1.0653 rad/s. */
    put_le16(&gyro_dev.reg[EASYTVC_BMI088_GYRO_RATE_X_LSB_REG], 1000);
    put_le16(&gyro_dev.reg[EASYTVC_BMI088_GYRO_RATE_X_LSB_REG + 2], -1000);
    put_le16(&gyro_dev.reg[EASYTVC_BMI088_GYRO_RATE_X_LSB_REG + 4], 0);

    assert(EasyTVC_Bmi088Read(&imu));
    assert(imu.accel_x == -100 && imu.accel_y == 200 && imu.accel_z == 1365);
    assert(fabsf(imu.accel_m_s2[2] - 9.80665f) < 0.01f);
    assert(fabsf(imu.gyro_rad_s[0] - 1.0653f) < 0.001f);
    assert(fabsf(imu.gyro_rad_s[1] + 1.0653f) < 0.001f);
    assert(!imu.accel_saturated && !imu.gyro_saturated);

    put_le16(&accel_dev.reg[EASYTVC_BMI088_ACCEL_X_LSB_REG + 4], 32767);
    assert(EasyTVC_Bmi088Read(&imu));
    assert(imu.accel_saturated);
}

static void test_bmi088_configure_fails_if_registers_do_not_stick(void)
{
    FakeDevice accel_dev, gyro_dev;
    memset(&accel_dev, 0, sizeof(accel_dev));
    memset(&gyro_dev, 0, sizeof(gyro_dev));
    EasyTVCBus accel = {.context = &accel_dev, .write = fake_write, .read = fake_read};
    EasyTVCBus gyro = {.context = &gyro_dev, .write = fake_write, .read = fake_read};
    EasyTVCBmi088 imu;
    EasyTVC_Bmi088Init(&imu, &accel, &gyro);
    assert(!EasyTVC_Bmi088Configure(&imu));
}

int main(void)
{
    test_bme280_matches_bosch_example();
    test_bme280_rejects_uncalibrated_and_skipped();
    test_bme280_config_must_stick();
    test_bmi088_wakes_accelerometer_and_scales();
    test_bmi088_configure_fails_if_registers_do_not_stick();
    puts("driver tests passed");
    return 0;
}
