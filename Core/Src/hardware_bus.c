#include "hardware_bus.h"
#include "spi_driver.h"
#include "i2c_driver.h"
#include "stm32f4xx.h"
#include "board_pins.h"

/* BMI088 SPI framing (datasheet section 6):
 *   bit 7 of the first byte is R/W: 1 = READ, 0 = WRITE (the opposite of what many chips do).
 *   The accelerometer inserts one dummy byte after the address on reads; the gyro does not. */
#define BMI088_SPI_READ  0x80U
#define BMI088_MAX_XFER  32U

typedef struct {
    void (*chip_select)(bool inactive);
    uint8_t read_dummy_bytes;
} Bmi088SpiPort;

static const Bmi088SpiPort accel_port = {EasyTVC_SPI1_CS_Accel_Set, 1U};
static const Bmi088SpiPort gyro_port = {EasyTVC_SPI1_CS_Gyro_Set, 0U};

static EasyTVCBus bmi088_accel_bus;
static EasyTVCBus bmi088_gyro_bus;
static EasyTVCBus bme280_bus;

static int bmi088_spi_write(EasyTVCBus *bus, uint8_t reg, const uint8_t *data, size_t length)
{
    const Bmi088SpiPort *port = bus->context;
    uint8_t tx[BMI088_MAX_XFER + 1U];
    uint8_t rx[BMI088_MAX_XFER + 1U];

    if (port == NULL || length > BMI088_MAX_XFER) {
        return -1;
    }
    tx[0] = reg & 0x7FU;
    for (size_t i = 0; i < length; i++) {
        tx[i + 1U] = data[i];
    }

    port->chip_select(false);
    const int status = EasyTVC_SPI1_Transfer(tx, rx, length + 1U);
    port->chip_select(true);
    /* Datasheet: >= 2 us between writes in normal mode. */
    STM32_DelayMicroseconds(3U);
    return status;
}

static int bmi088_spi_read(EasyTVCBus *bus, uint8_t reg, uint8_t *data, size_t length)
{
    const Bmi088SpiPort *port = bus->context;
    uint8_t tx[BMI088_MAX_XFER + 2U];
    uint8_t rx[BMI088_MAX_XFER + 2U];

    if (port == NULL || length > BMI088_MAX_XFER) {
        return -1;
    }
    const size_t lead = 1U + port->read_dummy_bytes;   /* address (+ accel dummy) */
    tx[0] = reg | BMI088_SPI_READ;
    for (size_t i = 1; i < lead + length; i++) {
        tx[i] = 0xFFU;
    }

    port->chip_select(false);
    const int status = EasyTVC_SPI1_Transfer(tx, rx, lead + length);
    port->chip_select(true);
    if (status != 0) {
        return status;
    }

    for (size_t i = 0; i < length; i++) {
        data[i] = rx[lead + i];
    }
    return 0;
}

/* The barometer may sit on either I2C bus at either address (hwdef probes I2C1 0x76/0x77 and
 * I2C2 0x77). Search once, then stay on the address that answered. */
typedef struct {
    uint8_t bus;    /* 1 = I2C1, 2 = I2C2 */
    uint8_t addr;
} BaroLocation;

static const BaroLocation baro_candidates[] = {
    {1U, EASYTVC_BME280_I2C_ADDR_SECONDARY},   /* 0x77: the usual strap */
    {1U, EASYTVC_BME280_I2C_ADDR_PRIMARY},     /* 0x76 */
    {2U, EASYTVC_BME280_I2C_ADDR_SECONDARY},
    {2U, EASYTVC_BME280_I2C_ADDR_PRIMARY},
};
static BaroLocation baro_found;
static bool baro_located;

static int baro_read_at(BaroLocation where, uint8_t reg, uint8_t *data, size_t length)
{
    return where.bus == 1U ? EasyTVC_I2C1_Read(where.addr, reg, data, length)
                           : EasyTVC_I2C2_Read(where.addr, reg, data, length);
}

static int baro_write_at(BaroLocation where, uint8_t reg, const uint8_t *data, size_t length)
{
    return where.bus == 1U ? EasyTVC_I2C1_Write(where.addr, reg, data, length)
                           : EasyTVC_I2C2_Write(where.addr, reg, data, length);
}

static int bme280_i2c_read(EasyTVCBus *bus, uint8_t reg, uint8_t *data, size_t length)
{
    (void)bus;
    if (baro_located) {
        return baro_read_at(baro_found, reg, data, length);
    }
    /* Not found yet: only a chip-id read may select the device, and only if it IS a BME280. */
    for (size_t i = 0; i < sizeof(baro_candidates) / sizeof(baro_candidates[0]); i++) {
        uint8_t id = 0U;
        if (baro_read_at(baro_candidates[i], EASYTVC_BME280_CHIP_ID_REG, &id, 1U) == 0 &&
            id == EASYTVC_BME280_CHIP_ID) {
            baro_found = baro_candidates[i];
            baro_located = true;
            return baro_read_at(baro_found, reg, data, length);
        }
    }
    return -1;
}

static int bme280_i2c_write(EasyTVCBus *bus, uint8_t reg, const uint8_t *data, size_t length)
{
    (void)bus;
    if (!baro_located) {
        uint8_t id = 0U;
        if (bme280_i2c_read(bus, EASYTVC_BME280_CHIP_ID_REG, &id, 1U) != 0) {
            return -1;
        }
    }
    return baro_write_at(baro_found, reg, data, length);
}

void EasyTVC_HardwareBus_Init(void)
{
    bmi088_accel_bus.context = (void *)&accel_port;
    bmi088_accel_bus.write = bmi088_spi_write;
    bmi088_accel_bus.read = bmi088_spi_read;

    bmi088_gyro_bus.context = (void *)&gyro_port;
    bmi088_gyro_bus.write = bmi088_spi_write;
    bmi088_gyro_bus.read = bmi088_spi_read;

    bme280_bus.context = NULL;
    bme280_bus.write = bme280_i2c_write;
    bme280_bus.read = bme280_i2c_read;
    baro_located = false;
}

EasyTVCBus* EasyTVC_GetBMI088_AccelBus(void)
{
    return &bmi088_accel_bus;
}

EasyTVCBus* EasyTVC_GetBMI088_GyroBus(void)
{
    return &bmi088_gyro_bus;
}

EasyTVCBus* EasyTVC_GetBME280Bus(void)
{
    return &bme280_bus;
}
