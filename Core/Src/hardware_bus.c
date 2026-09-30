#include "hardware_bus.h"
#include "spi_driver.h"
#include "i2c_driver.h"
#include "board_pins.h"

/* Static bus instances */
static EasyTVCBus bmi088_accel_bus;
static EasyTVCBus bmi088_gyro_bus;
static EasyTVCBus bme280_bus;

/* SPI write function for BMI088 */
static int bmi088_spi_write(EasyTVCBus *bus, uint8_t reg, const uint8_t *data, size_t length)
{
    (void)bus;  /* Context not needed */
    
    uint8_t tx_buffer[256];
    uint8_t rx_buffer[256];
    
    if (length > 255) {
        return -1;
    }
    
    /* BMI088 SPI write: first byte is register with write bit set */
    tx_buffer[0] = reg | 0x80;  /* Write bit */
    for (size_t i = 0; i < length; i++) {
        tx_buffer[i + 1] = data[i];
    }
    
    if (bus == &bmi088_accel_bus) {
        EasyTVC_SPI1_CS_Accel_Set(false);
        EasyTVC_SPI1_Transfer(tx_buffer, rx_buffer, length + 1);
        EasyTVC_SPI1_CS_Accel_Set(true);
    } else if (bus == &bmi088_gyro_bus) {
        EasyTVC_SPI1_CS_Gyro_Set(false);
        EasyTVC_SPI1_Transfer(tx_buffer, rx_buffer, length + 1);
        EasyTVC_SPI1_CS_Gyro_Set(true);
    } else {
        return -1;
    }
    
    return 0;
}

/* SPI read function for BMI088 */
static int bmi088_spi_read(EasyTVCBus *bus, uint8_t reg, uint8_t *data, size_t length)
{
    (void)bus;
    
    uint8_t tx_buffer[256];
    uint8_t rx_buffer[256];
    
    if (length > 255) {
        return -1;
    }
    
    /* BMI088 SPI read: first byte is register with write bit clear */
    tx_buffer[0] = reg & 0x7F;  /* Read bit */
    for (size_t i = 1; i <= length; i++) {
        tx_buffer[i] = 0xFF;  /* Dummy bytes */
    }
    
    if (bus == &bmi088_accel_bus) {
        EasyTVC_SPI1_CS_Accel_Set(false);
        EasyTVC_SPI1_Transfer(tx_buffer, rx_buffer, length + 1);
        EasyTVC_SPI1_CS_Accel_Set(true);
    } else if (bus == &bmi088_gyro_bus) {
        EasyTVC_SPI1_CS_Gyro_Set(false);
        EasyTVC_SPI1_Transfer(tx_buffer, rx_buffer, length + 1);
        EasyTVC_SPI1_CS_Gyro_Set(true);
    } else {
        return -1;
    }
    
    /* Copy received data (skip first byte which is dummy) */
    for (size_t i = 0; i < length; i++) {
        data[i] = rx_buffer[i + 1];
    }
    
    return 0;
}

/* I2C write function for BME280 */
static int bme280_i2c_write(EasyTVCBus *bus, uint8_t reg, const uint8_t *data, size_t length)
{
    (void)bus;
    /* Try both I2C addresses from hwdef */
    int result = EasyTVC_I2C1_Write(0x76, reg, data, length);
    if (result != 0) {
        result = EasyTVC_I2C1_Write(0x77, reg, data, length);
    }
    return result;
}

/* I2C read function for BME280 */
static int bme280_i2c_read(EasyTVCBus *bus, uint8_t reg, uint8_t *data, size_t length)
{
    (void)bus;
    /* Try both I2C addresses from hwdef */
    int result = EasyTVC_I2C1_Read(0x76, reg, data, length);
    if (result != 0) {
        result = EasyTVC_I2C1_Read(0x77, reg, data, length);
    }
    return result;
}

void EasyTVC_HardwareBus_Init(void)
{
    /* Initialize BMI088 accel bus */
    bmi088_accel_bus.context = NULL;
    bmi088_accel_bus.write = bmi088_spi_write;
    bmi088_accel_bus.read = bmi088_spi_read;
    
    /* Initialize BMI088 gyro bus */
    bmi088_gyro_bus.context = NULL;
    bmi088_gyro_bus.write = bmi088_spi_write;
    bmi088_gyro_bus.read = bmi088_spi_read;
    
    /* Initialize BME280 bus */
    bme280_bus.context = NULL;
    bme280_bus.write = bme280_i2c_write;
    bme280_bus.read = bme280_i2c_read;
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
