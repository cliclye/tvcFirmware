#include "data_logger.h"
#include "spi_driver.h"
#include "stm32f4xx.h"
#include "board_pins.h"

static DataLogger *current_logger = NULL;

/* Flash chip commands for W25Q128JVS */
#define FLASH_CMD_WRITE_ENABLE  0x06
#define FLASH_CMD_WRITE_DISABLE 0x04
#define FLASH_CMD_READ_STATUS  0x05
#define FLASH_CMD_READ_DATA    0x03
#define FLASH_CMD_PAGE_PROGRAM 0x02
#define FLASH_CMD_SECTOR_ERASE 0x20
#define FLASH_CMD_CHIP_ERASE  0xC7
#define FLASH_CMD_READ_ID     0x9F
#define FLASH_CMD_WAKE        0xAB

/* Flash chip ID */
#define FLASH_W25Q128JVS_ID   0xEF4018

/* Simple delay */
static void delay_us(uint32_t us) {
    volatile uint32_t counter;
    for (counter = 0; counter < (us * 4); counter++) {
        __asm volatile ("nop");
    }
}

/* SPI2 single-byte transfer */
static uint8_t spi2_transfer_byte(uint8_t data) {
    STM32_SPI_TypeDef *spi = STM32_SPI2;
    uint8_t rx_data;
    
    while (!(spi->SR & STM32_SPI_SR_TXE));
    spi->DR = data;
    
    while (!(spi->SR & STM32_SPI_SR_RXNE));
    rx_data = (uint8_t)(spi->DR);
    
    while (spi->SR & STM32_SPI_SR_BSY);
    
    return rx_data;
}

/* Select flash chip */
static void flash_select(void) {
    EasyTVC_SPI2_CS_Flash_Set(false);
}

/* Deselect flash chip */
static void flash_deselect(void) {
    EasyTVC_SPI2_CS_Flash_Set(true);
}

/* Send flash command */
static void flash_send_command(uint8_t cmd) {
    flash_select();
    spi2_transfer_byte(cmd);
    flash_deselect();
}

/* Send flash command with address */
static void flash_send_command_addr(uint8_t cmd, uint32_t addr) {
    flash_select();
    spi2_transfer_byte(cmd);
    spi2_transfer_byte((addr >> 16) & 0xFF);
    spi2_transfer_byte((addr >> 8) & 0xFF);
    spi2_transfer_byte(addr & 0xFF);
    flash_deselect();
}

/* Enable write */
static void flash_write_enable(void) {
    flash_send_command(FLASH_CMD_WRITE_ENABLE);
}

/* Wait for write complete */
static bool flash_wait_ready(void) {
    uint8_t status;
    uint32_t timeout = 10000;
    
    flash_select();
    spi2_transfer_byte(FLASH_CMD_READ_STATUS);
    
    do {
        status = spi2_transfer_byte(0xFF);
        if ((status & 0x01) == 0) {
            flash_deselect();
            return true;
        }
        timeout--;
    } while (timeout > 0);
    
    flash_deselect();
    return false;
}

/* Erase sector (4KB) */
static bool flash_erase_sector(uint32_t address) {
    flash_write_enable();
    flash_send_command_addr(FLASH_CMD_SECTOR_ERASE, address);
    
    return flash_wait_ready();
}

/* Write page (256 bytes) */
static bool flash_write_page(uint32_t address, const uint8_t *data, uint16_t length) {
    flash_write_enable();
    
    flash_select();
    spi2_transfer_byte(FLASH_CMD_PAGE_PROGRAM);
    spi2_transfer_byte((address >> 16) & 0xFF);
    spi2_transfer_byte((address >> 8) & 0xFF);
    spi2_transfer_byte(address & 0xFF);
    
    for (uint16_t i = 0; i < length; i++) {
        spi2_transfer_byte(data[i]);
    }
    
    flash_deselect();
    
    return flash_wait_ready();
}

bool DataLogger_Init(DataLogger *logger) {
    current_logger = logger;
    logger->initialized = false;
    logger->record_count = 0;
    logger->current_address = 0x100000;  /* Start after 1MB (skip bootloader) */
    logger->sector_count = 0;
    
    /* Wake flash from sleep */
    flash_send_command(FLASH_CMD_WAKE);
    delay_us(100);
    
    /* Read flash ID */
    flash_select();
    spi2_transfer_byte(FLASH_CMD_READ_ID);
    uint8_t id1 = spi2_transfer_byte(0xFF);
    uint8_t id2 = spi2_transfer_byte(0xFF);
    uint8_t id3 = spi2_transfer_byte(0xFF);
    flash_deselect();
    
    uint32_t flash_id = (id1 << 16) | (id2 << 8) | id3;
    
    if (flash_id != FLASH_W25Q128JVS_ID) {
        return false;  /* Wrong flash chip */
    }
    
    /* Erase first sector */
    if (!flash_erase_sector(logger->current_address)) {
        return false;
    }
    
    logger->sector_count = 1;
    logger->initialized = true;
    
    return true;
}

bool DataLogger_Log(DataLogger *logger, const TelemetryRecord *record) {
    if (!logger->initialized) {
        return false;
    }
    
    /* Write record to flash */
    uint8_t *data = (uint8_t *)record;
    uint16_t length = sizeof(TelemetryRecord);
    
    /* Check if we're at sector boundary (4KB) */
    if ((logger->current_address & 0xFFF) == 0) {
        /* Need to erase new sector */
        if (!flash_erase_sector(logger->current_address)) {
            return false;
        }
        logger->sector_count++;
    }
    
    /* Write page (256 bytes at a time) */
    for (uint16_t i = 0; i < length; i += 256) {
        uint16_t chunk_size = (length - i < 256) ? (length - i) : 256;
        if (!flash_write_page(logger->current_address + i, data + i, chunk_size)) {
            return false;
        }
    }
    
    logger->current_address += length;
    logger->record_count++;
    
    return true;
}

uint32_t DataLogger_GetRecordCount(DataLogger *logger) {
    return logger->record_count;
}

uint32_t DataLogger_GetCurrentAddress(DataLogger *logger) {
    return logger->current_address;
}
