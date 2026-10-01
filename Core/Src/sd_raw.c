#include "sd_raw.h"
#include "spi_driver.h"
#include "stm32f4xx.h"
#include "board_pins.h"

static SD_Card *current_sd = NULL;

/* SD card commands */
#define SD_CMD0  0
#define SD_CMD8  8
#define SD_CMD17 17
#define SD_CMD24 24
#define SD_CMD55 55
#define SD_CMD58 58
#define SD_ACMD41 41

/* SD card response */
#define SD_R1_IDLE_STATE (1 << 0)

/* Simple delay */
static void delay_ms(uint32_t ms) {
    volatile uint32_t counter;
    for (counter = 0; counter < (ms * 4000); counter++) {
        __asm volatile ("nop");
    }
}

/* Send command to SD card */
static uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg, uint8_t crc) {
    uint8_t response;
    
    EasyTVC_SPI3_CS_SD_Set(false);
    
    EasyTVC_SPI3_TransferByte(0x40 | cmd);
    EasyTVC_SPI3_TransferByte((arg >> 24) & 0xFF);
    EasyTVC_SPI3_TransferByte((arg >> 16) & 0xFF);
    EasyTVC_SPI3_TransferByte((arg >> 8) & 0xFF);
    EasyTVC_SPI3_TransferByte(arg & 0xFF);
    EasyTVC_SPI3_TransferByte(crc);
    
    /* Wait for response */
    for (uint8_t i = 0; i < 8; i++) {
        response = EasyTVC_SPI3_TransferByte(0xFF);
        if ((response & 0x80) == 0) break;
    }
    
    EasyTVC_SPI3_CS_SD_Set(true);
    
    return response;
}

SD_Result SD_Raw_Init(SD_Card *sd) {
    current_sd = sd;
    sd->initialized = 0;
    sd->capacity = 0;
    sd->sector_size = 512;
    
    /* Deselect card */
    EasyTVC_SPI3_CS_SD_Set(true);
    delay_ms(10);
    
    /* Send 80 clock cycles */
    EasyTVC_SPI3_CS_SD_Set(false);
    for (uint8_t i = 0; i < 10; i++) {
        EasyTVC_SPI3_TransferByte(0xFF);
    }
    EasyTVC_SPI3_CS_SD_Set(true);
    
    delay_ms(10);
    
    /* Send CMD0 - go idle state */
    uint8_t response = sd_send_cmd(SD_CMD0, 0, 0x95);
    if (response != 0x01) {
        return SD_ERROR;
    }
    
    /* Send CMD8 - check voltage */
    response = sd_send_cmd(SD_CMD8, 0x000001AA, 0x87);
    
    /* Send CMD55 + ACMD41 to initialize */
    for (uint16_t i = 0; i < 1000; i++) {
        sd_send_cmd(SD_CMD55, 0, 0xFF);
        response = sd_send_cmd(SD_ACMD41, 0x40000000, 0xFF);
        if (response == 0x00) break;
        delay_ms(1);
    }
    
    if (response != 0x00) {
        return SD_ERROR;
    }
    
    /* Send CMD58 to read OCR */
    response = sd_send_cmd(SD_CMD58, 0, 0xFF);
    
    sd->initialized = 1;
    sd->capacity = 0;  /* Will need proper detection */
    
    return SD_OK;
}

SD_Result SD_Raw_ReadSector(SD_Card *sd, uint32_t sector, uint8_t *buffer) {
    if (!sd->initialized) {
        return SD_NOT_INITIALIZED;
    }
    
    uint8_t response;
    
    /* Send CMD17 - read single block */
    EasyTVC_SPI3_CS_SD_Set(false);
    EasyTVC_SPI3_TransferByte(0x40 | SD_CMD17);
    EasyTVC_SPI3_TransferByte((sector >> 24) & 0xFF);
    EasyTVC_SPI3_TransferByte((sector >> 16) & 0xFF);
    EasyTVC_SPI3_TransferByte((sector >> 8) & 0xFF);
    EasyTVC_SPI3_TransferByte(sector & 0xFF);
    EasyTVC_SPI3_TransferByte(0xFF);
    
    /* Wait for response */
    for (uint8_t i = 0; i < 8; i++) {
        response = EasyTVC_SPI3_TransferByte(0xFF);
        if (response == 0x00) break;
    }
    
    if (response != 0x00) {
        EasyTVC_SPI3_CS_SD_Set(true);
        return SD_ERROR;
    }
    
    /* Wait for data token (0xFE) */
    while (EasyTVC_SPI3_TransferByte(0xFF) != 0xFE);
    
    /* Read 512 bytes */
    for (uint16_t i = 0; i < 512; i++) {
        buffer[i] = EasyTVC_SPI3_TransferByte(0xFF);
    }
    
    /* Read CRC (2 bytes) */
    EasyTVC_SPI3_TransferByte(0xFF);
    EasyTVC_SPI3_TransferByte(0xFF);
    
    EasyTVC_SPI3_CS_SD_Set(true);
    
    return SD_OK;
}

SD_Result SD_Raw_WriteSector(SD_Card *sd, uint32_t sector, const uint8_t *buffer) {
    if (!sd->initialized) {
        return SD_NOT_INITIALIZED;
    }
    
    uint8_t response;
    
    /* Send CMD24 - write single block */
    EasyTVC_SPI3_CS_SD_Set(false);
    EasyTVC_SPI3_TransferByte(0x40 | SD_CMD24);
    EasyTVC_SPI3_TransferByte((sector >> 24) & 0xFF);
    EasyTVC_SPI3_TransferByte((sector >> 16) & 0xFF);
    EasyTVC_SPI3_TransferByte((sector >> 8) & 0xFF);
    EasyTVC_SPI3_TransferByte(sector & 0xFF);
    EasyTVC_SPI3_TransferByte(0xFF);
    
    /* Wait for response */
    for (uint8_t i = 0; i < 8; i++) {
        response = EasyTVC_SPI3_TransferByte(0xFF);
        if (response == 0x00) break;
    }
    
    if (response != 0x00) {
        EasyTVC_SPI3_CS_SD_Set(true);
        return SD_ERROR;
    }
    
    /* Send data token */
    EasyTVC_SPI3_TransferByte(0xFE);
    
    /* Write 512 bytes */
    for (uint16_t i = 0; i < 512; i++) {
        EasyTVC_SPI3_TransferByte(buffer[i]);
    }
    
    /* Write dummy CRC */
    EasyTVC_SPI3_TransferByte(0xFF);
    EasyTVC_SPI3_TransferByte(0xFF);
    
    /* Wait for data acceptance */
    response = EasyTVC_SPI3_TransferByte(0xFF);
    if ((response & 0x1F) != 0x05) {
        EasyTVC_SPI3_CS_SD_Set(true);
        return SD_ERROR;
    }
    
    /* Wait for write complete */
    while (EasyTVC_SPI3_TransferByte(0xFF) == 0x00);
    
    EasyTVC_SPI3_CS_SD_Set(true);
    
    return SD_OK;
}
