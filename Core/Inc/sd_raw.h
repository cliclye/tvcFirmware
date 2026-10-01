#ifndef SD_RAW_H
#define SD_RAW_H

#include <stdint.h>
#include <stdbool.h>

/* SD card result codes */
typedef enum {
    SD_OK = 0,
    SD_ERROR,
    SD_TIMEOUT,
    SD_NOT_INITIALIZED
} SD_Result;

/* SD card handle */
typedef struct {
    uint8_t initialized;
    uint32_t capacity;  /* in sectors */
    uint32_t sector_size;
} SD_Card;

/* Initialize SD card on SPI3 */
SD_Result SD_Raw_Init(SD_Card *sd);

/* Read sector from SD card */
SD_Result SD_Raw_ReadSector(SD_Card *sd, uint32_t sector, uint8_t *buffer);

/* Write sector to SD card */
SD_Result SD_Raw_WriteSector(SD_Card *sd, uint32_t sector, const uint8_t *buffer);

#endif // SD_RAW_H
