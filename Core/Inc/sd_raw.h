#ifndef SD_RAW_H
#define SD_RAW_H

#include <stdint.h>
#include <stdbool.h>

/* SD card result codes */
typedef enum {
    SD_OK = 0,
    SD_ERROR,
    SD_TIMEOUT,
    SD_NOT_INITIALIZED,
    SD_BUSY
} SD_Result;

/* SD card handle */
typedef struct {
    uint8_t initialized;
    uint8_t block_addressed;   /* 1 = SDHC/SDXC (sector index), 0 = SDSC (byte address) */
    uint32_t capacity;         /* in sectors (not detected yet; 0) */
    uint32_t sector_size;
} SD_Card;

/* Initialise the card over SPI3 (slow clock), then switch SPI3 to full speed. */
SD_Result SD_Raw_Init(SD_Card *sd);

/* Blocking read/write of one 512-byte sector, with timeouts. */
SD_Result SD_Raw_ReadSector(SD_Card *sd, uint32_t sector, uint8_t *buffer);
SD_Result SD_Raw_WriteSector(SD_Card *sd, uint32_t sector, const uint8_t *buffer);

/* Non-blocking write. Start clocks the 512 bytes out (~0.5 ms) and returns; the card then spends
 * milliseconds programming flash. Poll SD_Raw_WritePoll() until it stops returning SD_BUSY.
 * No other SD access is allowed while a write is pending. */
SD_Result SD_Raw_WriteStart(SD_Card *sd, uint32_t sector, const uint8_t *buffer);
SD_Result SD_Raw_WritePoll(SD_Card *sd);

#endif // SD_RAW_H
