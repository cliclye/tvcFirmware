#include "data_logger.h"
#include "sd_raw.h"
#include <stddef.h>

static DataLogger *current_logger = NULL;
static SD_Card sd_card;

#define TELEMETRY_RECORD_SIZE sizeof(TelemetryRecord)
#define SECTOR_SIZE 512
#define SECTORS_PER_CLUSTER 8  /* 4KB clusters */

/* Manual memset implementation */
static void my_memset(void *ptr, int value, size_t num) {
    unsigned char *p = (unsigned char *)ptr;
    while (num--) {
        *p++ = (unsigned char)value;
    }
}

/* Manual memcpy implementation */
static void my_memcpy(void *dest, const void *src, size_t num) {
    unsigned char *d = (unsigned char *)dest;
    const unsigned char *s = (const unsigned char *)src;
    while (num--) {
        *d++ = *s++;
    }
}

bool DataLogger_Init(DataLogger *logger) {
    current_logger = logger;
    logger->initialized = false;
    logger->record_count = 0;
    logger->current_sector = 1024;  /* Start at sector 1024 (512KB offset) */
    logger->sector_offset = 0;
    my_memset(logger->sector_buffer, 0xFF, SECTOR_SIZE);
    
    /* Initialize SD card */
    SD_Result result = SD_Raw_Init(&sd_card);
    if (result != SD_OK) {
        return false;
    }
    
    logger->initialized = true;
    return true;
}

bool DataLogger_Log(DataLogger *logger, const TelemetryRecord *record) {
    if (!logger->initialized) {
        return false;
    }
    
    /* Check if record fits in current sector */
    if (logger->sector_offset + TELEMETRY_RECORD_SIZE > SECTOR_SIZE) {
        /* Flush current sector */
        if (!DataLogger_Flush(logger)) {
            return false;
        }
        
        /* Move to next sector */
        logger->current_sector++;
        logger->sector_offset = 0;
        my_memset(logger->sector_buffer, 0xFF, SECTOR_SIZE);
    }
    
    /* Copy record to sector buffer */
    my_memcpy(&logger->sector_buffer[logger->sector_offset], record, TELEMETRY_RECORD_SIZE);
    logger->sector_offset += TELEMETRY_RECORD_SIZE;
    logger->record_count++;
    
    return true;
}

bool DataLogger_Flush(DataLogger *logger) {
    if (!logger->initialized) {
        return false;
    }
    
    if (logger->sector_offset == 0) {
        return true;  /* Nothing to flush */
    }
    
    /* Write sector to SD card */
    SD_Result result = SD_Raw_WriteSector(&sd_card, logger->current_sector, logger->sector_buffer);
    
    if (result != SD_OK) {
        return false;
    }
    
    logger->sector_offset = 0;
    return true;
}

uint32_t DataLogger_GetRecordCount(DataLogger *logger) {
    return logger->record_count;
}
