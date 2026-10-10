#include "data_logger.h"
#include "sd_raw.h"
#include <stddef.h>

static SD_Card sd_card;

#define TELEMETRY_RECORD_SIZE sizeof(TelemetryRecord)
#define SECTOR_SIZE DATA_LOGGER_SECTOR_SIZE

_Static_assert(sizeof(TelemetryRecord) == 23, "log format is 23 bytes; update the reader if this changes");

static void fill_ff(uint8_t *ptr, size_t num)
{
    while (num--) {
        *ptr++ = 0xFFU;
    }
}

static void copy_bytes(uint8_t *dest, const uint8_t *src, size_t num)
{
    while (num--) {
        *dest++ = *src++;
    }
}

bool DataLogger_Init(DataLogger *logger)
{
    logger->initialized = false;
    logger->dirty = false;
    logger->write_in_flight = false;
    logger->record_count = 0;
    logger->dropped_records = 0;
    logger->write_errors = 0;
    logger->current_sector = DATA_LOGGER_START_SECTOR;
    logger->write_sector = 0;
    logger->sector_offset = 0;
    fill_ff(logger->sector_buffer, SECTOR_SIZE);
    fill_ff(logger->write_buffer, SECTOR_SIZE);

    if (SD_Raw_Init(&sd_card) != SD_OK) {
        return false;
    }

    logger->initialized = true;
    return true;
}

void DataLogger_Poll(DataLogger *logger)
{
    if (!logger->initialized || !logger->write_in_flight) {
        return;
    }
    const SD_Result result = SD_Raw_WritePoll(&sd_card);
    if (result == SD_BUSY) {
        return;
    }
    logger->write_in_flight = false;
    if (result != SD_OK) {
        logger->write_errors++;
    }
}

/* Snapshot the sector buffer and start programming it. `advance` moves on to a fresh sector. */
static bool commit(DataLogger *logger, bool advance)
{
    DataLogger_Poll(logger);
    if (logger->write_in_flight) {
        return false;
    }

    copy_bytes(logger->write_buffer, logger->sector_buffer, SECTOR_SIZE);
    logger->write_sector = logger->current_sector;
    if (SD_Raw_WriteStart(&sd_card, logger->write_sector, logger->write_buffer) != SD_OK) {
        logger->write_errors++;
        if (!advance) {
            return false;
        }
        /* A full sector the card refuses is skipped rather than retried forever. */
    } else {
        logger->write_in_flight = true;
    }
    logger->dirty = false;

    if (advance) {
        logger->current_sector++;
        logger->sector_offset = 0;
        fill_ff(logger->sector_buffer, SECTOR_SIZE);
    }
    return true;
}

bool DataLogger_Log(DataLogger *logger, const TelemetryRecord *record)
{
    if (!logger->initialized) {
        return false;
    }

    if (logger->sector_offset + TELEMETRY_RECORD_SIZE > SECTOR_SIZE) {
        if (!commit(logger, true)) {
            logger->dropped_records++;
            return false;
        }
    }

    copy_bytes(&logger->sector_buffer[logger->sector_offset], (const uint8_t *)record, TELEMETRY_RECORD_SIZE);
    logger->sector_offset = (uint16_t)(logger->sector_offset + TELEMETRY_RECORD_SIZE);
    logger->record_count++;
    logger->dirty = true;
    return true;
}

bool DataLogger_Flush(DataLogger *logger)
{
    if (!logger->initialized) {
        return false;
    }
    if (!logger->dirty) {
        return true;
    }
    /* Write the partial sector but KEEP filling it: the old code reset sector_offset here
     * without advancing, so every periodic flush overwrote the previous second of data. */
    return commit(logger, false);
}

uint32_t DataLogger_GetRecordCount(DataLogger *logger)
{
    return logger->record_count;
}
