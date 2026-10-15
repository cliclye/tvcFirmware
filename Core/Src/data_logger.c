#include "data_logger.h"
#include "sd_raw.h"

static SD_Card sd_card;

#define SECTOR_SIZE DATA_LOGGER_SECTOR_SIZE
#define RECORD_SIZE sizeof(TelemetryRecord)

_Static_assert(sizeof(TelemetryRecord) == 32, "log format is 32 bytes; update the reader if this changes");
_Static_assert(SECTOR_SIZE % sizeof(TelemetryRecord) == 0, "records must tile a sector");

static const uint8_t dir_magic[8] = {'E', 'T', 'V', 'C', 'D', 'I', 'R', '1'};
static const uint8_t log_magic[8] = {'E', 'T', 'V', 'C', 'L', 'O', 'G', '2'};

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

static bool same_bytes(const uint8_t *a, const uint8_t *b, size_t num)
{
    while (num--) {
        if (*a++ != *b++) {
            return false;
        }
    }
    return true;
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

/* Directory: magic, next session, its bitwise complement (guards against a torn/random sector). */
static uint32_t read_next_session(uint8_t *buffer)
{
    if (SD_Raw_ReadSector(&sd_card, DATA_LOGGER_DIR_SECTOR, buffer) != SD_OK ||
        !same_bytes(buffer, dir_magic, sizeof(dir_magic))) {
        return 0U;
    }
    const uint32_t next = get_u32(&buffer[8]);
    return get_u32(&buffer[12]) == ~next ? next : 0U;
}

bool DataLogger_Init(DataLogger *logger, const char *header_text)
{
    logger->initialized = false;
    logger->region_full = false;
    logger->write_in_flight = false;
    logger->dirty = false;
    logger->session = 0;
    logger->record_count = 0;
    logger->dropped_records = 0;
    logger->write_errors = 0;
    logger->sector_offset = 0;
    logger->queue_head = 0;
    logger->queue_count = 0;
    fill_ff(logger->fill, SECTOR_SIZE);

    if (SD_Raw_Init(&sd_card) != SD_OK) {
        return false;
    }

    /* Claim the session number BEFORE writing any data, so a crash cannot reuse it. */
    uint8_t *sector = logger->queue[0];
    const uint32_t session = read_next_session(sector);
    fill_ff(sector, SECTOR_SIZE);
    copy_bytes(sector, dir_magic, sizeof(dir_magic));
    put_u32(&sector[8], session + 1U);
    put_u32(&sector[12], ~(session + 1U));
    if (SD_Raw_WriteSector(&sd_card, DATA_LOGGER_DIR_SECTOR, sector) != SD_OK) {
        return false;
    }

    const uint32_t header_sector = DATA_LOGGER_FIRST_SESSION +
        (session % DATA_LOGGER_MAX_SESSIONS) * DATA_LOGGER_SESSION_SECTORS;
    for (size_t i = 0; i < SECTOR_SIZE; ++i) {
        sector[i] = 0U;
    }
    copy_bytes(sector, log_magic, sizeof(log_magic));
    put_u32(&sector[8], session);
    put_u32(&sector[12], (uint32_t)RECORD_SIZE);
    if (header_text != NULL) {
        for (size_t i = 0; header_text[i] != '\0' && 16U + i < SECTOR_SIZE - 1U; ++i) {
            sector[16U + i] = (uint8_t)header_text[i];
        }
    }
    if (SD_Raw_WriteSector(&sd_card, header_sector, sector) != SD_OK) {
        return false;
    }

    logger->session = session;
    logger->current_sector = header_sector + 1U;
    logger->end_sector = header_sector + DATA_LOGGER_SESSION_SECTORS;
    logger->initialized = true;
    return true;
}

void DataLogger_Poll(DataLogger *logger)
{
    if (!logger->initialized) {
        return;
    }
    if (logger->write_in_flight) {
        const SD_Result result = SD_Raw_WritePoll(&sd_card);
        if (result == SD_BUSY) {
            return;
        }
        logger->write_in_flight = false;
        if (result != SD_OK) {
            logger->write_errors++;
        }
        logger->queue_head = (uint8_t)((logger->queue_head + 1U) % DATA_LOGGER_QUEUE_SECTORS);
        logger->queue_count--;
    }
    while (logger->queue_count > 0U) {
        if (SD_Raw_WriteStart(&sd_card, logger->queue_sector[logger->queue_head],
                              logger->queue[logger->queue_head]) == SD_OK) {
            logger->write_in_flight = true;
            return;
        }
        /* A sector the card refuses is skipped rather than retried forever. */
        logger->write_errors++;
        logger->queue_head = (uint8_t)((logger->queue_head + 1U) % DATA_LOGGER_QUEUE_SECTORS);
        logger->queue_count--;
    }
}

/* Copy the fill buffer into the write queue. `advance` moves on to a fresh sector. */
static bool enqueue_fill(DataLogger *logger, bool advance)
{
    if (logger->queue_count >= DATA_LOGGER_QUEUE_SECTORS) {
        DataLogger_Poll(logger);
        if (logger->queue_count >= DATA_LOGGER_QUEUE_SECTORS) {
            return false;
        }
    }
    const uint8_t slot = (uint8_t)((logger->queue_head + logger->queue_count) % DATA_LOGGER_QUEUE_SECTORS);
    copy_bytes(logger->queue[slot], logger->fill, SECTOR_SIZE);
    logger->queue_sector[slot] = logger->current_sector;
    logger->queue_count++;
    logger->dirty = false;

    if (advance) {
        logger->current_sector++;
        logger->sector_offset = 0;
        fill_ff(logger->fill, SECTOR_SIZE);
        if (logger->current_sector >= logger->end_sector) {
            logger->region_full = true;
        }
    }
    DataLogger_Poll(logger);
    return true;
}

bool DataLogger_Log(DataLogger *logger, const TelemetryRecord *record)
{
    return DataLogger_LogRaw(logger, record);
}

bool DataLogger_LogRaw(DataLogger *logger, const void *record)
{
    if (!logger->initialized || logger->region_full) {
        return false;
    }
    copy_bytes(&logger->fill[logger->sector_offset], (const uint8_t *)record, RECORD_SIZE);
    const uint16_t next = (uint16_t)(logger->sector_offset + RECORD_SIZE);
    if (next < SECTOR_SIZE) {
        logger->sector_offset = next;
        logger->dirty = true;
    } else if (!enqueue_fill(logger, true)) {
        /* Queue full: un-write the record so the sector stays consistent; retry next time. */
        fill_ff(&logger->fill[logger->sector_offset], RECORD_SIZE);
        logger->dropped_records++;
        return false;
    }
    logger->record_count++;
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
    return enqueue_fill(logger, false);
}

uint32_t DataLogger_GetRecordCount(DataLogger *logger)
{
    return logger->record_count;
}
