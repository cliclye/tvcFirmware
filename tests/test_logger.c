#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "data_logger.h"
#include "sd_raw.h"

/* In-memory stand-in for the card. The real sd_raw.c needs the SPI hardware. */
#define FAKE_SECTORS (DATA_LOGGER_FIRST_SESSION + 3U * DATA_LOGGER_SESSION_SECTORS)
static uint8_t card[FAKE_SECTORS][512];
static bool fail_writes_to_dir;
static bool init_ok;
static unsigned busy_polls_per_write;   /* how many SD_BUSY answers each write produces */
static unsigned busy_remaining;
static bool write_pending;
static bool reject_writes;
static unsigned write_calls;

SD_Result SD_Raw_Init(SD_Card *sd)
{
    sd->initialized = init_ok ? 1U : 0U;
    return init_ok ? SD_OK : SD_ERROR;
}

SD_Result SD_Raw_ReadSector(SD_Card *sd, uint32_t sector, uint8_t *buffer)
{
    (void)sd;
    assert(sector < FAKE_SECTORS);
    memcpy(buffer, card[sector], 512);
    return SD_OK;
}

SD_Result SD_Raw_WriteStart(SD_Card *sd, uint32_t sector, const uint8_t *buffer)
{
    (void)sd;
    assert(!write_pending);   /* the logger must never overlap card transactions */
    ++write_calls;
    if (reject_writes) {
        return SD_ERROR;
    }
    assert(sector < FAKE_SECTORS);
    if (fail_writes_to_dir && sector == DATA_LOGGER_DIR_SECTOR) {
        return SD_ERROR;
    }
    memcpy(card[sector], buffer, 512);
    write_pending = true;
    busy_remaining = busy_polls_per_write;
    return SD_OK;
}

SD_Result SD_Raw_WritePoll(SD_Card *sd)
{
    (void)sd;
    if (!write_pending) {
        return SD_OK;
    }
    if (busy_remaining > 0) {
        --busy_remaining;
        return SD_BUSY;
    }
    write_pending = false;
    return SD_OK;
}

SD_Result SD_Raw_WriteSector(SD_Card *sd, uint32_t sector, const uint8_t *buffer)
{
    SD_Result r = SD_Raw_WriteStart(sd, sector, buffer);
    while (r == SD_OK || r == SD_BUSY) {
        r = SD_Raw_WritePoll(sd);
        if (r == SD_OK) return SD_OK;
    }
    return r;
}

static void reset_card(void)
{
    memset(card, 0xFF, sizeof(card));
    init_ok = true;
    busy_polls_per_write = 0;
    busy_remaining = 0;
    write_pending = false;
    reject_writes = false;
    fail_writes_to_dir = false;
    write_calls = 0;
}

#define PER_SECTOR (512U / sizeof(TelemetryRecord))

static uint32_t session_header(uint32_t session)
{
    return DATA_LOGGER_FIRST_SESSION + (session % DATA_LOGGER_MAX_SESSIONS) * DATA_LOGGER_SESSION_SECTORS;
}

static TelemetryRecord make_record(uint32_t n)
{
    TelemetryRecord r;
    memset(&r, 0, sizeof(r));
    r.timestamp = n;
    r.accel[2] = (int16_t)(1000 + n);
    r.flags = TELEMETRY_FLAG_IMU_OK;
    r.marker = TELEMETRY_MARKER;
    return r;
}

static uint32_t count_valid_records(uint32_t sector, uint32_t expect_first_ts)
{
    uint32_t found = 0;
    for (unsigned i = 0; i + sizeof(TelemetryRecord) <= 512; i += sizeof(TelemetryRecord)) {
        TelemetryRecord r;
        memcpy(&r, &card[sector][i], sizeof(r));
        if (r.marker != TELEMETRY_MARKER) {
            break;
        }
        assert(r.timestamp == expect_first_ts + found);
        ++found;
    }
    return found;
}

static void drain(DataLogger *l)
{
    for (int i = 0; i < 100; ++i) {
        DataLogger_Poll(l);
    }
}

static DataLogger l;   /* ~5 KB: keep it off the stack */

/* Every power-up gets a new session region, so rebooting after a flight keeps that flight. */
static void test_sessions_do_not_overwrite_each_other(void)
{
    reset_card();
    assert(DataLogger_Init(&l, "mode=flight"));
    assert(l.session == 0);
    assert(memcmp(card[session_header(0)], "ETVCLOG2", 8) == 0);
    assert(strcmp((const char *)&card[session_header(0)][16], "mode=flight") == 0);
    for (uint32_t n = 0; n < 5; ++n) {
        TelemetryRecord r = make_record(n);
        assert(DataLogger_Log(&l, &r));
    }
    assert(DataLogger_Flush(&l));
    drain(&l);

    assert(DataLogger_Init(&l, NULL));
    assert(l.session == 1);
    TelemetryRecord r = make_record(100);
    assert(DataLogger_Log(&l, &r));
    assert(DataLogger_Flush(&l));
    drain(&l);

    assert(count_valid_records(session_header(0) + 1U, 0) == 5);
    assert(count_valid_records(session_header(1) + 1U, 100) == 1);
}

static void test_corrupt_directory_restarts_at_session_zero(void)
{
    reset_card();
    memcpy(card[DATA_LOGGER_DIR_SECTOR], "ETVCDIR1\x05\x00\x00\x00\x00\x00\x00\x00", 16);
    assert(DataLogger_Init(&l, NULL));
    assert(l.session == 0);
}

static void test_directory_write_failure_disables_logging(void)
{
    reset_card();
    fail_writes_to_dir = true;
    assert(!DataLogger_Init(&l, NULL));
    TelemetryRecord r = make_record(0);
    assert(!DataLogger_Log(&l, &r));
}

/* Regression: the periodic flush used to reset the sector offset without advancing, so every
 * second of data overwrote the previous second in the same sector. */
static void test_periodic_flush_keeps_earlier_records(void)
{
    reset_card();
    assert(DataLogger_Init(&l, NULL));
    const uint32_t first = session_header(0) + 1U;

    uint32_t n = 0;
    for (int second = 0; second < 2; ++second) {
        for (int i = 0; i < 6; ++i) {
            TelemetryRecord r = make_record(n++);
            assert(DataLogger_Log(&l, &r));
        }
        assert(DataLogger_Flush(&l));
        drain(&l);
    }
    assert(count_valid_records(first, 0) == 12);
    assert(l.write_errors == 0 && l.dropped_records == 0);
}

static void test_records_roll_over_to_next_sector(void)
{
    reset_card();
    assert(DataLogger_Init(&l, NULL));
    const uint32_t first = session_header(0) + 1U;
    assert(PER_SECTOR == 16);

    for (uint32_t n = 0; n < 40; ++n) {
        TelemetryRecord r = make_record(n);
        assert(DataLogger_Log(&l, &r));
        DataLogger_Poll(&l);
    }
    assert(DataLogger_Flush(&l));
    drain(&l);

    assert(count_valid_records(first, 0) == 16);
    assert(count_valid_records(first + 1, 16) == 16);
    assert(count_valid_records(first + 2, 32) == 8);
    assert(l.record_count == 40);
}

/* A slow card is absorbed by the RAM queue; only when that is full are records dropped, and the
 * loop is never blocked and sectors already queued are never corrupted. */
static void test_busy_card_queues_then_drops(void)
{
    reset_card();
    assert(DataLogger_Init(&l, NULL));
    const uint32_t first = session_header(0) + 1U;
    busy_polls_per_write = 1000000;

    const uint32_t capacity = DATA_LOGGER_QUEUE_SECTORS * PER_SECTOR;
    uint32_t n = 0;
    for (; n < capacity; ++n) {
        TelemetryRecord r = make_record(n);
        assert(DataLogger_Log(&l, &r));
    }
    assert(l.dropped_records == 0);
    for (uint32_t extra = 0; extra < 15; ++extra) {
        TelemetryRecord r = make_record(n++);
        assert(DataLogger_Log(&l, &r));        /* fills the RAM sector */
    }
    TelemetryRecord r = make_record(n);
    assert(!DataLogger_Log(&l, &r));           /* that sector cannot be queued */
    assert(l.dropped_records == 1);

    busy_polls_per_write = 0;
    busy_remaining = 0;
    drain(&l);
    for (uint32_t s = 0; s < DATA_LOGGER_QUEUE_SECTORS; ++s) {
        assert(count_valid_records(first + s, s * PER_SECTOR) == PER_SECTOR);
    }
    assert(DataLogger_Log(&l, &r));            /* room again */
    drain(&l);
    assert(count_valid_records(first + DATA_LOGGER_QUEUE_SECTORS, capacity) == PER_SECTOR);
}

static void test_rejected_sector_is_skipped_not_retried_forever(void)
{
    reset_card();
    assert(DataLogger_Init(&l, NULL));
    reject_writes = true;
    for (uint32_t n = 0; n < 100; ++n) {
        TelemetryRecord r = make_record(n);
        (void)DataLogger_Log(&l, &r);
        DataLogger_Poll(&l);
    }
    assert(l.write_errors >= 5);
    assert(l.record_count == 100);
}

static void test_init_failure_disables_logging(void)
{
    reset_card();
    init_ok = false;
    assert(!DataLogger_Init(&l, NULL));
    TelemetryRecord r = make_record(0);
    assert(!DataLogger_Log(&l, &r));
    assert(!DataLogger_Flush(&l));
    assert(write_calls == 0);
}

int main(void)
{
    test_sessions_do_not_overwrite_each_other();
    test_corrupt_directory_restarts_at_session_zero();
    test_directory_write_failure_disables_logging();
    test_periodic_flush_keeps_earlier_records();
    test_records_roll_over_to_next_sector();
    test_busy_card_queues_then_drops();
    test_rejected_sector_is_skipped_not_retried_forever();
    test_init_failure_disables_logging();
    puts("logger tests passed");
    return 0;
}
