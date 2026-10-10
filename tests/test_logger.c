#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "data_logger.h"
#include "sd_raw.h"

/* In-memory stand-in for the card. The real sd_raw.c needs the SPI hardware. */
#define FAKE_SECTORS 2048U
static uint8_t card[FAKE_SECTORS][512];
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
    write_calls = 0;
}

static TelemetryRecord make_record(uint32_t n)
{
    TelemetryRecord r;
    memset(&r, 0, sizeof(r));
    r.timestamp = n;
    r.accel_z = (int16_t)(1000 + n);
    r.flags = TELEMETRY_FLAG_MARKER | TELEMETRY_FLAG_IMU_OK;
    return r;
}

static uint32_t count_valid_records(uint32_t sector, uint32_t expect_first_ts)
{
    uint32_t found = 0;
    for (unsigned i = 0; i + sizeof(TelemetryRecord) <= 512; i += sizeof(TelemetryRecord)) {
        TelemetryRecord r;
        memcpy(&r, &card[sector][i], sizeof(r));
        if ((r.flags & TELEMETRY_FLAG_MARKER_MASK) != TELEMETRY_FLAG_MARKER) {
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

/* Regression: the periodic flush used to reset the sector offset without advancing, so every
 * second of data overwrote the previous second in the same sector. */
static void test_periodic_flush_keeps_earlier_records(void)
{
    reset_card();
    DataLogger l;
    assert(DataLogger_Init(&l));

    uint32_t n = 0;
    for (int second = 0; second < 2; ++second) {
        for (int i = 0; i < 10; ++i) {
            TelemetryRecord r = make_record(n++);
            assert(DataLogger_Log(&l, &r));
        }
        assert(DataLogger_Flush(&l));
        drain(&l);
    }
    assert(count_valid_records(DATA_LOGGER_START_SECTOR, 0) == 20);
    assert(l.write_errors == 0 && l.dropped_records == 0);
}

static void test_records_roll_over_to_next_sector(void)
{
    reset_card();
    DataLogger l;
    assert(DataLogger_Init(&l));
    const uint32_t per_sector = 512 / sizeof(TelemetryRecord);
    assert(per_sector == 22);

    for (uint32_t n = 0; n < 50; ++n) {
        TelemetryRecord r = make_record(n);
        assert(DataLogger_Log(&l, &r));
        DataLogger_Poll(&l);
    }
    assert(DataLogger_Flush(&l));
    drain(&l);

    assert(count_valid_records(DATA_LOGGER_START_SECTOR, 0) == 22);
    assert(count_valid_records(DATA_LOGGER_START_SECTOR + 1, 22) == 22);
    assert(count_valid_records(DATA_LOGGER_START_SECTOR + 2, 44) == 6);
    assert(l.record_count == 50);
}

/* While the card is still programming a full sector, a new record must be dropped (and counted),
 * never block the control loop and never corrupt the sector in flight. */
static void test_busy_card_drops_instead_of_blocking(void)
{
    reset_card();
    busy_polls_per_write = 1000000;
    DataLogger l;
    assert(DataLogger_Init(&l));

    for (uint32_t n = 0; n < 22; ++n) {
        TelemetryRecord r = make_record(n);
        assert(DataLogger_Log(&l, &r));
    }
    TelemetryRecord r = make_record(22);
    assert(DataLogger_Log(&l, &r));            /* sector 1 handed to the card, 22 goes in sector 2 */
    for (uint32_t n = 23; n < 44; ++n) {
        TelemetryRecord x = make_record(n);
        assert(DataLogger_Log(&l, &x));
    }
    TelemetryRecord y = make_record(44);
    assert(!DataLogger_Log(&l, &y));           /* needs a new sector but the card is still busy */
    assert(l.dropped_records == 1);
    assert(count_valid_records(DATA_LOGGER_START_SECTOR, 0) == 22);
}

static void test_rejected_sector_is_skipped_not_retried_forever(void)
{
    reset_card();
    reject_writes = true;
    DataLogger l;
    assert(DataLogger_Init(&l));
    for (uint32_t n = 0; n < 100; ++n) {
        TelemetryRecord r = make_record(n);
        (void)DataLogger_Log(&l, &r);
    }
    assert(l.write_errors >= 3);
    assert(l.record_count == 100);
}

static void test_init_failure_disables_logging(void)
{
    reset_card();
    init_ok = false;
    DataLogger l;
    assert(!DataLogger_Init(&l));
    TelemetryRecord r = make_record(0);
    assert(!DataLogger_Log(&l, &r));
    assert(!DataLogger_Flush(&l));
    assert(write_calls == 0);
}

int main(void)
{
    test_periodic_flush_keeps_earlier_records();
    test_records_roll_over_to_next_sector();
    test_busy_card_drops_instead_of_blocking();
    test_rejected_sector_is_skipped_not_retried_forever();
    test_init_failure_disables_logging();
    puts("logger tests passed");
    return 0;
}
