#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/*
 * Raw-sector SD log, no file system. Use a dedicated card (>= 1 GB): everything from sector 1024
 * on is overwritten.
 *
 *   sector 1024                      directory: "ETVCDIR1", next session number
 *   2048 + (n % 32) * 16384          session n header: "ETVCLOG2", session, record size, text
 *   ... + 1 onwards                  32-byte records, 16 per sector
 *
 * Every power-up is a new session in its own 8 MB region, so switching the board back on after
 * a flight does not overwrite that flight. The 32 most recent sessions are kept.
 */
#define DATA_LOGGER_DIR_SECTOR      1024U
#define DATA_LOGGER_FIRST_SESSION   2048U
#define DATA_LOGGER_SESSION_SECTORS 16384U
#define DATA_LOGGER_MAX_SESSIONS    32U
#define DATA_LOGGER_SECTOR_SIZE     512U
#define DATA_LOGGER_QUEUE_SECTORS   8U      /* RAM buffer: rides out ~1 s of slow card */

#define TELEMETRY_MARKER            0xA5U

#define TELEMETRY_FLAG_IMU_OK       0x01U
#define TELEMETRY_FLAG_BARO_OK      0x02U
#define TELEMETRY_FLAG_GYRO_CAL_OK  0x04U
#define TELEMETRY_FLAG_ACCEL_SAT    0x08U
#define TELEMETRY_FLAG_GYRO_SAT     0x10U
#define TELEMETRY_FLAG_TVC_ACTIVE   0x20U
#define TELEMETRY_FLAG_CTRL_FAULT   0x40U
#define TELEMETRY_FLAG_CHUTE_OPEN   0x80U

/* Binary telemetry record - packed for SD card storage (32 bytes, 16 per sector). Body frame. */
typedef struct __attribute__((packed)) {
    uint32_t timestamp;       /* ms since boot */
    int16_t accel[3];         /* mg */
    int16_t gyro[3];          /* 0.1 deg/s, bias-corrected */
    int16_t tilt[2];          /* 0.01 deg, about body X / Y (what the controller corrects) */
    uint16_t servo_us[2];     /* pulse sent to TVC servo A / B */
    int16_t baro_alt_dm;      /* 0.1 m above the pad */
    int16_t vspeed_cm_s;      /* estimated vertical speed, cm/s */
    uint8_t session;          /* low byte of the session number */
    uint8_t state;            /* EasyTVCFlightState */
    uint8_t flags;            /* TELEMETRY_FLAG_* */
    uint8_t marker;           /* TELEMETRY_MARKER */
} TelemetryRecord;

typedef struct {
    bool initialized;
    bool region_full;            /* this session's 8 MB is used up; logging stopped */
    bool write_in_flight;
    bool dirty;                  /* fill buffer holds records not yet queued */
    uint32_t session;
    uint32_t record_count;
    uint32_t dropped_records;    /* lost because the card fell behind */
    uint32_t write_errors;       /* sectors the card rejected or timed out on */
    uint32_t current_sector;     /* sector being filled */
    uint32_t end_sector;         /* first sector past this session's region */
    uint16_t sector_offset;
    uint8_t fill[DATA_LOGGER_SECTOR_SIZE];
    uint8_t queue[DATA_LOGGER_QUEUE_SECTORS][DATA_LOGGER_SECTOR_SIZE];
    uint32_t queue_sector[DATA_LOGGER_QUEUE_SECTORS];
    uint8_t queue_head;
    uint8_t queue_count;
} DataLogger;

/* Initialise the card, open a new session and write its header (blocking, boot time only).
 * `header_text` (may be NULL) is stored in the session header for later reference. */
bool DataLogger_Init(DataLogger *logger, const char *header_text);

/* Append a record. Never blocks on the card: if the RAM queue is full the record is counted
 * in dropped_records. */
bool DataLogger_Log(DataLogger *logger, const TelemetryRecord *record);

/* Same, for any other 32-byte record layout (the sensor-test image's records). */
bool DataLogger_LogRaw(DataLogger *logger, const void *record);

/* Queue the partly filled sector so a power loss costs at most ~1 s (it keeps filling). */
bool DataLogger_Flush(DataLogger *logger);

/* Call every loop: advances queued writes without blocking. */
void DataLogger_Poll(DataLogger *logger);

uint32_t DataLogger_GetRecordCount(DataLogger *logger);

#endif // DATA_LOGGER_H
