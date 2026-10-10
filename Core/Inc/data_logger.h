#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <stdint.h>
#include <stdbool.h>

/* First raw sector written. The card is used WITHOUT a file system: whatever FAT volume was on
 * it is overwritten from here on. Use a dedicated card. */
#define DATA_LOGGER_START_SECTOR 1024U
#define DATA_LOGGER_SECTOR_SIZE  512U

/* flags: bits 7:5 are a fixed marker (0b101) so readers can tell real records from stale card
 * contents or the 0xFF padding at the end of each sector. */
#define TELEMETRY_FLAG_MARKER_MASK 0xE0U
#define TELEMETRY_FLAG_MARKER      0xA0U
#define TELEMETRY_FLAG_IMU_OK      0x01U
#define TELEMETRY_FLAG_BARO_OK     0x02U
#define TELEMETRY_FLAG_GYRO_CAL_OK 0x04U
#define TELEMETRY_FLAG_ACCEL_SAT   0x08U
#define TELEMETRY_FLAG_GYRO_SAT    0x10U

/* Binary telemetry record - packed for SD card storage (23 bytes, 22 per sector). */
typedef struct __attribute__((packed)) {
    uint32_t timestamp;       /* Milliseconds since boot */
    int16_t accel_x;          /* Accelerometer X (mg, sensor frame) */
    int16_t accel_y;          /* Accelerometer Y (mg) */
    int16_t accel_z;          /* Accelerometer Z (mg) */
    int16_t gyro_x;           /* Gyroscope X (0.1 deg/s, bias-corrected) */
    int16_t gyro_y;           /* Gyroscope Y (0.1 deg/s) */
    int16_t gyro_z;           /* Gyroscope Z (0.1 deg/s) */
    int16_t temperature;      /* Temperature (0.01 degC) */
    uint32_t pressure;        /* Pressure (Pa) */
    uint8_t flags;            /* TELEMETRY_FLAG_* */
} TelemetryRecord;

/* Logger state for SD card */
typedef struct {
    bool initialized;
    bool dirty;                  /* sector_buffer has records not yet handed to the card */
    bool write_in_flight;
    uint32_t record_count;
    uint32_t dropped_records;    /* lost because the card was still busy */
    uint32_t write_errors;       /* sectors the card rejected or timed out on */
    uint32_t current_sector;     /* Sector being filled */
    uint32_t write_sector;       /* Sector currently being programmed */
    uint16_t sector_offset;      /* Offset within sector (bytes) */
    uint8_t sector_buffer[DATA_LOGGER_SECTOR_SIZE];   /* being filled */
    uint8_t write_buffer[DATA_LOGGER_SECTOR_SIZE];    /* snapshot owned by the card */
} DataLogger;

/* Initialize data logger to SD card (blocks for the SD init, boot time only). */
bool DataLogger_Init(DataLogger *logger);

/* Append a record. Never blocks on the card: if a full sector cannot be handed over because the
 * previous write is still programming, the record is counted in dropped_records. */
bool DataLogger_Log(DataLogger *logger, const TelemetryRecord *record);

/* Hand the partly filled sector to the card (it keeps filling the same sector afterwards). */
bool DataLogger_Flush(DataLogger *logger);

/* Call every loop: finishes the in-flight sector write without blocking. */
void DataLogger_Poll(DataLogger *logger);

/* Get current record count */
uint32_t DataLogger_GetRecordCount(DataLogger *logger);

#endif // DATA_LOGGER_H
