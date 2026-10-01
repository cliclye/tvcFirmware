#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <stdint.h>
#include <stdbool.h>

/* Binary telemetry record - packed for SD card storage */
typedef struct __attribute__((packed)) {
    uint32_t timestamp;       /* Milliseconds since boot */
    int16_t accel_x;          /* Accelerometer X (mg) */
    int16_t accel_y;          /* Accelerometer Y (mg) */
    int16_t accel_z;          /* Accelerometer Z (mg) */
    int16_t gyro_x;           /* Gyroscope X (deg/s) */
    int16_t gyro_y;           /* Gyroscope Y (deg/s) */
    int16_t gyro_z;           /* Gyroscope Z (deg/s) */
    int16_t temperature;      /* Temperature (0.01°C) */
    uint32_t pressure;        /* Pressure (Pa) */
    uint8_t flags;           /* Status flags */
} TelemetryRecord;

/* Logger state for SD card */
typedef struct {
    bool initialized;
    uint32_t record_count;
    uint32_t current_sector;  /* Current SD card sector */
    uint16_t sector_offset;   /* Offset within sector (bytes) */
    uint8_t sector_buffer[512];  /* Buffer for SD card writes */
} DataLogger;

/* Initialize data logger to SD card */
bool DataLogger_Init(DataLogger *logger);

/* Log sensor data to SD card */
bool DataLogger_Log(DataLogger *logger, const TelemetryRecord *record);

/* Flush remaining data to SD card */
bool DataLogger_Flush(DataLogger *logger);

/* Get current record count */
uint32_t DataLogger_GetRecordCount(DataLogger *logger);

#endif // DATA_LOGGER_H
