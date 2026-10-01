#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <stdint.h>
#include <stdbool.h>

/* Binary telemetry record - packed for flash storage */
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

/* Logger state for flash memory */
typedef struct {
    bool initialized;
    uint32_t record_count;
    uint32_t current_address;  /* Current flash write address */
    uint32_t sector_count;     /* 4KB sectors used */
} DataLogger;

/* Initialize data logger to flash memory */
bool DataLogger_Init(DataLogger *logger);

/* Log sensor data to flash */
bool DataLogger_Log(DataLogger *logger, const TelemetryRecord *record);

/* Get current record count */
uint32_t DataLogger_GetRecordCount(DataLogger *logger);

/* Get current flash address */
uint32_t DataLogger_GetCurrentAddress(DataLogger *logger);

#endif // DATA_LOGGER_H
