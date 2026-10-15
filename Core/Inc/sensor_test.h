#pragma once

#include <stdint.h>

/*
 * Sensor-test image (easytvc_sensortest): reads every sensor on the board at its full rate and
 * logs every raw reading to the SD card. Never arms, never moves a servo (all outputs held at
 * 1500 us). The session header text starts with "EasyTVC mode=sensortest".
 *
 * Records are 32 bytes like the flight log, with the same tail (session tag, type, sequence,
 * 0xA5 marker) so the reader finds the end of the session the same way.
 * scripts/telemetry_viewer.py (parse_sensor_session) decodes them; keep the two in sync.
 */
#define SENSOR_TEST_IMU_HZ    400U   /* BMI088 output data rate */
#define SENSOR_TEST_BARO_HZ   50U
#define SENSOR_TEST_ADC_HZ    10U

enum {
    SENSOR_REC_IMU = 1,      /* every BMI088 sample */
    SENSOR_REC_BARO = 2,     /* every BME280 sample */
    SENSOR_REC_ADC = 3,      /* battery pin, internal reference, MCU temperature */
    SENSOR_REC_STATUS = 4,   /* once a second: chip IDs, check results, counters */
};

/* Check bits (status record, serial line). All set = every sensor passed. */
enum {
    SENSOR_CHECK_ACCEL_ID   = 1u << 0,   /* BMI088 accel answers with chip ID 0x1E */
    SENSOR_CHECK_GYRO_ID    = 1u << 1,   /* BMI088 gyro answers with chip ID 0x0F */
    SENSOR_CHECK_BARO_ID    = 1u << 2,   /* BME280 answers with chip ID 0x60 */
    SENSOR_CHECK_ACCEL_1G   = 1u << 3,   /* at rest the accelerometer reads 1 g (0.9..1.1) */
    SENSOR_CHECK_GYRO_STILL = 1u << 4,   /* at rest: gyro bias < 5 deg/s, noise < 3 deg/s p-p */
    SENSOR_CHECK_IMU_ALIVE  = 1u << 5,   /* raw values change sample to sample (not stuck) */
    SENSOR_CHECK_IMU_RATE   = 1u << 6,   /* >= 99 % of IMU reads succeeded */
    SENSOR_CHECK_PRESSURE   = 1u << 7,   /* 60..110 kPa */
    SENSOR_CHECK_BARO_TEMP  = 1u << 8,   /* -20..60 C */
    SENSOR_CHECK_HUMIDITY   = 1u << 9,   /* 0..100 %RH and not stuck */
    SENSOR_CHECK_BARO_RATE  = 1u << 10,  /* >= 95 % of baro reads succeeded */
    SENSOR_CHECK_ACCEL_TEMP = 1u << 11,  /* BMI088 die temperature -20..70 C */
    SENSOR_CHECK_SD         = 1u << 12,  /* card initialised, no write errors, nothing dropped */
    SENSOR_CHECK_SUPPLY     = 1u << 13,  /* 3.3 V rail 3.0..3.6 V (from the internal reference) */
    SENSOR_CHECK_ALL        = (1u << 14) - 1u,
};

typedef struct __attribute__((packed)) {
    uint32_t t_us;                   /* microseconds since boot */
    union {
        struct __attribute__((packed)) {
            int16_t accel[3];        /* raw counts, sensor frame; g = raw * 24 / 32768 */
            int16_t gyro[3];         /* raw counts, sensor frame; deg/s = raw * 2000 / 32768 */
            int16_t accel_temp;      /* BMI088 die temperature, 0.01 C (latest, 10 Hz) */
            uint16_t read_us;        /* how long the SPI read took */
            uint16_t errors;         /* IMU read failures so far */
            uint8_t reserved[6];
        } imu;
        struct __attribute__((packed)) {
            uint32_t pressure_pa;    /* compensated */
            int32_t temperature_cc;  /* 0.01 C */
            uint32_t humidity_q10;   /* %RH * 1024 */
            uint32_t adc_p;          /* raw 20-bit conversions */
            uint32_t adc_t;
            uint16_t adc_h;          /* raw 16-bit */
            uint16_t errors;         /* baro read failures so far */
        } baro;
        struct __attribute__((packed)) {
            uint16_t batt_raw;       /* ADC1 IN4 (PA4), 12-bit */
            uint16_t vref_raw;       /* internal 1.21 V reference */
            uint16_t mcu_temp_raw;   /* internal temperature sensor */
            uint16_t vdda_mv;        /* 3.3 V rail, from the factory VREFINT calibration */
            uint16_t batt_pin_mv;    /* voltage at PA4 (the board's divider is not known yet) */
            int16_t mcu_temp_cc;     /* 0.01 C (datasheet typical, +-1.5 C) */
            uint16_t loop_overruns;  /* 400 Hz loop ticks that ran late */
            uint16_t max_loop_us;    /* longest loop body this second */
            uint8_t reserved[8];
        } adc;
        struct __attribute__((packed)) {
            uint32_t checks;         /* SENSOR_CHECK_* passing now */
            uint32_t boot_checks;    /* SENSOR_CHECK_* that passed the power-up test */
            uint8_t accel_id, gyro_id, baro_id, reserved;
            uint32_t imu_samples;
            uint32_t baro_samples;
            uint16_t sd_dropped;
            uint16_t sd_write_errors;
        } status;
    } u;
    uint8_t session;                 /* low byte of the session number */
    uint8_t type;                    /* SENSOR_REC_* */
    uint8_t seq;                     /* per-record counter, catches gaps */
    uint8_t marker;                  /* TELEMETRY_MARKER (0xA5) */
} SensorTestRecord;

_Static_assert(sizeof(SensorTestRecord) == 32, "sensor-test records are 32 bytes");
