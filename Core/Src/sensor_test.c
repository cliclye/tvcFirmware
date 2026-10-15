#include "board_pins.h"
#include "stm32f4xx.h"
#include "gpio_init.h"
#include "spi_driver.h"
#include "i2c_driver.h"
#include "pwm_driver.h"
#include "uart_driver.h"
#include "hardware_bus.h"
#include "bmi088.h"
#include "bme280.h"
#include "data_logger.h"
#include "sensor_test.h"
#include "vehicle_config.h"

#include <math.h>
#include <string.h>

/*
 * easytvc_sensortest: checks every sensor on the board and logs every raw reading to the SD card.
 *
 * Put the board flat and still, power it up:
 *   1. LEDs red, green, blue one at a time, then one beep (the outputs work).
 *   2. 3 s power-up test: chip IDs, 1 g at rest, gyro still and quiet, pressure / temperature /
 *      humidity in range, SD card, 3.3 V rail.
 *   3. Result: two short beeps + steady green = all passed.
 *      N long beeps + red blinking N times = failed group N (1 IMU, 2 barometer, 3 SD card,
 *      4 power), the first failing group. The details are in the log and on the serial line.
 *   4. Keeps reading and logging: IMU 400 Hz, barometer 50 Hz, ADC 10 Hz, status 1 Hz. Blue blinks
 *      while the SD card is recording. One power-up = one SD session (~9 minutes of space).
 *
 * Servos are held at 1500 us and never move. Nothing arms.
 * Serial (TX2, 115200): "T,..." at 10 Hz with scaled values, "Q,..." at 1 Hz with check results.
 */

#define LOOP_HZ          SENSOR_TEST_IMU_HZ
#define LOOP_PERIOD_US   (1000000UL / LOOP_HZ)
#define BARO_DIVIDER     (LOOP_HZ / SENSOR_TEST_BARO_HZ)
#define ADC_DIVIDER      (LOOP_HZ / SENSOR_TEST_ADC_HZ)
#define SERIAL_DIVIDER   (LOOP_HZ / 10U)
#define STATUS_DIVIDER   LOOP_HZ
#define BOOT_TEST_TICKS  (3U * LOOP_HZ)

/* counts per g and per deg/s, from the ranges the chip actually reports (EasyTVC_Bmi088Configure) */
#define GRAVITY_LSB      (9.80665f / bmi088.accel_m_s2_per_lsb)
#define GYRO_LSB_DPS     (0.017453292519943f / bmi088.gyro_rad_s_per_lsb)

/* ADC common registers (not in stm32f4xx.h) and the factory VREFINT calibration (3.3 V, 30 C). */
#define ADC_COMMON_CCR      (*(volatile uint32_t *)(STM32_ADC1_BASE + 0x300UL + 0x04UL))   /* 0x40012304 */
#define ADC_CCR_TSVREFE     (1UL << 23)
#define ADC_CCR_ADCPRE_DIV4 (1UL << 16)               /* 84 MHz / 4 = 21 MHz (max 36) */
#define VREFINT_CAL         (*(const volatile uint16_t *)0x1FFF7A2AUL)
#define ADC_CH_BATT         EASYTVC_ADC_BATT_CHANNEL
#define ADC_CH_TEMP         16U
#define ADC_CH_VREF         17U

static EasyTVCBmi088 bmi088;
static EasyTVCBme280 bme280;
static DataLogger logger;
static uint8_t record_seq;

/* ---- text output (no printf: nano libc has no float formatting) ---- */

typedef struct {
    char *buffer;
    size_t used;
    size_t capacity;
} TextLine;

static void line_str(TextLine *line, const char *text)
{
    while (*text != '\0' && line->used + 1U < line->capacity) {
        line->buffer[line->used++] = *text++;
    }
    line->buffer[line->used] = '\0';
}

static void line_int(TextLine *line, int32_t value)
{
    char digits[12];
    size_t n = 0;
    uint32_t magnitude = value < 0 ? 0U - (uint32_t)value : (uint32_t)value;
    do {
        digits[n++] = (char)('0' + magnitude % 10U);
        magnitude /= 10U;
    } while (magnitude != 0U);
    if (value < 0) {
        digits[n++] = '-';
    }
    char text[12];
    for (size_t i = 0; i < n; ++i) {
        text[i] = digits[n - 1U - i];
    }
    text[n] = '\0';
    line_str(line, text);
}

static void line_field(TextLine *line, int32_t value)
{
    line_str(line, ",");
    line_int(line, value);
}

static void line_hex8(TextLine *line, uint8_t value)
{
    const char *hex = "0123456789ABCDEF";
    char text[5] = {'0', 'x', hex[value >> 4], hex[value & 15U], '\0'};
    line_str(line, text);
}

static void queue_text(const char *text)
{
    (void)EasyTVC_USART2_Queue((const uint8_t *)text, strlen(text));
}

/* ---- ADC: battery pin, internal reference, MCU temperature ---- */

static void adc_init(void)
{
    ADC_COMMON_CCR = (ADC_COMMON_CCR & ~(3UL << 16)) | ADC_CCR_ADCPRE_DIV4 | ADC_CCR_TSVREFE;
    STM32_ADC1->CR1 = 0;                                   /* 12-bit, single channel */
    STM32_ADC1->CR2 = STM32_ADC_CR2_ADON;
    /* 480-cycle sampling: the temperature sensor needs >= 10 us */
    STM32_ADC1->SMPR1 = (7UL << ((ADC_CH_TEMP - 10U) * 3U)) | (7UL << ((ADC_CH_VREF - 10U) * 3U));
    STM32_ADC1->SMPR2 = 7UL << (ADC_CH_BATT * 3U);
    STM32_DelayMicroseconds(20U);
}

static uint16_t adc_read(uint32_t channel)
{
    STM32_ADC1->SQR1 = 0;                                  /* one conversion */
    STM32_ADC1->SQR3 = channel;
    STM32_ADC1->SR = 0;
    STM32_ADC1->CR2 |= STM32_ADC_CR2_SWSTART;
    const uint32_t start = STM32_Micros();
    while ((STM32_ADC1->SR & STM32_ADC_SR_EOC) == 0U) {
        if ((uint32_t)(STM32_Micros() - start) > 200U) {
            return 0xFFFFU;
        }
    }
    return (uint16_t)(STM32_ADC1->DR & 0xFFFU);
}

/* ---- BMI088 accelerometer die temperature (0x22/0x23, 11-bit, 0.125 C/LSB, 23 C offset) ---- */

static bool read_accel_temp(int16_t *centi_c)
{
    uint8_t raw[2];
    if (EasyTVC_BusRead(bmi088.accel, 0x22U, raw, 2) != 0) {
        return false;
    }
    int32_t value = ((int32_t)raw[0] << 3) | (raw[1] >> 5);
    if (value > 1023) {
        value -= 2048;
    }
    *centi_c = (int16_t)(value * 125 / 10 + 2300);
    return true;
}

/* ---- logging ---- */

static void log_record(SensorTestRecord *record, uint8_t type, uint32_t t_us)
{
    record->t_us = t_us;
    record->session = (uint8_t)logger.session;
    record->type = type;
    record->seq = record_seq++;
    record->marker = TELEMETRY_MARKER;
    (void)DataLogger_LogRaw(&logger, record);
}

/* ---- running statistics for the checks ---- */

typedef struct {
    float sum[6], sum_sq[6], low[6], high[6];
    uint32_t count;
    uint32_t changes;            /* samples where any raw value differed from the previous one */
    int16_t last[6];
} ImuStats;

static void imu_stats_reset(ImuStats *s)
{
    memset(s, 0, sizeof(*s));
    for (unsigned i = 0; i < 6U; ++i) {
        s->low[i] = 1e9f;
        s->high[i] = -1e9f;
    }
}

static void imu_stats_add(ImuStats *s, const int16_t raw[6])
{
    bool changed = false;
    for (unsigned i = 0; i < 6U; ++i) {
        const float v = (float)raw[i];
        s->sum[i] += v;
        s->sum_sq[i] += v * v;
        s->low[i] = fminf(s->low[i], v);
        s->high[i] = fmaxf(s->high[i], v);
        changed = changed || (s->count > 0U && raw[i] != s->last[i]);
        s->last[i] = raw[i];
    }
    s->changes += changed ? 1U : 0U;
    s->count++;
}

static uint32_t imu_checks(const ImuStats *s)
{
    if (s->count < 10U) {
        return 0U;
    }
    uint32_t checks = 0U;
    const float n = (float)s->count;
    const float ax = s->sum[0] / n / GRAVITY_LSB, ay = s->sum[1] / n / GRAVITY_LSB, az = s->sum[2] / n / GRAVITY_LSB;
    const float g = sqrtf(ax * ax + ay * ay + az * az);
    if (bmi088.accel_ok && g > 0.9f && g < 1.1f) {
        checks |= SENSOR_CHECK_ACCEL_1G;
    }
    bool still = true;
    for (unsigned i = 3U; i < 6U; ++i) {
        const float bias = fabsf(s->sum[i] / n) / GYRO_LSB_DPS;
        const float p2p = (s->high[i] - s->low[i]) / GYRO_LSB_DPS;
        still = still && bias < 5.0f && p2p < 3.0f;
    }
    if (bmi088.gyro_ok && still) {
        checks |= SENSOR_CHECK_GYRO_STILL;
    }
    if (s->changes * 2U > s->count) {
        checks |= SENSOR_CHECK_IMU_ALIVE;
    }
    return checks;
}

/* ---- result indication ---- */

static unsigned failed_group(uint32_t checks)
{
    const uint32_t imu = SENSOR_CHECK_ACCEL_ID | SENSOR_CHECK_GYRO_ID | SENSOR_CHECK_ACCEL_1G |
                         SENSOR_CHECK_GYRO_STILL | SENSOR_CHECK_IMU_ALIVE | SENSOR_CHECK_IMU_RATE |
                         SENSOR_CHECK_ACCEL_TEMP;
    const uint32_t baro = SENSOR_CHECK_BARO_ID | SENSOR_CHECK_PRESSURE | SENSOR_CHECK_BARO_TEMP |
                          SENSOR_CHECK_HUMIDITY | SENSOR_CHECK_BARO_RATE;
    if ((checks & imu) != imu) return 1U;
    if ((checks & baro) != baro) return 2U;
    if ((checks & SENSOR_CHECK_SD) == 0U) return 3U;
    if ((checks & SENSOR_CHECK_SUPPLY) == 0U) return 4U;
    return 0U;
}

static void beeps(unsigned count, uint32_t on_ms, uint32_t off_ms)
{
    for (unsigned i = 0; i < count; ++i) {
        EasyTVC_GPIO_BuzzerSet(true);
        STM32_DelayMilliseconds(on_ms);
        EasyTVC_GPIO_BuzzerSet(false);
        STM32_DelayMilliseconds(off_ms);
        STM32_WatchdogFeed();
    }
}

/* Red blinks N times, then a 1.5 s pause. */
static bool red_pattern(unsigned group, uint32_t now_ms)
{
    const uint32_t period = 400U * group + 1500U;
    const uint32_t t = now_ms % period;
    return t < 400U * group && (t % 400U) < 200U;
}

static void halt_clock_fault(void)
{
    EasyTVC_GPIO_Init();
    for (;;) {
        EasyTVC_GPIO_LedSet(2, true);
        for (volatile uint32_t i = 0; i < 400000UL; ++i) {
        }
        EasyTVC_GPIO_LedSet(2, false);
        for (volatile uint32_t i = 0; i < 400000UL; ++i) {
        }
    }
}

int main(void)
{
    if (!STM32_SystemInit()) {
        halt_clock_fault();
    }
    EasyTVC_GPIO_Init();
    EasyTVC_SPI1_Init();
    EasyTVC_SPI2_Init();
    EasyTVC_SPI3_Init();
    EasyTVC_I2C1_Init();
    EasyTVC_I2C2_Init();
    EasyTVC_PWM_Init();                 /* every servo channel held at 1500 us */
    EasyTVC_USART2_Init();
    EasyTVC_UART4_Init();
    EasyTVC_HardwareBus_Init();
    adc_init();

    /* outputs: red, green, blue, then a beep */
    for (uint8_t led = 3; led-- > 0;) {
        EasyTVC_GPIO_LedSet(led, true);
        STM32_DelayMilliseconds(250U);
        EasyTVC_GPIO_LedSet(led, false);
    }
    beeps(1U, 100U, 200U);

    EasyTVC_Bmi088Init(&bmi088, EasyTVC_GetBMI088_AccelBus(), EasyTVC_GetBMI088_GyroBus());
    EasyTVC_Bme280Init(&bme280, EasyTVC_GetBME280Bus());
    bmi088.delay_ms = STM32_DelayMilliseconds;
    bme280.delay_ms = STM32_DelayMilliseconds;

    /* chip IDs, read directly so a wrong ID is recorded rather than just "failed" */
    uint8_t accel_id = 0, gyro_id = 0, baro_id = 0;
    (void)EasyTVC_BusReadU8(bmi088.accel, EASYTVC_BMI088_ACCEL_CHIP_ID_REG, &accel_id);   /* switches it to SPI */
    (void)EasyTVC_BusReadU8(bmi088.accel, EASYTVC_BMI088_ACCEL_CHIP_ID_REG, &accel_id);
    (void)EasyTVC_BusReadU8(bmi088.gyro, EASYTVC_BMI088_GYRO_CHIP_ID_REG, &gyro_id);
    (void)EasyTVC_BusReadU8(bme280.bus, EASYTVC_BME280_CHIP_ID_REG, &baro_id);
    uint32_t id_checks = (accel_id == EASYTVC_BMI088_ACCEL_CHIP_ID ? SENSOR_CHECK_ACCEL_ID : 0U) |
                         (gyro_id == EASYTVC_BMI088_GYRO_CHIP_ID ? SENSOR_CHECK_GYRO_ID : 0U) |
                         (baro_id == EASYTVC_BME280_CHIP_ID ? SENSOR_CHECK_BARO_ID : 0U);
    bool imu_ok = false, baro_ok = false;
    int16_t accel_temp_boot = 2300;
    for (unsigned attempt = 0; attempt < 3U && !imu_ok; ++attempt) {
        imu_ok = EasyTVC_Bmi088Probe(&bmi088) && EasyTVC_Bmi088Configure(&bmi088);
    }
    for (unsigned attempt = 0; attempt < 3U && !baro_ok; ++attempt) {
        baro_ok = EasyTVC_Bme280Probe(&bme280) && EasyTVC_Bme280Configure(&bme280);
    }

    if (imu_ok) {
        (void)read_accel_temp(&accel_temp_boot);   /* so the first samples don't log 0 C */
    }
    char header[480];   /* the logger keeps up to 495 characters */
    TextLine h = {header, 0, sizeof(header)};
    line_str(&h, "EasyTVC mode=sensortest imu_hz="); line_int(&h, SENSOR_TEST_IMU_HZ);
    line_str(&h, " baro_hz="); line_int(&h, SENSOR_TEST_BARO_HZ);
    line_str(&h, " adc_hz="); line_int(&h, SENSOR_TEST_ADC_HZ);
    line_str(&h, " accel_id="); line_hex8(&h, accel_id);
    line_str(&h, " gyro_id="); line_hex8(&h, gyro_id);
    line_str(&h, " baro_id="); line_hex8(&h, baro_id);
    line_str(&h, " imu_config="); line_int(&h, imu_ok);
    line_str(&h, " imu_fail_step="); line_int(&h, bmi088.config_fail_step);
    line_str(&h, " accel_ok="); line_int(&h, bmi088.accel_ok);
    line_str(&h, " gyro_ok="); line_int(&h, bmi088.gyro_ok);
    line_str(&h, " accel_passes="); line_int(&h, bmi088.accel_passes);
    line_str(&h, " rb=");
    for (unsigned i = 0; i < sizeof(bmi088.readback); ++i) {
        if (i) line_str(&h, ",");
        line_hex8(&h, bmi088.readback[i]);
    }
    line_str(&h, " accel_range_code="); line_int(&h, bmi088.readback[3] & 3U);
    line_str(&h, " gyro_range_code="); line_int(&h, bmi088.readback[4] & 7U);
    line_str(&h, " baro_config="); line_int(&h, baro_ok);
    line_str(&h, " nose_axis="); line_int(&h, EASYTVC_NOSE_AXIS);
    line_str(&h, " vrefint_cal="); line_int(&h, VREFINT_CAL);
    (void)DataLogger_Init(&logger, header);

    STM32_WatchdogStart();

    ImuStats stats;
    imu_stats_reset(&stats);
    uint32_t imu_samples = 0, imu_errors = 0, imu_attempts = 0;
    uint32_t baro_samples = 0, baro_errors = 0, baro_attempts = 0;
    uint32_t baro_temp_ok = 0, humidity_ok = 0, pressure_ok = 0, humidity_changes = 0;
    uint32_t last_humidity = 0;
    int16_t accel_temp = accel_temp_boot;
    bool accel_temp_ok = false;
    uint16_t vdda_mv = 0, batt_pin_mv = 0;
    int16_t mcu_temp_cc = 0;
    uint16_t loop_overruns = 0, max_loop_us = 0;
    uint32_t boot_checks = 0;
    bool boot_done = false;
    uint32_t checks = id_checks;
    unsigned group = 0;

    uint32_t verdict_ms = 0;
    uint32_t still_passed = 0;
    uint32_t tick = 0;
    uint32_t next_us = STM32_Micros();
    for (;;) {
        /* 400 Hz, watchdog fed while waiting */
        while ((int32_t)(STM32_Micros() - next_us) < 0) {
            STM32_WatchdogFeed();
        }
        const uint32_t start_us = STM32_Micros();
        if ((uint32_t)(start_us - next_us) > LOOP_PERIOD_US) {
            ++loop_overruns;
            next_us = start_us;
        }
        next_us += LOOP_PERIOD_US;
        const uint32_t now_ms = STM32_Millis();
        ++tick;
        SensorTestRecord record;

        /* ---- IMU: every tick ---- */
        if (!imu_ok && tick % (5U * LOOP_HZ) == 0U) {
            imu_ok = EasyTVC_Bmi088Probe(&bmi088) && EasyTVC_Bmi088Configure(&bmi088);
        }
        /* Read whenever the chips answer, even if setup failed: the raw values (a gyro that works,
         * an accelerometer stuck at zero) say more than "no data". The checks still fail. */
        if (imu_ok || bmi088.identity_confirmed) {
            ++imu_attempts;
            const uint32_t t0 = STM32_Micros();
            if (EasyTVC_Bmi088Read(&bmi088)) {
                const int16_t raw[6] = {bmi088.accel_x, bmi088.accel_y, bmi088.accel_z,
                                        bmi088.gyro_x, bmi088.gyro_y, bmi088.gyro_z};
                imu_stats_add(&stats, raw);
                ++imu_samples;
                memset(&record, 0, sizeof(record));
                memcpy(record.u.imu.accel, raw, sizeof(record.u.imu.accel));
                memcpy(record.u.imu.gyro, &raw[3], sizeof(record.u.imu.gyro));
                record.u.imu.accel_temp = accel_temp;
                record.u.imu.read_us = (uint16_t)(STM32_Micros() - t0);
                record.u.imu.errors = (uint16_t)imu_errors;
                log_record(&record, SENSOR_REC_IMU, t0);
            } else {
                ++imu_errors;
            }
        } else {
            ++imu_attempts;
            ++imu_errors;
        }

        /* ---- barometer: 50 Hz ---- */
        if (tick % BARO_DIVIDER == 0U) {
            if (!baro_ok && tick % LOOP_HZ == 0U) {
                baro_ok = EasyTVC_Bme280Probe(&bme280) && EasyTVC_Bme280Configure(&bme280);
            }
            ++baro_attempts;
            if (baro_ok && EasyTVC_Bme280Read(&bme280)) {
                ++baro_samples;
                memset(&record, 0, sizeof(record));
                record.u.baro.pressure_pa = bme280.pressure;
                record.u.baro.temperature_cc = bme280.temperature;
                record.u.baro.humidity_q10 = bme280.humidity;
                record.u.baro.adc_p = bme280.adc_p;
                record.u.baro.adc_t = bme280.adc_t;
                record.u.baro.adc_h = bme280.adc_h;
                record.u.baro.errors = (uint16_t)baro_errors;
                log_record(&record, SENSOR_REC_BARO, STM32_Micros());
                pressure_ok += bme280.pressure >= 60000U && bme280.pressure <= 110000U;
                baro_temp_ok += bme280.temperature > -2000 && bme280.temperature < 6000;
                humidity_ok += bme280.humidity <= 100U * 1024U;
                humidity_changes += bme280.adc_h != last_humidity;
                last_humidity = bme280.adc_h;
            } else {
                ++baro_errors;
            }
        }

        /* ---- ADC and accel temperature: 10 Hz ---- */
        if (tick % ADC_DIVIDER == 0U) {
            if (imu_ok) {
                accel_temp_ok = read_accel_temp(&accel_temp) && accel_temp > -2000 && accel_temp < 7000;
            }
            memset(&record, 0, sizeof(record));
            record.u.adc.batt_raw = adc_read(ADC_CH_BATT);
            record.u.adc.vref_raw = adc_read(ADC_CH_VREF);
            record.u.adc.mcu_temp_raw = adc_read(ADC_CH_TEMP);
            const uint16_t vref = record.u.adc.vref_raw;
            vdda_mv = (vref > 0U && vref < 4096U) ? (uint16_t)(3300UL * VREFINT_CAL / vref) : 0U;
            batt_pin_mv = (uint16_t)((uint32_t)record.u.adc.batt_raw * vdda_mv / 4095UL);
            /* datasheet: V25 = 0.76 V, 2.5 mV/C */
            const int32_t sense_uv = (int32_t)((uint32_t)record.u.adc.mcu_temp_raw * vdda_mv * 1000UL / 4095UL);
            mcu_temp_cc = (int16_t)((sense_uv - 760000) / 25 + 2500);
            record.u.adc.vdda_mv = vdda_mv;
            record.u.adc.batt_pin_mv = batt_pin_mv;
            record.u.adc.mcu_temp_cc = mcu_temp_cc;
            record.u.adc.loop_overruns = loop_overruns;
            record.u.adc.max_loop_us = max_loop_us;
            log_record(&record, SENSOR_REC_ADC, STM32_Micros());
        }

        /* ---- checks, status record, serial status: 1 Hz (and the power-up verdict at 3 s) ---- */
        const bool second = tick % STATUS_DIVIDER == 0U;
        const bool verdict_now = !boot_done && tick == BOOT_TEST_TICKS;
        if (second || verdict_now) {
            checks = id_checks | imu_checks(&stats);
            if (imu_attempts > 0U && imu_samples * 100U >= imu_attempts * 99U && imu_samples > 0U) checks |= SENSOR_CHECK_IMU_RATE;
            if (baro_attempts > 0U && baro_samples * 100U >= baro_attempts * 95U && baro_samples > 0U) checks |= SENSOR_CHECK_BARO_RATE;
            if (baro_samples > 0U && pressure_ok == baro_samples) checks |= SENSOR_CHECK_PRESSURE;
            if (baro_samples > 0U && baro_temp_ok == baro_samples) checks |= SENSOR_CHECK_BARO_TEMP;
            if (baro_samples > 0U && humidity_ok == baro_samples && humidity_changes > 1U) checks |= SENSOR_CHECK_HUMIDITY;
            if (accel_temp_ok) checks |= SENSOR_CHECK_ACCEL_TEMP;
            if (logger.initialized && logger.write_errors == 0U && logger.dropped_records == 0U) checks |= SENSOR_CHECK_SD;
            if (vdda_mv >= 3000U && vdda_mv <= 3600U) checks |= SENSOR_CHECK_SUPPLY;
            /* "1 g at rest" and "gyro still" need a still board. Once either has passed (at power-up
             * or in any later still second) it stays passed, so picking the board up later does not
             * turn the LED red, and a board that was bumped during the 3 s test goes green as soon
             * as it is left alone. */
            const uint32_t still_only = SENSOR_CHECK_ACCEL_1G | SENSOR_CHECK_GYRO_STILL;
            still_passed |= checks & still_only;
            checks = (checks & ~still_only) | still_passed;

            if (verdict_now) {
                boot_done = true;
                boot_checks = checks;
                group = failed_group(checks);
                /* played by the loop below, so sampling and logging never stop */
                verdict_ms = now_ms;
            }

            memset(&record, 0, sizeof(record));
            record.u.status.checks = checks;
            record.u.status.boot_checks = boot_checks;
            record.u.status.accel_id = accel_id;
            record.u.status.gyro_id = gyro_id;
            record.u.status.baro_id = baro_id;
            record.u.status.imu_fail_step = bmi088.config_fail_step;
            record.u.status.imu_samples = imu_samples;
            record.u.status.baro_samples = baro_samples;
            record.u.status.sd_dropped = (uint16_t)logger.dropped_records;
            record.u.status.sd_write_errors = (uint16_t)logger.write_errors;
            log_record(&record, SENSOR_REC_STATUS, STM32_Micros());

            /* Q,ms,checks,boot_checks,accel_id,gyro_id,baro_id,imu_samples,imu_errors,baro_samples,
             *   baro_errors,sd,records,dropped,write_errors,session,overruns */
            char text[160];
            TextLine line = {text, 0, sizeof(text)};
            line_str(&line, "Q");
            line_field(&line, (int32_t)now_ms);
            line_field(&line, (int32_t)checks);
            line_field(&line, (int32_t)boot_checks);
            line_field(&line, accel_id);
            line_field(&line, gyro_id);
            line_field(&line, baro_id);
            line_field(&line, (int32_t)imu_samples);
            line_field(&line, (int32_t)imu_errors);
            line_field(&line, (int32_t)baro_samples);
            line_field(&line, (int32_t)baro_errors);
            line_field(&line, logger.initialized);
            line_field(&line, (int32_t)logger.record_count);
            line_field(&line, (int32_t)logger.dropped_records);
            line_field(&line, (int32_t)logger.write_errors);
            line_field(&line, (int32_t)logger.session);
            line_field(&line, loop_overruns);
            line_str(&line, "\n");
            queue_text(text);
            max_loop_us = 0;
            /* after the power-up test, judge the IMU over the most recent second only */
            if (boot_done) {
                imu_stats_reset(&stats);
            }
        }

        /* ---- serial values: 10 Hz ----
         * T,ms,ax_mg,ay_mg,az_mg,gx_cdps,gy_cdps,gz_cdps,accel_temp_cc,pressure_pa,baro_temp_cc,
         *   humidity_permille,vdda_mv,batt_pin_mv,mcu_temp_cc */
        if (tick % SERIAL_DIVIDER == 0U) {
            char text[160];
            TextLine line = {text, 0, sizeof(text)};
            line_str(&line, "T");
            line_field(&line, (int32_t)now_ms);
            line_field(&line, (int32_t)lroundf(bmi088.accel_x * 1000.0f / GRAVITY_LSB));
            line_field(&line, (int32_t)lroundf(bmi088.accel_y * 1000.0f / GRAVITY_LSB));
            line_field(&line, (int32_t)lroundf(bmi088.accel_z * 1000.0f / GRAVITY_LSB));
            line_field(&line, (int32_t)lroundf(bmi088.gyro_x * 100.0f / GYRO_LSB_DPS));
            line_field(&line, (int32_t)lroundf(bmi088.gyro_y * 100.0f / GYRO_LSB_DPS));
            line_field(&line, (int32_t)lroundf(bmi088.gyro_z * 100.0f / GYRO_LSB_DPS));
            line_field(&line, accel_temp);
            line_field(&line, (int32_t)bme280.pressure);
            line_field(&line, bme280.temperature);
            line_field(&line, (int32_t)(bme280.humidity * 10U / 1024U));
            line_field(&line, vdda_mv);
            line_field(&line, batt_pin_mv);
            line_field(&line, mcu_temp_cc);
            line_str(&line, "\n");
            queue_text(text);
        }

        DataLogger_Poll(&logger);
        if (tick % LOOP_HZ == 0U) {
            (void)DataLogger_Flush(&logger);
        }

        /* LEDs: blue = recording, green = all passed, red = failed group N (blinks N times) */
        const bool recording = logger.initialized && !logger.region_full && logger.write_errors == 0U;
        EasyTVC_GPIO_LedSet(0, recording && (now_ms % 1000U) < 500U);
        if (boot_done) {
            group = failed_group(checks);
            EasyTVC_GPIO_LedSet(1, group == 0U);
            EasyTVC_GPIO_LedSet(2, group != 0U && red_pattern(group, now_ms));
        } else {
            EasyTVC_GPIO_LedSet(1, (now_ms % 200U) < 100U);   /* testing: fast green */
            EasyTVC_GPIO_LedSet(2, false);
        }

        /* result beeps, once: two short = passed, N long = group N failed (heard without a card or
         * a serial cable, and whatever the LED wiring) */
        if (boot_done) {
            const uint32_t t = now_ms - verdict_ms;
            const unsigned first_failed = failed_group(boot_checks);
            const bool passed = first_failed == 0U;
            const uint32_t on = passed ? 80U : 400U, period = passed ? 200U : 600U, count = passed ? 2U : first_failed;
            EasyTVC_GPIO_BuzzerSet(t < count * period && (t % period) < on);
        }

        const uint32_t body_us = STM32_Micros() - start_us;
        if (body_us > max_loop_us) {
            max_loop_us = (uint16_t)(body_us > 65535U ? 65535U : body_us);
        }
    }
}
