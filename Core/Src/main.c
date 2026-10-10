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
#include "app.h"

#include <math.h>
#include <string.h>

/*
 * MONITOR-ONLY image.
 *
 * This runs the full estimation stack (attitude, altitude, flight state machine, controller) on
 * real sensor data, logs to SD and streams telemetry, but it can NOT actuate the vehicle:
 *   - physical_arm_qualified is hard-wired false (no arm sense input exists on this board yet),
 *     so the flight state machine never leaves IDLE and the controller never enables;
 *   - servos therefore sit at their configured centre pulse;
 *   - pyro outputs are compiled out (EASYTVC_PYRO_OUTPUTS_COMPILED_IN == 0).
 * Do not change those until the items in docs/software.md "Required board adapter" are done.
 */

#define LOOP_HZ              200U
#define LOOP_PERIOD_US       (1000000UL / LOOP_HZ)
#define BARO_DIVIDER         4U      /* 50 Hz */
#define LOG_DIVIDER          20U     /* 10 Hz SD records */
#define TELEMETRY_DIVIDER    20U     /* 10 Hz serial */
#define STATUS_DIVIDER       200U    /* 1 Hz serial status line */
#define FLUSH_DIVIDER        200U    /* partial-sector flush every second */

#define IMU_FAIL_RETRY_COUNT 5U
#define SENSOR_RETRY_MS      500U
#define BARO_STALE_MS        250U

#define GYRO_CAL_SAMPLES     200U
#define GYRO_CAL_MAX_P2P     0.10f   /* rad/s peak-to-peak allowed while "stationary" */
#define GYRO_CAL_ATTEMPTS    5U
#define BARO_REF_SAMPLES     20U

/* Set to 1 to make a missing/stale barometer mark the whole sample set unhealthy (which also
 * stops the attitude filter, because AppStep gates both on one flag). Left 0 for bring-up. */
#define EASYTVC_REQUIRE_BARO 0

static EasyTVCBmi088 bmi088;
static EasyTVCBme280 bme280;
static DataLogger logger;
static EasyTVCApp app;

static float gyro_bias[3];
static float baro_reference_pa;

static int16_t clamp_i16(float value)
{
    if (!(value > -32768.0f)) {
        return -32768;
    }
    if (value > 32767.0f) {
        return 32767;
    }
    return (int16_t)value;
}

/*
 * Sensor frame -> body frame (+Z toward the nose). The hwdef says ROLL_180_YAW_90 for the
 * flight orientation; that has not been bench-checked, so this is the identity on purpose.
 * Nothing downstream can act on the data (see the banner above).
 */
static void imu_to_body(const float in[3], float out[3])
{
    out[0] = in[0];
    out[1] = in[1];
    out[2] = in[2];
}

static float pressure_to_altitude_m(uint32_t pressure_pa)
{
    if (baro_reference_pa <= 0.0f) {
        return 0.0f;
    }
    return 44330.0f * (1.0f - powf((float)pressure_pa / baro_reference_pa, 0.190295f));
}

/* Blocks until the next 200 Hz tick, keeping the watchdog fed. Returns the elapsed time. */
static uint32_t wait_for_tick(uint32_t *last_us)
{
    uint32_t elapsed;
    do {
        STM32_WatchdogFeed();
        elapsed = STM32_Micros() - *last_us;
    } while (elapsed < LOOP_PERIOD_US);
    *last_us += LOOP_PERIOD_US;
    if (elapsed > 4U * LOOP_PERIOD_US) {
        *last_us = STM32_Micros();   /* we fell far behind (SD init, I2C recovery): do not burst */
    }
    return elapsed;
}

static bool imu_start(void)
{
    for (unsigned attempt = 0; attempt < 3U; ++attempt) {
        if (EasyTVC_Bmi088Probe(&bmi088) && EasyTVC_Bmi088Configure(&bmi088)) {
            return true;
        }
        STM32_DelayMilliseconds(20U);
    }
    return false;
}

static bool baro_start(void)
{
    for (unsigned attempt = 0; attempt < 3U; ++attempt) {
        if (EasyTVC_Bme280Probe(&bme280) && EasyTVC_Bme280Configure(&bme280)) {
            return true;
        }
        STM32_DelayMilliseconds(20U);
    }
    return false;
}

/* Average the gyro while the board is still to remove turn-on bias. */
static bool calibrate_gyro(uint32_t *last_us)
{
    for (unsigned attempt = 0; attempt < GYRO_CAL_ATTEMPTS; ++attempt) {
        float sum[3] = {0.0f, 0.0f, 0.0f};
        float low[3] = {1e9f, 1e9f, 1e9f};
        float high[3] = {-1e9f, -1e9f, -1e9f};
        unsigned good = 0;

        for (unsigned i = 0; i < GYRO_CAL_SAMPLES; ++i) {
            (void)wait_for_tick(last_us);
            if (!EasyTVC_Bmi088Read(&bmi088)) {
                continue;
            }
            for (unsigned axis = 0; axis < 3U; ++axis) {
                const float value = bmi088.gyro_rad_s[axis];
                sum[axis] += value;
                low[axis] = fminf(low[axis], value);
                high[axis] = fmaxf(high[axis], value);
            }
            ++good;
        }

        if (good < GYRO_CAL_SAMPLES / 2U) {
            return false;
        }
        bool still = true;
        for (unsigned axis = 0; axis < 3U; ++axis) {
            gyro_bias[axis] = sum[axis] / (float)good;
            still = still && (high[axis] - low[axis]) < GYRO_CAL_MAX_P2P;
        }
        if (still) {
            return true;
        }
    }
    gyro_bias[0] = gyro_bias[1] = gyro_bias[2] = 0.0f;
    return false;
}

static bool calibrate_baro(uint32_t *last_us)
{
    float sum = 0.0f;
    unsigned good = 0;
    for (unsigned i = 0; i < 200U && good < BARO_REF_SAMPLES; ++i) {
        (void)wait_for_tick(last_us);
        if (i % BARO_DIVIDER == 0U && EasyTVC_Bme280Read(&bme280)) {
            sum += (float)bme280.pressure;
            ++good;
        }
    }
    if (good == 0U) {
        baro_reference_pa = 0.0f;
        return false;
    }
    baro_reference_pa = sum / (float)good;
    return true;
}

static void halt_clock_fault(void)
{
    /* The PLL did not lock: the whole timing/baud/bus budget is wrong. Do not run the vehicle. */
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

static void queue_text(const char *text)
{
    (void)EasyTVC_USART2_Queue((const uint8_t *)text, strlen(text));
}

static size_t put_uint(char *out, uint32_t value)
{
    char digits[10];
    size_t count = 0;
    do {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);
    for (size_t i = 0; i < count; ++i) {
        out[i] = digits[count - 1U - i];
    }
    return count;
}

/* "S,imu,baro,sd,gyro_cal,logs,dropped,write_errors\n" */
static void send_status_line(bool imu_ok, bool baro_ok, bool gyro_cal_ok)
{
    char line[80];
    size_t n = 0;
    line[n++] = 'S';
    line[n++] = ',';
    line[n++] = imu_ok ? '1' : '0';
    line[n++] = ',';
    line[n++] = baro_ok ? '1' : '0';
    line[n++] = ',';
    line[n++] = logger.initialized ? '1' : '0';
    line[n++] = ',';
    line[n++] = gyro_cal_ok ? '1' : '0';
    line[n++] = ',';
    n += put_uint(&line[n], logger.record_count);
    line[n++] = ',';
    n += put_uint(&line[n], logger.dropped_records);
    line[n++] = ',';
    n += put_uint(&line[n], logger.write_errors);
    line[n++] = '\n';
    line[n] = '\0';
    queue_text(line);
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
    EasyTVC_PWM_Init();       /* servos start at 1500 us */
    EasyTVC_USART2_Init();
    EasyTVC_UART4_Init();
    EasyTVC_HardwareBus_Init();

    EasyTVC_Bmi088Init(&bmi088, EasyTVC_GetBMI088_AccelBus(), EasyTVC_GetBMI088_GyroBus());
    EasyTVC_Bme280Init(&bme280, EasyTVC_GetBME280Bus());
    bmi088.delay_ms = STM32_DelayMilliseconds;
    bme280.delay_ms = STM32_DelayMilliseconds;

    bool imu_ok = imu_start();
    bool baro_ok = baro_start();
    const bool logger_ok = DataLogger_Init(&logger);   /* SD init: can take ~1 s */

    EasyTVC_GPIO_LedSet(0, imu_ok);
    EasyTVC_GPIO_LedSet(1, baro_ok);
    EasyTVC_GPIO_LedSet(2, logger_ok);

    /* From here on every loop must feed the watchdog. A hang or a fault resets the board and
     * the servos return to centre. */
    STM32_WatchdogStart();

    uint32_t last_us = STM32_Micros();
    bool gyro_cal_ok = imu_ok && calibrate_gyro(&last_us);
    if (baro_ok) {
        baro_ok = calibrate_baro(&last_us);
    }

    EasyTVC_AppInit(&app, STM32_Millis());

    uint32_t tick = 0;
    uint32_t last_imu_good_ms = STM32_Millis();
    uint32_t last_baro_good_ms = STM32_Millis();
    uint32_t last_imu_retry_ms = 0;
    uint32_t last_baro_retry_ms = 0;
    unsigned imu_failures = 0;
    float altitude_m = 0.0f;
    bool led_state = false;

    for (;;) {
        const uint32_t elapsed_us = wait_for_tick(&last_us);
        const uint32_t now_ms = STM32_Millis();
        ++tick;

        /* ---- sensors ---- */
        if (imu_ok) {
            if (EasyTVC_Bmi088Read(&bmi088)) {
                last_imu_good_ms = now_ms;
                imu_failures = 0;
            } else if (++imu_failures >= IMU_FAIL_RETRY_COUNT) {
                imu_ok = false;
                EasyTVC_GPIO_LedSet(0, false);
            }
        } else if ((uint32_t)(now_ms - last_imu_retry_ms) >= SENSOR_RETRY_MS) {
            last_imu_retry_ms = now_ms;
            imu_ok = imu_start();
            imu_failures = 0;
        }

        if (tick % BARO_DIVIDER == 0U) {
            if (baro_ok) {
                if (EasyTVC_Bme280Read(&bme280)) {
                    last_baro_good_ms = now_ms;
                    altitude_m = pressure_to_altitude_m(bme280.pressure);
                }
            } else if ((uint32_t)(now_ms - last_baro_retry_ms) >= SENSOR_RETRY_MS * 2U) {
                last_baro_retry_ms = now_ms;
                if (baro_start()) {
                    baro_ok = true;
                    if (baro_reference_pa <= 0.0f) {
                        baro_reference_pa = (float)bme280.pressure;
                    }
                }
            }
        }
        const bool baro_fresh = baro_ok && (uint32_t)(now_ms - last_baro_good_ms) <= BARO_STALE_MS;

        /* ---- estimation / control (never able to arm: see banner) ---- */
        float gyro_sensor[3];
        float accel_body[3];
        float gyro_body[3];
        for (unsigned axis = 0; axis < 3U; ++axis) {
            gyro_sensor[axis] = bmi088.gyro_rad_s[axis] - gyro_bias[axis];
        }
        imu_to_body(gyro_sensor, gyro_body);
        imu_to_body(bmi088.accel_m_s2, accel_body);

        EasyTVCAppInput input = {
            .baro_altitude_m = altitude_m,
            .sensors_healthy = imu_ok && gyro_cal_ok && !bmi088.accel_saturated &&
                               !bmi088.gyro_saturated && (EASYTVC_REQUIRE_BARO ? baro_fresh : true),
            .physical_arm_qualified = false,
            .software_arm_request = false,
            .accel_is_gravity_reference = true,
            .now_ms = now_ms,
            .sensor_age_ms = (uint32_t)(now_ms - last_imu_good_ms),
            .dt_s = (float)elapsed_us * 1e-6f,
        };
        memcpy(input.gyro_rad_s, gyro_body, sizeof(input.gyro_rad_s));
        memcpy(input.accel_m_s2, accel_body, sizeof(input.accel_m_s2));

        const EasyTVCAppOutput output = EasyTVC_AppStep(&app, &input);

        /* ---- actuators: unused channels stay centred ---- */
        uint16_t pulse[4] = {1500U, 1500U, 1500U, 1500U};
        for (unsigned i = 0; i < 2U; ++i) {
            const uint8_t channel = app.settings.servo[i].channel;
            if (channel >= 1U && channel <= 4U) {
                pulse[channel - 1U] = output.control.pulse_us[i];
            }
        }
        for (uint8_t channel = 1; channel <= 4U; ++channel) {
            EasyTVC_PWM_SetChannel(channel, pulse[channel - 1U]);
        }

        /* ---- SD logging (10 Hz, never blocks the loop) ---- */
        DataLogger_Poll(&logger);
        if (tick % LOG_DIVIDER == 0U && logger.initialized) {
            TelemetryRecord record;
            record.timestamp = now_ms;
            record.accel_x = clamp_i16(accel_body[0] / 9.80665f * 1000.0f);
            record.accel_y = clamp_i16(accel_body[1] / 9.80665f * 1000.0f);
            record.accel_z = clamp_i16(accel_body[2] / 9.80665f * 1000.0f);
            record.gyro_x = clamp_i16(gyro_body[0] * 572.9578f);
            record.gyro_y = clamp_i16(gyro_body[1] * 572.9578f);
            record.gyro_z = clamp_i16(gyro_body[2] * 572.9578f);
            record.temperature = clamp_i16((float)bme280.temperature);
            record.pressure = bme280.pressure;
            record.flags = (uint8_t)(TELEMETRY_FLAG_MARKER |
                                     (imu_ok ? TELEMETRY_FLAG_IMU_OK : 0U) |
                                     (baro_fresh ? TELEMETRY_FLAG_BARO_OK : 0U) |
                                     (gyro_cal_ok ? TELEMETRY_FLAG_GYRO_CAL_OK : 0U) |
                                     (bmi088.accel_saturated ? TELEMETRY_FLAG_ACCEL_SAT : 0U) |
                                     (bmi088.gyro_saturated ? TELEMETRY_FLAG_GYRO_SAT : 0U));
            (void)DataLogger_Log(&logger, &record);
        }
        if (tick % FLUSH_DIVIDER == 0U) {
            (void)DataLogger_Flush(&logger);
        }

        /* ---- serial telemetry (queued, dropped if the line is slow) ---- */
        if (tick % TELEMETRY_DIVIDER == 0U) {
            queue_text(output.telemetry);
        }
        if (tick % STATUS_DIVIDER == 0U) {
            send_status_line(imu_ok, baro_fresh, gyro_cal_ok);
            led_state = !led_state;
            EasyTVC_GPIO_LedSet(0, led_state && imu_ok);
            EasyTVC_GPIO_LedSet(1, baro_fresh);
            EasyTVC_GPIO_LedSet(2, logger.initialized && logger.write_errors == 0U);
        }
    }
}
