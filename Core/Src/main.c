#include "board_pins.h"
#include "stm32f4xx.h"
#include "gpio_init.h"
#include "spi_driver.h"
#include "i2c_driver.h"
#include "pwm_driver.h"
#include "uart_driver.h"
#include "text_line.h"
#include "hardware_bus.h"
#include "bmi088.h"
#include "bme280.h"
#include "data_logger.h"
#include "app.h"
#include "pad_arm.h"
#include "vehicle_config.h"

#include <math.h>
#include <string.h>

/*
 * Two images are built from this file (see docs/flight-guide.md):
 *
 *   easytvc_bench  (EASYTVC_BENCH_MODE=1)  never arms. The gimbal follows the board's tilt all the
 *                  time so servo channels, directions and endpoints can be checked by hand.
 *   easytvc_flight (EASYTVC_BENCH_MODE=0)  arms itself once the rocket has stood vertical and
 *                  still on the pad, then waits for launch. TVC runs only during boost.
 *
 * Pyro outputs stay compiled out (EASYTVC_PYRO_OUTPUTS_COMPILED_IN == 0, vendor-untested).
 * Recovery is by a parachute-release servo (EASYTVC_CHUTE_CHANNEL) or motor ejection.
 */
#ifndef EASYTVC_BENCH_MODE
#define EASYTVC_BENCH_MODE 0
#endif

#if EASYTVC_SERVO_A_CHANNEL < 1 || EASYTVC_SERVO_A_CHANNEL > 4 || \
    EASYTVC_SERVO_B_CHANNEL < 1 || EASYTVC_SERVO_B_CHANNEL > 4 || \
    EASYTVC_CHUTE_CHANNEL < 0 || EASYTVC_CHUTE_CHANNEL > 4
#error "servo channels are 1..4 (chute: 0 = none)"
#endif
#if EASYTVC_SERVO_A_CHANNEL == EASYTVC_SERVO_B_CHANNEL
#error "TVC servos A and B must use different channels"
#endif
#if EASYTVC_CHUTE_CHANNEL != 0 && \
    (EASYTVC_CHUTE_CHANNEL == EASYTVC_SERVO_A_CHANNEL || EASYTVC_CHUTE_CHANNEL == EASYTVC_SERVO_B_CHANNEL)
#error "The parachute servo cannot share a channel with a TVC servo"
#endif

#define LOOP_HZ              200U
#define LOOP_PERIOD_US       (1000000UL / LOOP_HZ)
#define BARO_DIVIDER         4U      /* 50 Hz */
#define LOG_DIVIDER_PAD      20U     /* 10 Hz SD records on the pad / bench */
#define LOG_DIVIDER_FLIGHT   2U      /* 100 Hz SD records from arming until landing */
#define TELEMETRY_DIVIDER    20U     /* 10 Hz serial */
#define STATUS_DIVIDER       200U    /* 1 Hz serial status line */
#define FLUSH_DIVIDER        200U    /* queue the partial sector every second */
/* Phases keep the serial lines and the flush off the barometer's tick (phase 0, a ~1 ms I2C read). */
#define TELEMETRY_PHASE      10U
#define STATUS_PHASE         50U
#define FLUSH_PHASE          150U

#define IMU_FAIL_RETRY_COUNT 5U
#define SENSOR_RETRY_MS      500U
#define IMU_RETRY_MS         3000U   /* one IMU setup takes up to ~0.5 s: retry rarely, pad only */
#define BARO_STALE_MS        250U

#define GYRO_CAL_SAMPLES     200U
#define GYRO_CAL_MAX_P2P     0.10f   /* rad/s peak-to-peak allowed while "stationary" */
#define GYRO_CAL_ATTEMPTS    5U
#define BARO_REF_SAMPLES     20U

#define PAD_STILL_GYRO_RAD_S 0.05f   /* ~3 deg/s */
#define DISARM_CONFIRM_MS    500U
#define LOCATOR_AFTER_MS     20000U  /* start the locator beeper this long after launch */

#define GRAVITY              9.80665f
#define DEG_TO_RAD           0.01745329252f

/* Set to 1 to make a missing/stale barometer mark the whole sample set unhealthy (which also
 * stops the attitude filter, because AppStep gates both on one flag). TVC does not need it. */
#define EASYTVC_REQUIRE_BARO 0

static EasyTVCBmi088 bmi088;
static EasyTVCBme280 bme280;
static DataLogger logger;
static EasyTVCApp app;
#if EASYTVC_BENCH_MODE
static EasyTVCController bench_controller;
#endif

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
 * Sensor frame -> body frame (+Z toward the nose), chosen by EASYTVC_NOSE_AXIS. Each mapping is
 * a proper rotation (right-handed); the BMI088 accel and gyro share one axis set.
 */
static void imu_to_body(const float s[3], float b[3])
{
#if EASYTVC_NOSE_AXIS == EASYTVC_AXIS_POS_X
    b[0] = s[1]; b[1] = s[2]; b[2] = s[0];
#elif EASYTVC_NOSE_AXIS == EASYTVC_AXIS_NEG_X
    b[0] = s[1]; b[1] = -s[2]; b[2] = -s[0];
#elif EASYTVC_NOSE_AXIS == EASYTVC_AXIS_POS_Y
    b[0] = s[2]; b[1] = s[0]; b[2] = s[1];
#elif EASYTVC_NOSE_AXIS == EASYTVC_AXIS_NEG_Y
    b[0] = s[2]; b[1] = -s[0]; b[2] = -s[1];
#elif EASYTVC_NOSE_AXIS == EASYTVC_AXIS_POS_Z
    b[0] = s[0]; b[1] = s[1]; b[2] = s[2];
#elif EASYTVC_NOSE_AXIS == EASYTVC_AXIS_NEG_Z
    b[0] = s[0]; b[1] = -s[1]; b[2] = -s[2];
#else
#error "EASYTVC_NOSE_AXIS must be one of EASYTVC_AXIS_*"
#endif
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

static int32_t milli(float value)
{
    return (int32_t)lroundf(value * 1000.0f);
}

/* ---- outputs ---- */

static void beep_blocking(uint32_t ms)
{
    EasyTVC_GPIO_BuzzerSet(true);
    STM32_DelayMilliseconds(ms);
    EasyTVC_GPIO_BuzzerSet(false);
}

static void write_outputs(uint16_t pulse_a, uint16_t pulse_b, bool chute_open)
{
    uint16_t pulse[4] = {1500U, 1500U, 1500U, 1500U};
    pulse[EASYTVC_SERVO_A_CHANNEL - 1] = pulse_a;
    pulse[EASYTVC_SERVO_B_CHANNEL - 1] = pulse_b;
#if EASYTVC_CHUTE_CHANNEL != 0
    pulse[EASYTVC_CHUTE_CHANNEL - 1] = chute_open ? EASYTVC_CHUTE_OPEN_US : EASYTVC_CHUTE_CLOSED_US;
#else
    (void)chute_open;
#endif
    for (uint8_t channel = 1; channel <= 4U; ++channel) {
        EasyTVC_PWM_SetChannel(channel, pulse[channel - 1U]);
    }
}

/* Power-up self test: each TVC servo goes centre -> max -> min -> centre on its own, so a wrong
 * channel or a binding endpoint is obvious. The parachute servo is NOT moved here in the flight
 * image: it may already be holding a packed parachute. */
static void servo_self_test(void)
{
    const uint16_t a[] = {EASYTVC_SERVO_A_CENTER_US, EASYTVC_SERVO_A_MAX_US,
                          EASYTVC_SERVO_A_MIN_US, EASYTVC_SERVO_A_CENTER_US};
    const uint16_t b[] = {EASYTVC_SERVO_B_CENTER_US, EASYTVC_SERVO_B_MAX_US,
                          EASYTVC_SERVO_B_MIN_US, EASYTVC_SERVO_B_CENTER_US};
    for (unsigned i = 0; i < 4U; ++i) {
        write_outputs(a[i], EASYTVC_SERVO_B_CENTER_US, false);
        STM32_DelayMilliseconds(300U);
    }
    for (unsigned i = 0; i < 4U; ++i) {
        write_outputs(EASYTVC_SERVO_A_CENTER_US, b[i], false);
        STM32_DelayMilliseconds(300U);
    }
#if EASYTVC_BENCH_MODE && EASYTVC_CHUTE_CHANNEL != 0
    write_outputs(EASYTVC_SERVO_A_CENTER_US, EASYTVC_SERVO_B_CENTER_US, true);
    STM32_DelayMilliseconds(600U);
    write_outputs(EASYTVC_SERVO_A_CENTER_US, EASYTVC_SERVO_B_CENTER_US, false);
    STM32_DelayMilliseconds(600U);
#endif
}

static bool booted_in_motion(void)
{
    unsigned moving = 0;
    for (unsigned i = 0; i < 20U; ++i) {
        STM32_DelayMilliseconds(5U);
        if (!EasyTVC_Bmi088Read(&bmi088)) {
            continue;
        }
        const float *a = bmi088.accel_m_s2;
        const float *g = bmi088.gyro_rad_s;
        const float a_norm = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        const float g_norm = sqrtf(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
        if (a_norm < 0.7f * GRAVITY || a_norm > 1.3f * GRAVITY || g_norm > 0.6f) {
            ++moving;
        }
    }
    return moving >= 10U;   /* most of 100 ms: not a single knock */
}

/* ---- configuration ---- */

static bool apply_vehicle_config(uint32_t now_ms)
{
    EasyTVCSettings s;
    EasyTVC_SettingsDefault(&s);
    for (unsigned i = 0; i < 2U; ++i) {
        s.axis[i] = (EasyTVCPidConfig){
            .kp = EASYTVC_KP, .ki = EASYTVC_KI, .kd = EASYTVC_KD,
            .integrator_limit = EASYTVC_INTEGRATOR_LIMIT, .output_limit = 1.0f,
            .output_rate_limit = EASYTVC_OUTPUT_RATE_LIMIT,
            .derivative_cutoff_hz = EASYTVC_DERIVATIVE_CUTOFF,
        };
    }
    s.servo[0] = (EasyTVCServoSettings){
        .minimum_us = EASYTVC_SERVO_A_MIN_US, .center_us = EASYTVC_SERVO_A_CENTER_US,
        .maximum_us = EASYTVC_SERVO_A_MAX_US, .channel = EASYTVC_SERVO_A_CHANNEL,
        .direction = EASYTVC_SERVO_A_DIRECTION};
    s.servo[1] = (EasyTVCServoSettings){
        .minimum_us = EASYTVC_SERVO_B_MIN_US, .center_us = EASYTVC_SERVO_B_CENTER_US,
        .maximum_us = EASYTVC_SERVO_B_MAX_US, .channel = EASYTVC_SERVO_B_CHANNEL,
        .direction = EASYTVC_SERVO_B_DIRECTION};
    s.maximum_tilt_deg = EASYTVC_MAX_TILT_DEG;

    app.flight_config.launch_accel_g = EASYTVC_LAUNCH_ACCEL_G;
    app.flight_config.launch_confirm_ms = EASYTVC_LAUNCH_CONFIRM_MS;
    app.flight_config.tvc_enable_delay_ms = EASYTVC_TVC_ENABLE_DELAY_MS;
    app.flight_config.failsafe_apogee_ms = EASYTVC_CHUTE_BACKUP_MS;
    EasyTVC_FlightInit(&app.flight, &app.flight_config, now_ms);
    return EasyTVC_AppApplySettings(&app, &s, false);
}

static void build_header(char *buffer, size_t capacity, bool recovery_boot)
{
    EasyTVCTextLine line = {buffer, 0, capacity};
    EasyTVC_LineStr(&line, EASYTVC_BENCH_MODE ? "EasyTVC mode=bench" : "EasyTVC mode=flight");
    EasyTVC_LineStr(&line, recovery_boot ? " reset=abnormal" : " reset=normal");
    EasyTVC_LineStr(&line, " nose_axis="); EasyTVC_LineInt(&line, EASYTVC_NOSE_AXIS);
    EasyTVC_LineStr(&line, " kp_milli="); EasyTVC_LineInt(&line, milli(EASYTVC_KP));
    EasyTVC_LineStr(&line, " ki_milli="); EasyTVC_LineInt(&line, milli(EASYTVC_KI));
    EasyTVC_LineStr(&line, " kd_milli="); EasyTVC_LineInt(&line, milli(EASYTVC_KD));
    EasyTVC_LineStr(&line, " A=ch"); EasyTVC_LineInt(&line, EASYTVC_SERVO_A_CHANNEL);
    EasyTVC_LineField(&line, EASYTVC_SERVO_A_MIN_US); EasyTVC_LineField(&line, EASYTVC_SERVO_A_CENTER_US);
    EasyTVC_LineField(&line, EASYTVC_SERVO_A_MAX_US); EasyTVC_LineField(&line, EASYTVC_SERVO_A_DIRECTION);
    EasyTVC_LineStr(&line, " B=ch"); EasyTVC_LineInt(&line, EASYTVC_SERVO_B_CHANNEL);
    EasyTVC_LineField(&line, EASYTVC_SERVO_B_MIN_US); EasyTVC_LineField(&line, EASYTVC_SERVO_B_CENTER_US);
    EasyTVC_LineField(&line, EASYTVC_SERVO_B_MAX_US); EasyTVC_LineField(&line, EASYTVC_SERVO_B_DIRECTION);
    EasyTVC_LineStr(&line, " servo_hz="); EasyTVC_LineInt(&line, EASYTVC_SERVO_RATE_HZ);
    EasyTVC_LineStr(&line, " chute=ch"); EasyTVC_LineInt(&line, EASYTVC_CHUTE_CHANNEL);
    EasyTVC_LineStr(&line, " max_tilt="); EasyTVC_LineInt(&line, (int32_t)EASYTVC_MAX_TILT_DEG);
    /* IMU setup result, as in the sensor test: accel awake, gyro ok, registers the chip reported */
    EasyTVC_LineStr(&line, " accel_ok="); EasyTVC_LineInt(&line, bmi088.accel_ok);
    EasyTVC_LineStr(&line, " gyro_ok="); EasyTVC_LineInt(&line, bmi088.gyro_ok);
    EasyTVC_LineStr(&line, " accel_passes="); EasyTVC_LineInt(&line, bmi088.accel_passes);
    EasyTVC_LineStr(&line, " rb=");
    for (unsigned i = 0; i < sizeof(bmi088.readback); ++i) {
        if (i) EasyTVC_LineStr(&line, ",");
        EasyTVC_LineHex8(&line, bmi088.readback[i]);
    }
}

/* ---- status indication ---- */

typedef enum {
    BUZZ_OFF,
    BUZZ_ERROR,      /* 0.5 s beep every 2 s: do not launch */
    BUZZ_ARMED,      /* short chirp every second: ready, hands off */
    BUZZ_LOCATOR,    /* two beeps every 3 s after the flight */
} BuzzPattern;

static bool buzzer_on(BuzzPattern pattern, uint32_t now_ms)
{
    switch (pattern) {
    case BUZZ_ERROR:   return (now_ms % 2000U) < 500U;
    case BUZZ_ARMED:   return (now_ms % 1000U) < 60U;
    case BUZZ_LOCATOR: {
        const uint32_t t = now_ms % 3000U;
        return t < 150U || (t >= 300U && t < 450U);
    }
    default:           return false;
    }
}

int main(void)
{
    if (!STM32_SystemInit()) {
        EasyTVC_GPIO_HaltClockFault();
    }
    /* A watchdog/fault reset may have happened in the air. Do not move the gimbal, do not arm,
     * release the parachute. Power-cycle to clear. */
    bool recovery_boot = STM32_ResetWasAbnormal();

    EasyTVC_GPIO_Init();
    EasyTVC_SPI1_Init();
    EasyTVC_SPI2_Init();
    EasyTVC_SPI3_Init();
    EasyTVC_I2C1_Init();
    EasyTVC_I2C2_Init();
    EasyTVC_PWM_Init();       /* all channels start at 1500 us */
    write_outputs(EASYTVC_SERVO_A_CENTER_US, EASYTVC_SERVO_B_CENTER_US, recovery_boot);
    EasyTVC_USART2_Init();
    EasyTVC_UART4_Init();
    EasyTVC_HardwareBus_Init();

    EasyTVC_Bmi088Init(&bmi088, EasyTVC_GetBMI088_AccelBus(), EasyTVC_GetBMI088_GyroBus());
    EasyTVC_Bme280Init(&bme280, EasyTVC_GetBME280Bus());
    bmi088.delay_ms = STM32_DelayMilliseconds;
    bme280.delay_ms = STM32_DelayMilliseconds;

    bool imu_ok = imu_start();
    /* The servos share the board's 5 V linear regulator: a current spike can brown the MCU out,
     * and that reboot looks like a power-on. If the vehicle is accelerating or turning while we
     * boot, assume we are in the air and fail safe exactly like a watchdog reset. */
    if (imu_ok && !recovery_boot && booted_in_motion()) {
        recovery_boot = true;
        write_outputs(EASYTVC_SERVO_A_CENTER_US, EASYTVC_SERVO_B_CENTER_US, true);
    }
    if (!recovery_boot) {
        /* LED self test: red, green, blue, one at a time (checks EASYTVC_LED_ACTIVE_LOW). */
        for (uint8_t led = 3; led-- > 0;) {
            EasyTVC_GPIO_LedSet(led, true);
            STM32_DelayMilliseconds(250U);
            EasyTVC_GPIO_LedSet(led, false);
        }
        beep_blocking(100U);
        servo_self_test();
    }
    bool baro_ok = baro_start();
    char header[480];   /* the logger keeps up to 495 characters */
    build_header(header, sizeof(header), recovery_boot);
    (void)DataLogger_Init(&logger, header);   /* SD init: can take ~1 s */

    EasyTVC_GPIO_LedSet(0, imu_ok);
    EasyTVC_GPIO_LedSet(1, baro_ok);
    EasyTVC_GPIO_LedSet(2, !logger.initialized);

    /* From here on every loop must feed the watchdog. A hang or a fault resets the board into
     * the recovery boot above. */
    STM32_WatchdogStart();

    uint32_t last_us = STM32_Micros();
    bool gyro_cal_ok = imu_ok && calibrate_gyro(&last_us);
    if (baro_ok) {
        baro_ok = calibrate_baro(&last_us);
    }

    EasyTVC_AppInit(&app, STM32_Millis());
    const bool config_ok = apply_vehicle_config(STM32_Millis());
#if EASYTVC_BENCH_MODE
    EasyTVC_ControllerReset(&bench_controller);
    /* On the bench a failed still-calibration (board in hand at power-up) just leaves a small
     * uncorrected bias; do not let it hide the gimbal test. The flight image re-calibrates on the pad. */
    gyro_cal_ok = imu_ok;
#endif

    uint32_t tick = 0;
    uint32_t last_imu_good_ms = STM32_Millis();
    uint32_t last_baro_good_ms = STM32_Millis();
    uint32_t last_imu_retry_ms = 0;
    uint32_t last_baro_retry_ms = 0;
    unsigned imu_failures = 0;
    float altitude_m = 0.0f;

    EasyTVCPadArm pad;
    EasyTVC_PadArmInit(&pad);
    const EasyTVCPadArmConfig pad_config = {
        .hold_ms = EASYTVC_ARM_HOLD_MS,
        .cos_arm_tilt = cosf(EASYTVC_ARM_MAX_TILT_DEG * DEG_TO_RAD),
        .cos_disarm_tilt = cosf(EASYTVC_DISARM_TILT_DEG * DEG_TO_RAD),
        .still_gyro_rad_s = PAD_STILL_GYRO_RAD_S,
        .disarm_confirm_ms = DISARM_CONFIRM_MS,
    };

    bool launched = false;
    uint32_t launch_ms = 0;
    bool chute_open = recovery_boot;

    for (;;) {
        const uint32_t elapsed_us = wait_for_tick(&last_us);
        const uint32_t now_ms = STM32_Millis();
        ++tick;
        /* Blocking sensor recovery (tens of ms) is only allowed before arming. */
        const bool on_ground_idle = app.flight.state == EASYTVC_FLIGHT_IDLE;

        /* ---- sensors ---- */
        bool imu_fresh = false;
        if (imu_ok) {
            if (EasyTVC_Bmi088Read(&bmi088)) {
                last_imu_good_ms = now_ms;
                imu_failures = 0;
                imu_fresh = true;
            } else if (++imu_failures >= IMU_FAIL_RETRY_COUNT) {
                imu_ok = false;
            }
        } else if (on_ground_idle && (uint32_t)(now_ms - last_imu_retry_ms) >= IMU_RETRY_MS) {
            last_imu_retry_ms = now_ms;
            imu_ok = EasyTVC_Bmi088Probe(&bmi088) && EasyTVC_Bmi088Configure(&bmi088);
            imu_failures = 0;
            last_us = STM32_Micros();   /* setup took a while: do not run a burst of late ticks */
        }

        if (tick % BARO_DIVIDER == 0U) {
            if (baro_ok) {
                if (EasyTVC_Bme280Read(&bme280)) {
                    last_baro_good_ms = now_ms;
                    altitude_m = pressure_to_altitude_m(bme280.pressure);
                }
            } else if (on_ground_idle &&
                       (uint32_t)(now_ms - last_baro_retry_ms) >= SENSOR_RETRY_MS * 2U) {
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

        float gyro_sensor[3];
        float accel_body[3];
        float gyro_body[3];
        for (unsigned axis = 0; axis < 3U; ++axis) {
            gyro_sensor[axis] = bmi088.gyro_rad_s[axis] - gyro_bias[axis];
        }
        imu_to_body(gyro_sensor, gyro_body);
        imu_to_body(bmi088.accel_m_s2, accel_body);

        /* ---- pad arming: vertical + still for ARM_HOLD_MS, then hands off ---- */
        const bool flight_ready = !EASYTVC_BENCH_MODE && !recovery_boot && imu_ok && config_ok &&
                                  (!EASYTVC_REQUIRE_SD || logger.initialized);
        float new_bias[3];
        if (EasyTVC_PadArmStep(&pad, &pad_config, app.flight.state, flight_ready, imu_fresh,
                               accel_body, gyro_body, bmi088.gyro_rad_s, now_ms, new_bias)) {
            /* Re-take the gyro bias on the pad, right before flight: the power-up calibration may
             * have been done in someone's hand. */
            for (unsigned axis = 0; axis < 3U; ++axis) {
                gyro_bias[axis] = new_bias[axis];
                app.attitude.integral_error[axis] = 0.0f;
            }
            gyro_cal_ok = true;
        }

        /* ---- estimation / flight state / control ---- */
        EasyTVCAppInput input = {
            .baro_altitude_m = altitude_m,
            .sensors_healthy = imu_ok && gyro_cal_ok && !bmi088.accel_saturated &&
                               !bmi088.gyro_saturated && (EASYTVC_REQUIRE_BARO ? baro_fresh : true),
            .physical_arm_qualified = pad.armed,
            .software_arm_request = pad.armed,
            .accel_is_gravity_reference = true,
            .now_ms = now_ms,
            .sensor_age_ms = (uint32_t)(now_ms - last_imu_good_ms),
            .dt_s = (float)elapsed_us * 1e-6f,
        };
        memcpy(input.gyro_rad_s, gyro_body, sizeof(input.gyro_rad_s));
        memcpy(input.accel_m_s2, accel_body, sizeof(input.accel_m_s2));

        const EasyTVCAppOutput output = EasyTVC_AppStep(&app, &input);
        const EasyTVCFlightState state = output.flight.state;
        if (state == EASYTVC_FLIGHT_IDLE) {
            pad.armed = false;   /* the state machine refused or dropped the arm: re-qualify */
        }
        if (!launched && state == EASYTVC_FLIGHT_BOOST) {
            launched = true;
            launch_ms = now_ms;
        }

        uint16_t pulse_a = output.control.pulse_us[0];
        uint16_t pulse_b = output.control.pulse_us[1];
        bool tvc_active = output.control.enabled;
        bool ctrl_fault = output.control.fault;
#if EASYTVC_BENCH_MODE
        {
            const bool healthy = input.sensors_healthy && input.sensor_age_ms <= 20U &&
                                 input.dt_s > 0.0f && input.dt_s <= 0.02f;
            const EasyTVCControlOutput bench = EasyTVC_ControllerStep(&bench_controller,
                &app.settings, &app.attitude.orientation, now_ms, now_ms - input.sensor_age_ms,
                input.dt_s, healthy, true, true);
            pulse_a = bench.pulse_us[0];
            pulse_b = bench.pulse_us[1];
            tvc_active = bench.enabled;
            ctrl_fault = bench.fault;
            if (bench.fault) {
                /* past max tilt or a bad sample: centre, and start again next tick */
                EasyTVC_ControllerReset(&bench_controller);
                pulse_a = EASYTVC_SERVO_A_CENTER_US;
                pulse_b = EASYTVC_SERVO_B_CENTER_US;
            }
        }
#endif

        /* ---- recovery ---- */
        if (EASYTVC_CHUTE_CHANNEL != 0 && !chute_open &&
            EasyTVC_ChuteShouldOpen(launched, now_ms - launch_ms, state,
                                    output.flight.drogue_deploy_requested, EASYTVC_CHUTE_BACKUP_MS)) {
            chute_open = true;
        }

        write_outputs(pulse_a, pulse_b, chute_open);

        /* ---- SD logging (never blocks the loop) ---- */
        float tilt[2];
        EasyTVC_ControllerTilt(&app.attitude.orientation, tilt);
        DataLogger_Poll(&logger);
        const uint32_t log_divider = state == EASYTVC_FLIGHT_IDLE ? LOG_DIVIDER_PAD : LOG_DIVIDER_FLIGHT;
        if (tick % log_divider == 0U && logger.initialized) {
            TelemetryRecord record;
            record.timestamp = now_ms;
            for (unsigned axis = 0; axis < 3U; ++axis) {
                record.accel[axis] = clamp_i16(accel_body[axis] / GRAVITY * 1000.0f);
                record.gyro[axis] = clamp_i16(gyro_body[axis] * 572.9578f);
            }
            record.tilt[0] = clamp_i16(tilt[0] * 5729.578f);
            record.tilt[1] = clamp_i16(tilt[1] * 5729.578f);
            record.servo_us[0] = pulse_a;
            record.servo_us[1] = pulse_b;
            record.baro_alt_dm = clamp_i16(altitude_m * 10.0f);
            record.vspeed_cm_s = clamp_i16(app.nav.vertical_speed_m_s * 100.0f);
            record.session = (uint8_t)logger.session;
            record.state = (uint8_t)state;
            record.flags = (uint8_t)((imu_ok ? TELEMETRY_FLAG_IMU_OK : 0U) |
                                     (baro_fresh ? TELEMETRY_FLAG_BARO_OK : 0U) |
                                     (gyro_cal_ok ? TELEMETRY_FLAG_GYRO_CAL_OK : 0U) |
                                     (bmi088.accel_saturated ? TELEMETRY_FLAG_ACCEL_SAT : 0U) |
                                     (bmi088.gyro_saturated ? TELEMETRY_FLAG_GYRO_SAT : 0U) |
                                     (tvc_active ? TELEMETRY_FLAG_TVC_ACTIVE : 0U) |
                                     (ctrl_fault ? TELEMETRY_FLAG_CTRL_FAULT : 0U) |
                                     (chute_open ? TELEMETRY_FLAG_CHUTE_OPEN : 0U));
            record.marker = TELEMETRY_MARKER;
            (void)DataLogger_Log(&logger, &record);
        }
        if (tick % FLUSH_DIVIDER == FLUSH_PHASE) {
            (void)DataLogger_Flush(&logger);
        }

        /* ---- serial telemetry (queued, dropped if the line is slow) ---- */
        if (tick % TELEMETRY_DIVIDER == TELEMETRY_PHASE) {
#if EASYTVC_BENCH_MODE
            /* B,ms,tiltX_cdeg,tiltY_cdeg,pulseA,pulseB,raw_ax_mg,raw_ay_mg,raw_az_mg,body_az_mg */
            char text[120];
            EasyTVCTextLine line = {text, 0, sizeof(text)};
            EasyTVC_LineStr(&line, "B");
            EasyTVC_LineField(&line, (int32_t)now_ms);
            EasyTVC_LineField(&line, (int32_t)(tilt[0] * 5729.578f));
            EasyTVC_LineField(&line, (int32_t)(tilt[1] * 5729.578f));
            EasyTVC_LineField(&line, pulse_a);
            EasyTVC_LineField(&line, pulse_b);
            for (unsigned axis = 0; axis < 3U; ++axis) {
                EasyTVC_LineField(&line, (int32_t)(bmi088.accel_m_s2[axis] / GRAVITY * 1000.0f));
            }
            EasyTVC_LineField(&line, (int32_t)(accel_body[2] / GRAVITY * 1000.0f));
            EasyTVC_LineStr(&line, "\n");
            EasyTVC_USART2_QueueText(text);
#else
            EasyTVC_USART2_QueueText(output.telemetry);
#endif
        }

        const bool error = !imu_ok || !config_ok || recovery_boot || state == EASYTVC_FLIGHT_FAULT ||
                           (EASYTVC_REQUIRE_SD && !EASYTVC_BENCH_MODE && !logger.initialized);
        if (tick % STATUS_DIVIDER == STATUS_PHASE) {
            /* S,imu,baro,sd,gyro_cal,records,dropped,write_errors,session,state,armed,recovery,chute */
            char text[120];
            EasyTVCTextLine line = {text, 0, sizeof(text)};
            EasyTVC_LineStr(&line, "S");
            EasyTVC_LineField(&line, imu_ok);
            EasyTVC_LineField(&line, baro_fresh);
            EasyTVC_LineField(&line, logger.initialized);
            EasyTVC_LineField(&line, gyro_cal_ok);
            EasyTVC_LineField(&line, (int32_t)logger.record_count);
            EasyTVC_LineField(&line, (int32_t)logger.dropped_records);
            EasyTVC_LineField(&line, (int32_t)logger.write_errors);
            EasyTVC_LineField(&line, (int32_t)logger.session);
            EasyTVC_LineField(&line, (int32_t)state);
            EasyTVC_LineField(&line, pad.armed);
            EasyTVC_LineField(&line, recovery_boot);
            EasyTVC_LineField(&line, chute_open);
            EasyTVC_LineStr(&line, "\n");
            EasyTVC_USART2_QueueText(text);
        }

        /* LEDs: blue = alive, red = do not launch, green = arming progress / armed. */
        EasyTVC_GPIO_LedSet(0, (now_ms % 1000U) < 500U);
        EasyTVC_GPIO_LedSet(2, error);
        bool green;
        if (EASYTVC_BENCH_MODE) {
            green = tvc_active;
        } else if (state == EASYTVC_FLIGHT_ARMED) {
            green = true;
        } else if (state == EASYTVC_FLIGHT_IDLE && flight_ready) {
            green = EasyTVC_PadArmQualifying(&pad) ? (now_ms % 250U) < 125U : (now_ms % 1000U) < 500U;
        } else {
            green = false;
        }
        EasyTVC_GPIO_LedSet(1, green);

        BuzzPattern pattern = BUZZ_OFF;
        if (launched && (uint32_t)(now_ms - launch_ms) >= LOCATOR_AFTER_MS) {
            pattern = BUZZ_LOCATOR;
        } else if (error && !launched) {
            pattern = BUZZ_ERROR;
        } else if (state == EASYTVC_FLIGHT_ARMED) {
            pattern = BUZZ_ARMED;
        }
        EasyTVC_GPIO_BuzzerSet(buzzer_on(pattern, now_ms));
    }
}
