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
#include "link.h"
#include "servo_test.h"
#include "recovery_charge.h"
#include "tunables.h"
#include "pyro_test.h"
#include "adc_driver.h"
#include "version.h"
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
#if EASYTVC_PYRO_CHUTE_CHANNEL != 0 && EASYTVC_PYRO_CHUTE_CHANNEL != 1 && EASYTVC_PYRO_CHUTE_CHANNEL != 3
#error "The parachute charge goes on P1 or P3 (0 = none). P2 is the SWD pin and can't be used."
#endif

#define LOOP_HZ              200U
#define LOOP_PERIOD_US       (1000000UL / LOOP_HZ)
#define BARO_DIVIDER         4U      /* 50 Hz */
#define LOG_DIVIDER_PAD      20U     /* 10 Hz SD records on the pad / bench */
#define LOG_DIVIDER_FLIGHT   2U      /* 100 Hz SD records from arming until landing */
#define TLM_DIVIDER_PAD      20U     /* 10 Hz ground-link telemetry on the pad / bench */
#define TLM_DIVIDER_FLIGHT   10U     /* 20 Hz from launch until landing */
#define LINK_RX_BYTES_PER_TICK 64U   /* bounded work per tick: at most one request is answered */
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

static int32_t milli(float value)
{
    return (int32_t)lroundf(value * 1000.0f);
}

/* ---- ground link (USART2): telemetry out, read-only requests in (docs/wireless-telemetry.md) ---- */
static EasyTVCLink ground_link;
static EasyTVCCharge charge;          /* parachute ejection charge (recovery_charge.h) */
static EasyTVCChargeConfig charge_config;   /* from `tun` */
static EasyTVCServoTest servo_test;   /* TVC gimbal test from the ground, IDLE only (servo_test.h) */
static EasyTVCPyroTest pyro_test;     /* pyro channel ground test, IDLE + not armed only (pyro_test.h) */
static uint32_t beep_test_until_ms;   /* LED + buzzer check from the ground */
static EasyTVCPadArm pad;
static uint32_t tick;

/* Settings that can change over the link (tunables.h): `tun` is what runs, compiled from
 * vehicle_config.h unless a set sent over the link is stored on the SD card for this install. */
static EasyTVCTunables tun, tun_compiled;
static bool tun_from_link;
static uint32_t tun_sequence;
static const EasyTVCServoLimits servo_limits = {
    {EASYTVC_SERVO_A_MIN_US, EASYTVC_SERVO_B_MIN_US}, {EASYTVC_SERVO_A_MAX_US, EASYTVC_SERVO_B_MAX_US}};
static bool store_tunables(const EasyTVCTunables *t, const char **error);
static bool clear_tunables(const char **error);
static bool apply_tunables_now(const EasyTVCTunables *t);

/* What the request handler may report; refreshed every tick by the loop. */
static struct {
    EasyTVCFlightState state;
    bool armed, recovery, imu_ok, baro_ok;
} link_view;

/* one SD sector cached for LOGREAD: four 128-byte replies per card read */
static uint8_t log_cache[DATA_LOGGER_SECTOR_SIZE];
static uint32_t log_cache_sector = 0xFFFFFFFFu;

static bool link_write(void *context, const char *frame, size_t length)
{
    (void)context;
    return EasyTVC_USART2_QueueAll((const uint8_t *)frame, length);
}

static bool parse_two_uints(const char *s, uint32_t *a, uint32_t *b)
{
    uint64_t v[2] = {0, 0};
    unsigned i = 0, digits = 0;
    for (; *s; ++s) {
        if (*s == ',' && i == 0U && digits) { i = 1; digits = 0; continue; }
        if (*s < '0' || *s > '9') return false;
        v[i] = v[i] * 10U + (uint64_t)(*s - '0');
        if (v[i] > 0xFFFFFFFFull) return false;
        ++digits;
    }
    if (i != 1U || !digits) return false;
    *a = (uint32_t)v[0];
    *b = (uint32_t)v[1];
    return true;
}

/* Requests. The link module has already refused everything but HELLO/PING when the rocket is
 * not IDLE. The only commands that change anything are the TVC gimbal test (SERVO, SWEEP, CENTER,
 * TRIM): servos A/B only, IDLE only, ended and un-trimmed the moment the state leaves IDLE. */
static bool link_handle(void *context, const char *command, const char *payload, char *out, size_t capacity)
{
    (void)context;
    EasyTVCTextLine line = {out, 0, capacity};
    if (!strcmp(command, "HELLO")) {
        EasyTVC_LineStr(&line, "EASYTVC,2,");
        EasyTVC_LineStr(&line, EASYTVC_BENCH_MODE ? (EASYTVC_PYRO_OUTPUTS_COMPILED_IN ? "pyrobench," : "bench,") : "flight,");
        EasyTVC_LineStr(&line, EASYTVC_FW_VERSION ",");
        EasyTVC_LineStr(&line, EASYTVC_GIT_HASH);
        return true;
    }
    if (!strcmp(command, "PING")) {
        EasyTVC_LineStr(&line, "PONG");
        EasyTVC_LineField(&line, (int32_t)STM32_Millis());
        return true;
    }
    if (!strcmp(command, "STATUS")) {
        EasyTVC_LineInt(&line, (int32_t)link_view.state);
        EasyTVC_LineField(&line, link_view.armed);
        EasyTVC_LineField(&line, link_view.recovery);
        EasyTVC_LineField(&line, link_view.imu_ok);
        EasyTVC_LineField(&line, link_view.baro_ok);
        EasyTVC_LineField(&line, logger.initialized);
        return true;
    }
    if (!strcmp(command, "CFG")) {
        /* the settings in use: compiled into this image (vehicle_config.h), with the tunable ones
         * replaced by a set sent over the link if one is stored (last field: 1) */
        const int32_t v[] = {
            milli(tun.kp), milli(tun.ki), milli(tun.kd), milli(tun.integrator_limit),
            milli(tun.output_rate_limit), milli(tun.derivative_cutoff_hz), milli(tun.max_tilt_deg),
            EASYTVC_SERVO_A_CHANNEL, EASYTVC_SERVO_A_MIN_US, tun.center_us[0], EASYTVC_SERVO_A_MAX_US,
            EASYTVC_SERVO_A_DIRECTION, EASYTVC_SERVO_B_CHANNEL, EASYTVC_SERVO_B_MIN_US, tun.center_us[1],
            EASYTVC_SERVO_B_MAX_US, EASYTVC_SERVO_B_DIRECTION, EASYTVC_SERVO_RATE_HZ, milli(EASYTVC_LAUNCH_ACCEL_G),
            (int32_t)EASYTVC_LAUNCH_CONFIRM_MS, (int32_t)EASYTVC_TVC_ENABLE_DELAY_MS, EASYTVC_CHUTE_CHANNEL,
            EASYTVC_CHUTE_CLOSED_US, EASYTVC_CHUTE_OPEN_US, (int32_t)tun.chute_backup_ms,
            (int32_t)EASYTVC_ARM_HOLD_MS, milli(EASYTVC_ARM_MAX_TILT_DEG), milli(EASYTVC_DISARM_TILT_DEG),
            EASYTVC_NOSE_AXIS, EASYTVC_LED_ACTIVE_LOW, EASYTVC_LED_BRIGHTNESS_PCT, EASYTVC_REQUIRE_SD,
            EASYTVC_PYRO_CHUTE_CHANNEL, (int32_t)tun.pyro_delay_ms, (int32_t)tun.pyro_fire_ms,
            EASYTVC_PYRO_OUTPUTS_COMPILED_IN, milli(tun.pyro_deploy_alt_m), (int32_t)tun.pyro_backup_ms,
            tun_from_link,
        };
        for (unsigned i = 0; i < sizeof(v) / sizeof(v[0]); ++i) {
            if (i) EasyTVC_LineStr(&line, ",");
            EasyTVC_LineInt(&line, v[i]);
        }
        return line.used + 1U < capacity;
    }
    if (!strcmp(command, "SETCFG")) {
        /* new tunables (tunables.h order): checked, stored on the card, read back, then used.
         * The reply is the set now in use, so the ground can compare what it sent. */
        EasyTVCTunables t;
        const char *error = "COMMAND";
        if (!EasyTVC_TunablesParse(payload, &t)) { EasyTVC_LineStr(&line, "PAYLOAD"); return false; }
        if (!EasyTVC_TunablesValid(&t, &servo_limits)) { EasyTVC_LineStr(&line, "RANGE"); return false; }
        if (pad.armed || link_view.state != EASYTVC_FLIGHT_IDLE) { EasyTVC_LineStr(&line, "ARMED"); return false; }
        if (!store_tunables(&t, &error)) { EasyTVC_LineStr(&line, error); return false; }
        if (!apply_tunables_now(&t)) { EasyTVC_LineStr(&line, "ARMED"); return false; }
        tun_from_link = true;
        return EasyTVC_TunablesFormat(&tun, out, capacity) > 0U;
    }
    if (!strcmp(command, "CFGRESET")) {
        const char *error = "COMMAND";
        if (pad.armed || link_view.state != EASYTVC_FLIGHT_IDLE) { EasyTVC_LineStr(&line, "ARMED"); return false; }
        if (!clear_tunables(&error)) { EasyTVC_LineStr(&line, error); return false; }
        if (!apply_tunables_now(&tun_compiled)) { EasyTVC_LineStr(&line, "ARMED"); return false; }
        tun_from_link = false;
        return EasyTVC_TunablesFormat(&tun, out, capacity) > 0U;
    }
    if (!strcmp(command, "PYROARM") || !strcmp(command, "PYROFIRE")) {
        /* pyro channel ground test with a test load (pyro_test.h): IDLE and not armed only */
        uint32_t a = 0, b = 0, c = 0;
        const char *p = payload, *error = "PAYLOAD";
        uint32_t *fields[3] = {&a, &b, &c};
        unsigned n = 0;
        while (n < 3U) {
            if (*p < '0' || *p > '9') break;
            while (*p >= '0' && *p <= '9') { *fields[n] = *fields[n] * 10U + (uint32_t)(*p - '0'); if (*fields[n] > 100000U) break; ++p; }
            ++n;
            if (*p != ',') break;
            ++p;
        }
        const bool arm = command[4] == 'A';
        if (*p != '\0' || n != (arm ? 1U : 3U)) { EasyTVC_LineStr(&line, "PAYLOAD"); return false; }
        const uint32_t now = STM32_Millis();
        const bool ok = arm ? EasyTVC_PyroTestArm(&pyro_test, (uint8_t)a, link_view.state, pad.armed, now, tick, &error)
                            : EasyTVC_PyroTestFire(&pyro_test, (uint8_t)a, (uint16_t)b, c, link_view.state, pad.armed, now, &error);
        if (!ok) { EasyTVC_LineStr(&line, error); return false; }
        EasyTVC_LineInt(&line, (int32_t)a);
        EasyTVC_LineField(&line, arm ? pyro_test.code : (int32_t)c);
        EasyTVC_LineField(&line, !EASYTVC_PYRO_OUTPUTS_COMPILED_IN);      /* 1: locked, the pin stays off */
        EasyTVC_LineField(&line, EasyTVC_GPIO_ContinuityBits());
        return true;
    }
    if (!strcmp(command, "BEEP")) {
        /* LED + buzzer check: about 2 s of red/green/blue and beeps */
        beep_test_until_ms = STM32_Millis() + 2000U;
        EasyTVC_LineInt(&line, 2000);
        return true;
    }
    if (!strcmp(command, "LOGINFO")) {
        EasyTVC_LineInt(&line, logger.initialized);
        EasyTVC_LineField(&line, (int32_t)logger.session);
        EasyTVC_LineField(&line, (int32_t)DATA_LOGGER_MAX_SESSIONS);
        EasyTVC_LineField(&line, (int32_t)DATA_LOGGER_SESSION_SECTORS);
        return true;
    }
    if (!strcmp(command, "LOGREAD")) {
        uint32_t session, offset;
        if (!parse_two_uints(payload, &session, &offset)) { EasyTVC_LineStr(&line, "PAYLOAD"); return false; }
        const uint32_t region = DATA_LOGGER_SESSION_SECTORS * DATA_LOGGER_SECTOR_SIZE;
        if (!logger.initialized || session > logger.session || logger.session - session >= DATA_LOGGER_MAX_SESSIONS ||
            offset >= region || offset % EASYTVC_LINK_LOG_CHUNK != 0U) { EasyTVC_LineStr(&line, "RANGE"); return false; }
        const uint32_t sector = DATA_LOGGER_FIRST_SESSION + (session % DATA_LOGGER_MAX_SESSIONS) * DATA_LOGGER_SESSION_SECTORS +
                                offset / DATA_LOGGER_SECTOR_SIZE;
        if (sector != log_cache_sector) {
            if (!DataLogger_ReadSector(&logger, sector, log_cache)) {
                log_cache_sector = 0xFFFFFFFFu;
                EasyTVC_LineStr(&line, "BUSY");   /* the logger is writing: the ground retries */
                return false;
            }
            log_cache_sector = sector;
        }
        const uint8_t *chunk = log_cache + offset % DATA_LOGGER_SECTOR_SIZE;
        static const char hex[] = "0123456789abcdef";
        const uint32_t crc = EasyTVC_Crc32(chunk, EASYTVC_LINK_LOG_CHUNK);
        char crc_text[9];
        for (unsigned i = 0; i < 8U; ++i) crc_text[i] = hex[(crc >> (28U - 4U * i)) & 15U];
        crc_text[8] = '\0';
        EasyTVC_LineInt(&line, (int32_t)session);
        EasyTVC_LineField(&line, (int32_t)offset);
        EasyTVC_LineField(&line, (int32_t)EASYTVC_LINK_LOG_CHUNK);
        EasyTVC_LineStr(&line, ",");
        EasyTVC_LineStr(&line, crc_text);
        EasyTVC_LineStr(&line, ",");
        char b64[EASYTVC_LINK_LOG_CHUNK / 3U * 4U + 8U];
        if (!EasyTVC_Base64(chunk, EASYTVC_LINK_LOG_CHUNK, b64, sizeof(b64))) return false;
        EasyTVC_LineStr(&line, b64);
        return line.used + 1U < capacity;
    }
    if (!strcmp(command, "SERVO") || !strcmp(command, "SWEEP") || !strcmp(command, "CENTER") || !strcmp(command, "TRIM")) {
        const char *error;
        if (!EasyTVC_ServoTestCommand(&servo_test, &app.settings, link_view.state, command, payload,
                                      STM32_Millis(), &error)) {
            EasyTVC_LineStr(&line, error);
            return false;
        }
        EasyTVC_LineInt(&line, (int32_t)servo_test.mode);
        EasyTVC_LineField(&line, servo_test.trim_us[0]);
        EasyTVC_LineField(&line, servo_test.trim_us[1]);
        return true;
    }
    EasyTVC_LineStr(&line, "COMMAND");
    return false;
}

static const char *state_event(EasyTVCFlightState state)
{
    switch (state) {
    case EASYTVC_FLIGHT_IDLE: return "IDLE";
    case EASYTVC_FLIGHT_ARMED: return "ARMED";
    case EASYTVC_FLIGHT_BOOST: return "LAUNCH_DETECTED";
    case EASYTVC_FLIGHT_COAST: return "BURNOUT";
    case EASYTVC_FLIGHT_APOGEE: return "APOGEE";
    case EASYTVC_FLIGHT_DESCENT: return "DESCENT";
    case EASYTVC_FLIGHT_LANDED: return "LANDED";
    default: return "FAULT";
    }
}

/* Events go out twice (now and ~15 ms later): the link drops frames, the ground de-duplicates. */
static struct { char text[40]; uint8_t repeat_in; } pending_event;

static void send_event(uint32_t now_ms, const char *text)
{
    char payload[48];
    EasyTVCTextLine line = {payload, 0, sizeof(payload)};
    EasyTVC_LineInt(&line, (int32_t)now_ms);
    EasyTVC_LineStr(&line, ",");
    EasyTVC_LineStr(&line, text);
    (void)EasyTVC_LinkSend(&ground_link, "EVT", payload);
    memcpy(pending_event.text, payload, sizeof(pending_event.text) - 1U);
    pending_event.text[sizeof(pending_event.text) - 1U] = '\0';
    pending_event.repeat_in = 3;
}

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

static EasyTVCTunables compiled_tunables(void)
{
    return (EasyTVCTunables){
        .kp = EASYTVC_KP, .ki = EASYTVC_KI, .kd = EASYTVC_KD, .integrator_limit = EASYTVC_INTEGRATOR_LIMIT,
        .output_rate_limit = EASYTVC_OUTPUT_RATE_LIMIT, .derivative_cutoff_hz = EASYTVC_DERIVATIVE_CUTOFF,
        .max_tilt_deg = EASYTVC_MAX_TILT_DEG, .center_us = {EASYTVC_SERVO_A_CENTER_US, EASYTVC_SERVO_B_CENTER_US},
        .chute_backup_ms = EASYTVC_CHUTE_BACKUP_MS, .pyro_deploy_alt_m = EASYTVC_PYRO_DEPLOY_ALT_M,
        .pyro_delay_ms = EASYTVC_PYRO_CHUTE_DELAY_MS, .pyro_fire_ms = EASYTVC_PYRO_FIRE_MS,
        .pyro_backup_ms = EASYTVC_PYRO_BACKUP_MS,
    };
}

/* After the SD card is up: use a stored set if one was made for this install. */
static void load_tunables(void)
{
    tun_compiled = compiled_tunables();
    tun = tun_compiled;
    tun_from_link = false;
    tun_sequence = 0;
    static uint8_t sector[DATA_LOGGER_SECTOR_SIZE];
    const uint32_t base = EasyTVC_TunablesCrc(&tun_compiled);
    for (uint32_t i = 0; i < 2U; ++i) {
        EasyTVCTunables t;
        uint32_t sequence;
        if (DataLogger_ReadReserved(&logger, EASYTVC_TUNABLES_SECTOR + i, sector) &&
            EasyTVC_TunablesDecode(sector, base, &t, &sequence) && EasyTVC_TunablesValid(&t, &servo_limits) &&
            (!tun_from_link || sequence > tun_sequence)) {
            tun = t;
            tun_from_link = true;
            tun_sequence = sequence;
        }
    }
}

/* Store `t` over the older copy, read it back and check it. NO_SD, BUSY (a log write is in
 * flight: the ground retries), WRITE, READBACK. */
static bool store_tunables(const EasyTVCTunables *t, const char **error)
{
    static uint8_t sector[DATA_LOGGER_SECTOR_SIZE];
    if (!logger.initialized) { *error = "NO_SD"; return false; }
    if (logger.write_in_flight) { *error = "BUSY"; return false; }
    const uint32_t base = EasyTVC_TunablesCrc(&tun_compiled), sequence = tun_sequence + 1U;
    const uint32_t where = EASYTVC_TUNABLES_SECTOR + (sequence & 1U);
    EasyTVC_TunablesEncode(t, base, sequence, sector);
    if (!DataLogger_WriteReserved(&logger, where, sector)) { *error = "WRITE"; return false; }
    EasyTVCTunables back;
    uint32_t back_sequence;
    memset(sector, 0, sizeof(sector));
    if (!DataLogger_ReadReserved(&logger, where, sector) || !EasyTVC_TunablesDecode(sector, base, &back, &back_sequence) ||
        back_sequence != sequence || EasyTVC_TunablesCrc(&back) != EasyTVC_TunablesCrc(t)) {
        *error = "READBACK";
        return false;
    }
    tun_sequence = sequence;
    return true;
}

/* Forget a stored set: both copies cleared, back to the compiled values. */
static bool clear_tunables(const char **error)
{
    static uint8_t sector[DATA_LOGGER_SECTOR_SIZE];
    if (!logger.initialized) { *error = "NO_SD"; return false; }
    if (logger.write_in_flight) { *error = "BUSY"; return false; }
    memset(sector, 0, sizeof(sector));
    for (uint32_t i = 0; i < 2U; ++i) {
        if (!DataLogger_WriteReserved(&logger, EASYTVC_TUNABLES_SECTOR + i, sector)) { *error = "WRITE"; return false; }
    }
    tun_sequence = 0;
    return true;
}

static EasyTVCSettings settings_from(const EasyTVCTunables *t)
{
    EasyTVCSettings s;
    EasyTVC_SettingsDefault(&s);
    for (unsigned i = 0; i < 2U; ++i) {
        s.axis[i] = (EasyTVCPidConfig){
            .kp = t->kp, .ki = t->ki, .kd = t->kd,
            .integrator_limit = t->integrator_limit, .output_limit = 1.0f,
            .output_rate_limit = t->output_rate_limit,
            .derivative_cutoff_hz = t->derivative_cutoff_hz,
        };
    }
    s.servo[0] = (EasyTVCServoSettings){
        .minimum_us = EASYTVC_SERVO_A_MIN_US, .center_us = t->center_us[0],
        .maximum_us = EASYTVC_SERVO_A_MAX_US, .channel = EASYTVC_SERVO_A_CHANNEL,
        .direction = EASYTVC_SERVO_A_DIRECTION};
    s.servo[1] = (EasyTVCServoSettings){
        .minimum_us = EASYTVC_SERVO_B_MIN_US, .center_us = t->center_us[1],
        .maximum_us = EASYTVC_SERVO_B_MAX_US, .channel = EASYTVC_SERVO_B_CHANNEL,
        .direction = EASYTVC_SERVO_B_DIRECTION};
    s.maximum_tilt_deg = t->max_tilt_deg;
    return s;
}

static void charge_from(const EasyTVCTunables *t)
{
    charge_config = (EasyTVCChargeConfig){
        .channel = EASYTVC_PYRO_CHUTE_CHANNEL, .delay_ms = t->pyro_delay_ms, .backup_ms = t->pyro_backup_ms,
        .fire_ms = t->pyro_fire_ms, .deploy_alt_m = t->pyro_deploy_alt_m};
}

static bool apply_vehicle_config(uint32_t now_ms)
{
    const EasyTVCSettings s = settings_from(&tun);
    charge_from(&tun);
    app.flight_config.launch_accel_g = EASYTVC_LAUNCH_ACCEL_G;
    app.flight_config.launch_confirm_ms = EASYTVC_LAUNCH_CONFIRM_MS;
    app.flight_config.tvc_enable_delay_ms = EASYTVC_TVC_ENABLE_DELAY_MS;
    app.flight_config.failsafe_apogee_ms = tun.chute_backup_ms;
    EasyTVC_FlightInit(&app.flight, &app.flight_config, now_ms);
    return EasyTVC_AppApplySettings(&app, &s, false);
}

/* A new set from the ground, on the pad (IDLE, not armed): used from the next tick. */
static bool apply_tunables_now(const EasyTVCTunables *t)
{
    const EasyTVCSettings s = settings_from(t);
    if (!EasyTVC_AppApplySettings(&app, &s, pad.armed)) return false;   /* refuses unless IDLE and disarmed */
    tun = *t;
    charge_from(&tun);
    app.flight_config.failsafe_apogee_ms = tun.chute_backup_ms;
    app.flight.config.failsafe_apogee_ms = tun.chute_backup_ms;
    return true;
}

static void build_header(char *buffer, size_t capacity, bool recovery_boot)
{
    EasyTVCTextLine line = {buffer, 0, capacity};
    EasyTVC_LineStr(&line, EASYTVC_BENCH_MODE ? (EASYTVC_PYRO_OUTPUTS_COMPILED_IN ? "EasyTVC mode=pyrobench" : "EasyTVC mode=bench")
                                              : "EasyTVC mode=flight");
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
    EasyTVC_ADC_Init();
    EasyTVC_LinkInit(&ground_link, link_write, link_handle, 0);
    EasyTVC_ServoTestInit(&servo_test);
    EasyTVC_ChargeInit(&charge);

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
    load_tunables();                          /* settings sent over the link, if stored for this install */

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
    app.skip_text_telemetry = true;      /* the ground link sends TLM frames instead */
    const bool config_ok = apply_vehicle_config(STM32_Millis());
#if EASYTVC_BENCH_MODE
    EasyTVC_ControllerReset(&bench_controller);
    /* On the bench a failed still-calibration (board in hand at power-up) just leaves a small
     * uncorrected bias; do not let it hide the gimbal test. The flight image re-calibrates on the pad. */
    gyro_cal_ok = imu_ok;
#endif

    tick = 0;
    uint32_t last_imu_good_ms = STM32_Millis();
    uint32_t last_baro_good_ms = STM32_Millis();
    uint32_t last_imu_retry_ms = 0;
    uint32_t last_baro_retry_ms = 0;
    unsigned imu_failures = 0;
    float altitude_m = 0.0f;

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

    /* ground-link bookkeeping */
    EasyTVCFlightState reported_state = EASYTVC_FLIGHT_IDLE;
    bool reported_imu_ok = imu_ok, reported_chute = chute_open, reported_charge = false;
    uint8_t reported_test = 0;
    uint32_t loop_max_us = 0, late_ticks = 0, imu_errors = 0;
    if (recovery_boot) send_event(STM32_Millis(), "RECOVERY_BOOT");
    if (!logger.initialized) send_event(STM32_Millis(), "SD_MISSING");

    uint32_t previous_tick_us = STM32_Micros() - LOOP_PERIOD_US;
    for (;;) {
        const uint32_t elapsed_us = wait_for_tick(&last_us);
        const uint32_t tick_start_us = STM32_Micros();
        /* The time step is the real time since the previous tick's samples. (elapsed_us is measured
         * from the previous tick's *scheduled* start, so after a late tick it counted the lateness
         * twice: once then and once in the catch-up tick.) */
        const uint32_t step_us = tick_start_us - previous_tick_us;
        previous_tick_us = tick_start_us;
        const uint32_t now_ms = STM32_Millis();
        ++tick;
        if (elapsed_us > LOOP_PERIOD_US + LOOP_PERIOD_US / 2U) {
            ++late_ticks;
        }
        /* Blocking sensor recovery (tens of ms) is only allowed before arming. */
        const bool on_ground_idle = app.flight.state == EASYTVC_FLIGHT_IDLE;

        /* ---- sensors ---- */
        bool imu_fresh = false;
        if (imu_ok) {
            if (EasyTVC_Bmi088Read(&bmi088)) {
                last_imu_good_ms = now_ms;
                imu_failures = 0;
                imu_fresh = true;
            } else {
                ++imu_errors;
                if (++imu_failures >= IMU_FAIL_RETRY_COUNT) {
                    imu_ok = false;
                }
            }
        } else if (on_ground_idle && (uint32_t)(now_ms - last_imu_retry_ms) >= IMU_RETRY_MS) {
            last_imu_retry_ms = now_ms;
            imu_ok = EasyTVC_Bmi088Probe(&bmi088) && EasyTVC_Bmi088Configure(&bmi088);
            imu_failures = 0;
            last_us = STM32_Micros();   /* setup took a while: do not run a burst of late ticks */
        } else if (!on_ground_idle) {
            /* Armed or flying: the blocking re-setup is not allowed, but a plain read costs nothing.
             * Without this, 25 ms of failed reads lost the IMU (and with it TVC) for the rest of the
             * flight, even if the glitch passed. All-zero accel means the chip reset itself into
             * suspend: that is not a recovery. */
            if (EasyTVC_Bmi088Read(&bmi088) && (bmi088.accel_x != 0 || bmi088.accel_y != 0 || bmi088.accel_z != 0)) {
                imu_ok = true;
                imu_failures = 0;
                last_imu_good_ms = now_ms;
                imu_fresh = true;
            } else {
                ++imu_errors;
            }
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
            /* ... and the pad's altitude: the pressure taken at power-up may be many minutes old
             * (weather, sensor warm-up), and the deploy height is measured from here. */
            if (baro_fresh && bme280.pressure > 0U) {
                baro_reference_pa = (float)bme280.pressure;
                altitude_m = 0.0f;
                EasyTVC_NavInit(&app.nav, 0.0f);
            }
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
            .dt_s = (float)step_us * 1e-6f,
            .baro_stale = !baro_fresh,
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
        float command[2] = {output.control.command[0], output.control.command[1]};
#if EASYTVC_BENCH_MODE
        {
            const bool healthy = input.sensors_healthy && input.sensor_age_ms <= 20U &&
                                 input.dt_s > 0.0f && input.dt_s <= 0.02f;
            const EasyTVCControlOutput bench = EasyTVC_ControllerStep(&bench_controller,
                &app.settings, &app.attitude.orientation, now_ms, now_ms - input.sensor_age_ms,
                input.dt_s, healthy, true, true);
            pulse_a = bench.pulse_us[0];
            pulse_b = bench.pulse_us[1];
            command[0] = bench.command[0];
            command[1] = bench.command[1];
            tvc_active = bench.enabled;
            ctrl_fault = bench.fault;
            if (bench.fault) {
                /* past max tilt or a bad sample: centre, and start again next tick */
                EasyTVC_ControllerReset(&bench_controller);
                pulse_a = tun.center_us[0];
                pulse_b = tun.center_us[1];
            }
        }
#endif

        /* ---- recovery ---- */
        if (EASYTVC_CHUTE_CHANNEL != 0 && !chute_open &&
            EasyTVC_ChuteShouldOpen(launched, now_ms - launch_ms, state,
                                    output.flight.drogue_deploy_requested, tun.chute_backup_ms)) {
            chute_open = true;
        }

        /* ejection charge: after a launch detected in this boot only (a reboot never fires it); the
         * pin is driven only if EASYTVC_PYRO_OUTPUTS_COMPILED_IN is 1 */
        const bool reached_apogee = output.flight.drogue_deploy_requested ||
                                    (state >= EASYTVC_FLIGHT_APOGEE && state <= EASYTVC_FLIGHT_LANDED);
        const bool charge_on = EasyTVC_ChargeStep(&charge, &charge_config, launched, launch_ms, reached_apogee,
                                                  app.nav.altitude_m, baro_fresh, now_ms);
        /* the ground test drives a channel only in IDLE, never armed (pyro_test.h); both outputs are
         * written every tick, and only when EASYTVC_PYRO_OUTPUTS_COMPILED_IN does the pin move */
        const uint8_t test_channel = EasyTVC_PyroTestStep(&pyro_test, state, pad.armed, now_ms);
        EasyTVC_GPIO_PyroWrite(1U, (charge_on && charge_config.channel == 1U) || test_channel == 1U);
        EasyTVC_GPIO_PyroWrite(3U, (charge_on && charge_config.channel == 3U) || test_channel == 3U);

        /* ---- TVC gimbal test from the ground (IDLE only; parachute servo never) ---- */
        {
            uint16_t test_pulse[2] = {pulse_a, pulse_b};
            if (EasyTVC_ServoTestApply(&servo_test, &app.settings, state, now_ms, test_pulse)) {
                pulse_a = test_pulse[0];
                pulse_b = test_pulse[1];
            }
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

        /* ---- ground link: requests in, telemetry out (queued whole or dropped, never waits) ---- */
        link_view.state = state;
        link_view.armed = pad.armed;
        link_view.recovery = recovery_boot;
        link_view.imu_ok = imu_ok;
        link_view.baro_ok = baro_fresh;
        /* a request is taken only when its reply is sure to fit in the transmit ring: back-pressure
         * instead of dropped replies when the ground pipelines requests */
        for (unsigned i = 0; EasyTVC_USART2_QueueFree() > EASYTVC_LINK_MAX_FRAME && i < LINK_RX_BYTES_PER_TICK; ++i) {
            uint8_t byte;
            if (EasyTVC_USART2_Receive(&byte, 1U) != 1) break;
            if (EasyTVC_LinkFeed(&ground_link, byte, state)) break;   /* one request per tick */
        }
        if (state != reported_state) {
            send_event(now_ms, state_event(state));
            reported_state = state;
        } else if (imu_ok != reported_imu_ok) {
            send_event(now_ms, imu_ok ? "IMU_OK" : "IMU_LOST");
            reported_imu_ok = imu_ok;
        } else if (chute_open != reported_chute) {
            send_event(now_ms, "PARACHUTE_RELEASED");
            reported_chute = chute_open;
        } else if (test_channel != reported_test) {
            if (test_channel) {
                send_event(now_ms, test_channel == 1U ? (EASYTVC_PYRO_OUTPUTS_COMPILED_IN ? "PYRO_TEST_P1" : "PYRO_TEST_P1_LOCKED")
                                                      : (EASYTVC_PYRO_OUTPUTS_COMPILED_IN ? "PYRO_TEST_P3" : "PYRO_TEST_P3_LOCKED"));
            }
            reported_test = test_channel;
        } else if (charge.fired != reported_charge) {
            send_event(now_ms, EASYTVC_PYRO_OUTPUTS_COMPILED_IN ? "CHARGE_FIRED" : "CHARGE_DUE_LOCKED");
            reported_charge = charge.fired;
        } else if (pending_event.repeat_in && --pending_event.repeat_in == 0U) {
            (void)EasyTVC_LinkSend(&ground_link, "EVT", pending_event.text);
        }
        const uint32_t tlm_divider = (state >= EASYTVC_FLIGHT_BOOST && state <= EASYTVC_FLIGHT_DESCENT) ?
                                     TLM_DIVIDER_FLIGHT : TLM_DIVIDER_PAD;
        if (tick % tlm_divider == tlm_divider / 2U) {   /* phase 5 / 10: off the barometer's tick */
            const EasyTVCQuaternion q = app.attitude.orientation;
            EasyTVCLinkTlm t = {
                .ms = now_ms, .state = (uint8_t)state,
                .since_launch_ms = launched ? (int32_t)(now_ms - launch_ms) : -1,
                .pressure_pa = baro_ok ? bme280.pressure : 0U, .baro_temp_cdeg = baro_ok ? (int16_t)bme280.temperature : 0,
                .deploy_eta_ms = EasyTVC_RecoveryEtaMs(&charge, &charge_config, EASYTVC_CHUTE_CHANNEL != 0,
                                                       chute_open, launched, launch_ms, now_ms),
                .q = {q.w, q.x, q.y, q.z},
                .altitude_m = app.nav.altitude_m, .vertical_speed_m_s = app.nav.vertical_speed_m_s,
                .tilt_rad = {tilt[0], tilt[1]}, .command = {command[0], command[1]},
                .pulse_us = {pulse_a, pulse_b, (uint16_t)(EASYTVC_CHUTE_CHANNEL == 0 ? 0U :
                             chute_open ? EASYTVC_CHUTE_OPEN_US : EASYTVC_CHUTE_CLOSED_US)},
                .flags = (uint16_t)((imu_ok ? EASYTVC_TLM_IMU_OK : 0U) | (baro_fresh ? EASYTVC_TLM_BARO_OK : 0U) |
                                    (gyro_cal_ok ? EASYTVC_TLM_GYRO_CAL : 0U) |
                                    (bmi088.accel_saturated ? EASYTVC_TLM_ACCEL_SAT : 0U) |
                                    (bmi088.gyro_saturated ? EASYTVC_TLM_GYRO_SAT : 0U) |
                                    (tvc_active ? EASYTVC_TLM_TVC : 0U) | (ctrl_fault ? EASYTVC_TLM_CTRL_FAULT : 0U) |
                                    (chute_open ? EASYTVC_TLM_CHUTE : 0U) | (pad.armed ? EASYTVC_TLM_ARMED : 0U) |
                                    (recovery_boot ? EASYTVC_TLM_RECOVERY : 0U) |
                                    (servo_test.mode != EASYTVC_SERVO_TEST_OFF ||
                                     servo_test.trim_us[0] != 0 || servo_test.trim_us[1] != 0 ? EASYTVC_TLM_SERVO_TEST : 0U)),
            };
            memcpy(t.gyro_rad_s, gyro_body, sizeof(t.gyro_rad_s));
            memcpy(t.accel_m_s2, accel_body, sizeof(t.accel_m_s2));
            char payload[EASYTVC_LINK_MAX_PAYLOAD + 1U];
            if (EasyTVC_LinkFormatTlm(&t, payload, sizeof(payload))) {
                (void)EasyTVC_LinkSend(&ground_link, "TLM", payload);
            }
        }
        if (tick % TELEMETRY_DIVIDER == TELEMETRY_PHASE) {
#if EASYTVC_BENCH_MODE
            /* BEN: ms,tiltX_cdeg,tiltY_cdeg,pulseA,pulseB,raw_ax_mg,raw_ay_mg,raw_az_mg,body_az_mg */
            char text[120];
            EasyTVCTextLine line = {text, 0, sizeof(text)};
            EasyTVC_LineInt(&line, (int32_t)now_ms);
            EasyTVC_LineField(&line, (int32_t)(tilt[0] * 5729.578f));
            EasyTVC_LineField(&line, (int32_t)(tilt[1] * 5729.578f));
            EasyTVC_LineField(&line, pulse_a);
            EasyTVC_LineField(&line, pulse_b);
            for (unsigned axis = 0; axis < 3U; ++axis) {
                EasyTVC_LineField(&line, (int32_t)(bmi088.accel_m_s2[axis] / GRAVITY * 1000.0f));
            }
            EasyTVC_LineField(&line, (int32_t)(accel_body[2] / GRAVITY * 1000.0f));
            (void)EasyTVC_LinkSend(&ground_link, "BEN", text);
#endif
        }

        const bool error = !imu_ok || !config_ok || recovery_boot || state == EASYTVC_FLIGHT_FAULT ||
                           (EASYTVC_REQUIRE_SD && !EASYTVC_BENCH_MODE && !logger.initialized);
        if (tick % STATUS_DIVIDER == STATUS_PHASE) {
            const uint32_t used_kb = (logger.current_sector >= logger.end_sector - DATA_LOGGER_SESSION_SECTORS)
                ? (logger.current_sector - (logger.end_sector - DATA_LOGGER_SESSION_SECTORS)) / 2U : 0U;
            const uint16_t vdda_mv = EasyTVC_ADC_VddaMv(EasyTVC_ADC_Read(EASYTVC_ADC_CH_VREF));
            const uint16_t batt_raw = EasyTVC_ADC_Read(EASYTVC_ADC_BATT_CHANNEL);
            const EasyTVCLinkSys sys = {
                .ms = now_ms, .image = EASYTVC_BENCH_MODE ? (EASYTVC_PYRO_OUTPUTS_COMPILED_IN ? 3U : 1U) : 0U, .state = (uint8_t)state,
                .armed = pad.armed, .chute_open = chute_open,
                .batt_pin_mv = batt_raw > 4095U ? 0U : (uint16_t)((uint32_t)batt_raw * vdda_mv / 4095U),
                .vdda_mv = vdda_mv,
                .continuity = EasyTVC_GPIO_ContinuityBits(),
                /* live only with the outputs compiled in (easytvc_safety.h) */
                .pyro_armed = EASYTVC_PYRO_OUTPUTS_COMPILED_IN && EASYTVC_PYRO_CHUTE_CHANNEL != 0 && launched && !charge.fired,
                .pyro_fired = EASYTVC_PYRO_OUTPUTS_COMPILED_IN && charge.fired,
                .sd_ok = logger.initialized, .session = logger.session, .records = logger.record_count,
                .dropped = logger.dropped_records, .write_errors = logger.write_errors,
                .sd_left_kb = logger.initialized ? DATA_LOGGER_SESSION_SECTORS / 2U - used_kb : 0U,
                .loop_max_us = loop_max_us, .late_ticks = late_ticks, .imu_errors = imu_errors,
                .baro_age_ms = now_ms - last_baro_good_ms, .link_dropped = ground_link.tx_dropped,
                .link_ignored = ground_link.rx_ignored,
                .errors = (uint16_t)((!imu_ok ? EASYTVC_SYS_IMU_FAIL : 0U) | (!baro_fresh ? EASYTVC_SYS_BARO_STALE : 0U) |
                                     (!logger.initialized ? EASYTVC_SYS_SD_MISSING : 0U) |
                                     (!config_ok ? EASYTVC_SYS_CONFIG_BAD : 0U) |
                                     (recovery_boot ? EASYTVC_SYS_RECOVERY : 0U) |
                                     (state == EASYTVC_FLIGHT_FAULT ? EASYTVC_SYS_FAULT : 0U) |
                                     (logger.write_errors ? EASYTVC_SYS_SD_ERRORS : 0U) |
                                     (logger.region_full ? EASYTVC_SYS_SD_FULL : 0U) |
                                     (ground_link.tx_dropped ? EASYTVC_SYS_LINK_DROPS : 0U) |
                                     (late_ticks ? EASYTVC_SYS_LATE_TICKS : 0U)),
            };
            char payload[EASYTVC_LINK_MAX_PAYLOAD + 1U];
            if (EasyTVC_LinkFormatSys(&sys, payload, sizeof(payload))) {
                (void)EasyTVC_LinkSend(&ground_link, "SYS", payload);
            }
            loop_max_us = 0;
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
#if EASYTVC_BENCH_MODE && EASYTVC_PYRO_OUTPUTS_COMPILED_IN
        /* pyro bench image: unmistakable, so it never goes into a rocket by mistake */
        EasyTVC_GPIO_LedSet(2, (now_ms % 500U) < 250U);
#endif
        const bool beep_test = state == EASYTVC_FLIGHT_IDLE && (int32_t)(beep_test_until_ms - now_ms) > 0;
        if (beep_test) {
            /* check from the ground: red, green, blue in turn, with beeps */
            const uint32_t phase = (now_ms / 250U) % 3U;
            EasyTVC_GPIO_LedSet(2, phase == 0U);
            EasyTVC_GPIO_LedSet(1, phase == 1U);
            EasyTVC_GPIO_LedSet(0, phase == 2U);
        }

        BuzzPattern pattern = BUZZ_OFF;
        if (launched && (uint32_t)(now_ms - launch_ms) >= LOCATOR_AFTER_MS) {
            pattern = BUZZ_LOCATOR;
        } else if (error && !launched) {
            pattern = BUZZ_ERROR;
        } else if (state == EASYTVC_FLIGHT_ARMED) {
            pattern = BUZZ_ARMED;
        }
        bool buzz = beep_test ? (now_ms % 400U) < 150U : buzzer_on(pattern, now_ms);
#if EASYTVC_BENCH_MODE && EASYTVC_PYRO_OUTPUTS_COMPILED_IN
        buzz = buzz || (now_ms % 5000U) < 80U;                /* a chirp every 5 s: pyro outputs live */
#endif
        EasyTVC_GPIO_BuzzerSet(buzz);

        const uint32_t body_us = STM32_Micros() - tick_start_us;
        if (body_us > loop_max_us) {
            loop_max_us = body_us;
        }
    }
}
