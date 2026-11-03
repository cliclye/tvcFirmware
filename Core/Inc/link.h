#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "easytvc_safety.h"

/*
 * Ground link on USART2 (TX2/RX2): a USB-UART adapter or a wireless serial bridge (DroneBridge on
 * an ESP32). E1 framing, as in docs/protocol.md, one ASCII line per frame:
 *
 *     E1 <sequence> <ID> <payload> *<crc32>\n          (<= 255 bytes, no spaces inside fields)
 *
 * Board -> ground, unsolicited: ID is the message type (TLM, SYS, EVT, BEN, SNS, SCK) and
 * <sequence> counts every unsolicited frame the board tried to send, so the ground side sees
 * dropped frames as gaps. Ground -> board requests carry the ground's own sequence; the reply
 * echoes it with ID OK or ERR.
 *
 * Read-only in flight: a request other than HELLO or PING is ignored (not even answered) unless
 * the flight state is IDLE. Nothing on this link can arm, fire, open the parachute or start a
 * launch: there is no such command at all. On the pad (IDLE only) the TVC gimbal can be tested and
 * trimmed (servo_test.h), settings sent (tunables.h) and a pyro channel ground-tested with a test load
 * (pyro_test.h); all of that ends the moment the state leaves IDLE.
 *
 * Sending never blocks: a frame is queued whole into the UART ring or dropped.
 */

#define EASYTVC_LINK_MAX_FRAME   255U
#define EASYTVC_LINK_MAX_PAYLOAD 200U
#define EASYTVC_LINK_LOG_CHUNK   128U   /* bytes of SD data per LOGREAD reply (172 base64 chars) */

/* Queue the whole frame or nothing; return false if it did not fit. */
typedef bool (*EasyTVCLinkWrite)(void *context, const char *frame, size_t length);
/* Answer an allowed request. Return true with the OK payload in `out`, or false with an error
 * token (e.g. "COMMAND") in `out`. Payloads contain no spaces. */
typedef bool (*EasyTVCLinkHandler)(void *context, const char *command, const char *payload,
                                   char *out, size_t capacity);

typedef struct {
    EasyTVCLinkWrite write;
    EasyTVCLinkHandler handler;
    void *context;
    uint32_t tx_sequence;      /* unsolicited frames attempted */
    uint32_t tx_dropped;       /* ... of which the UART ring had no room for */
    uint32_t rx_frames;        /* valid requests received */
    uint32_t rx_bad;           /* lines dropped: framing, CRC, length */
    uint32_t rx_ignored;       /* valid requests refused because the rocket was not IDLE */
    char line[EASYTVC_LINK_MAX_FRAME + 1U];
    size_t used;
    bool dropping;
} EasyTVCLink;

void EasyTVC_LinkInit(EasyTVCLink *link, EasyTVCLinkWrite write, EasyTVCLinkHandler handler, void *context);

/* Build one frame into `out` (capacity >= EASYTVC_LINK_MAX_FRAME + 1). Returns its length, or 0 if
 * the id/payload are invalid or the frame would be longer than EASYTVC_LINK_MAX_FRAME. */
size_t EasyTVC_LinkEncode(char *out, size_t capacity, uint32_t sequence, const char *id, const char *payload);

/* Send an unsolicited frame. The sequence advances even when the frame is dropped. */
bool EasyTVC_LinkSend(EasyTVCLink *link, const char *id, const char *payload);

/* The flight-state gate: which requests may run in `state`. */
bool EasyTVC_LinkAllowed(const char *command, EasyTVCFlightState state);

/* Feed one received byte; `state` is the current flight state. Returns true when a complete,
 * valid request line was processed (answered or ignored). */
bool EasyTVC_LinkFeed(EasyTVCLink *link, uint8_t byte, EasyTVCFlightState state);

/* Standard base64 (with padding). Returns the encoded length, or 0 if it does not fit. */
size_t EasyTVC_Base64(const uint8_t *data, size_t length, char *out, size_t capacity);

/* ---- telemetry payloads (CSV, integers only; field order documented in docs/wireless-telemetry.md) ---- */

enum {
    EASYTVC_TLM_IMU_OK = 1u << 0, EASYTVC_TLM_BARO_OK = 1u << 1, EASYTVC_TLM_GYRO_CAL = 1u << 2,
    EASYTVC_TLM_ACCEL_SAT = 1u << 3, EASYTVC_TLM_GYRO_SAT = 1u << 4, EASYTVC_TLM_TVC = 1u << 5,
    EASYTVC_TLM_CTRL_FAULT = 1u << 6, EASYTVC_TLM_CHUTE = 1u << 7, EASYTVC_TLM_ARMED = 1u << 8,
    EASYTVC_TLM_RECOVERY = 1u << 9, EASYTVC_TLM_SERVO_TEST = 1u << 10,   /* gimbal test or trim active */
};

typedef struct {
    uint32_t ms;
    uint8_t state;
    int32_t since_launch_ms;       /* -1 before launch */
    float q[4];                    /* attitude estimate w, x, y, z (body -> world) */
    float gyro_rad_s[3];           /* body rates */
    float accel_m_s2[3];           /* body specific force */
    float altitude_m;              /* baro, above the pad */
    float vertical_speed_m_s;
    float tilt_rad[2];             /* about body X / Y */
    float command[2];              /* controller command, -1..1 (fraction of servo travel) */
    uint16_t pulse_us[3];          /* servo A, servo B, parachute servo */
    uint16_t flags;                /* EASYTVC_TLM_* */
    int32_t deploy_eta_ms;         /* until the parachute deploys: -1 before launch / none, 0 once it has */
    uint32_t pressure_pa;          /* barometer, raw (0 = no barometer) */
    int16_t baro_temp_cdeg;        /* barometer temperature, 0.01 C */
} EasyTVCLinkTlm;

/* ms,state,since_launch_ms,qw,qx,qy,qz(x1e4),gx,gy,gz(mrad/s),ax,ay,az(mg),alt_cm,vs_cm_s,
 * tilt_x,tilt_y(cdeg),cmd_a,cmd_b(milli),pulse_a,pulse_b,pulse_chute,flags,deploy_eta_ms,
 * pressure_pa,baro_temp_cdeg */
size_t EasyTVC_LinkFormatTlm(const EasyTVCLinkTlm *t, char *out, size_t capacity);

enum {
    EASYTVC_SYS_IMU_FAIL = 1u << 0, EASYTVC_SYS_BARO_STALE = 1u << 1, EASYTVC_SYS_SD_MISSING = 1u << 2,
    EASYTVC_SYS_CONFIG_BAD = 1u << 3, EASYTVC_SYS_RECOVERY = 1u << 4, EASYTVC_SYS_FAULT = 1u << 5,
    EASYTVC_SYS_SD_ERRORS = 1u << 6, EASYTVC_SYS_SD_FULL = 1u << 7, EASYTVC_SYS_LINK_DROPS = 1u << 8,
    EASYTVC_SYS_LATE_TICKS = 1u << 9,
};

typedef struct {
    uint32_t ms;
    uint8_t image;                 /* 0 flight, 1 bench, 2 sensor test */
    uint8_t state;
    uint8_t armed;
    uint8_t chute_open;
    uint16_t batt_pin_mv;          /* voltage at PA4; Studio applies the measured divider */
    uint16_t vdda_mv;              /* 3.3 V rail from the internal reference */
    uint8_t continuity;            /* pyro continuity inputs P1/P2/P3 as bits 0..2 (raw levels) */
    uint8_t pyro_armed, pyro_fired;/* always 0: pyro outputs are compiled out */
    uint8_t sd_ok;
    uint32_t session, records, dropped, write_errors;
    uint32_t sd_left_kb;           /* space left in this session's 8 MB region */
    uint32_t loop_max_us;          /* longest loop body in the last second */
    uint32_t late_ticks;           /* ticks that started more than half a period late, since boot */
    uint32_t imu_errors;
    uint32_t baro_age_ms;
    uint32_t link_dropped, link_ignored;
    uint16_t errors;               /* EASYTVC_SYS_* */
} EasyTVCLinkSys;

/* ms,image,state,armed,chute,batt_pin_mv,vdda_mv,continuity,pyro_armed,pyro_fired,sd_ok,session,
 * records,dropped,write_errors,sd_left_kb,loop_max_us,late_ticks,imu_errors,baro_age_ms,
 * link_dropped,link_ignored,errors */
size_t EasyTVC_LinkFormatSys(const EasyTVCLinkSys *s, char *out, size_t capacity);
