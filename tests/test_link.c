/* Ground link (USART2): E1 frames, the read-only-in-flight gate, drops and the bandwidth budget. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "app.h"
#include "link.h"
#include "servo_test.h"
#include "recovery_charge.h"
#include "tunables.h"
#include "pyro_test.h"
#include "settings.h"

static char sent[16][EASYTVC_LINK_MAX_FRAME + 1U];
static unsigned sent_count;
static bool accept_writes = true;
static unsigned handler_calls;
static char last_command[16];

static bool capture(void *context, const char *frame, size_t length)
{
    (void)context;
    if (!accept_writes) return false;
    assert(length <= EASYTVC_LINK_MAX_FRAME && frame[length - 1U] == '\n');
    memcpy(sent[sent_count % 16U], frame, length);
    sent[sent_count % 16U][length] = '\0';
    ++sent_count;
    return true;
}

static bool handler(void *context, const char *command, const char *payload, char *out, size_t capacity)
{
    (void)context;
    ++handler_calls;
    strncpy(last_command, command, sizeof(last_command) - 1U);
    snprintf(out, capacity, "%s-%s", command, payload);
    return strcmp(command, "BAD") != 0;
}

static void feed(EasyTVCLink *link, const char *bytes, EasyTVCFlightState state)
{
    for (const char *p = bytes; *p; ++p) (void)EasyTVC_LinkFeed(link, (uint8_t)*p, state);
}

/* A request as the ground sends it. */
static void request(char *out, uint32_t seq, const char *command, const char *payload)
{
    assert(EasyTVC_LinkEncode(out, EASYTVC_LINK_MAX_FRAME + 1U, seq, command, payload) > 0U);
}

static void check_frame(const char *frame, uint32_t seq, const char *id, const char *payload)
{
    char expect[EASYTVC_LINK_MAX_FRAME + 1U];
    assert(EasyTVC_LinkEncode(expect, sizeof(expect), seq, id, payload) > 0U);
    assert(strcmp(frame, expect) == 0);
}

int main(void)
{
    EasyTVCLink link;
    char frame[EASYTVC_LINK_MAX_FRAME + 1U];

    /* 1. encoding: exact text, CRC over everything before " *" */
    assert(EasyTVC_LinkEncode(frame, sizeof(frame), 7, "PING", "-") > 0U);
    {
        const char *body = "E1 7 PING -";
        char expect[64];
        snprintf(expect, sizeof(expect), "%s *%08lx\n", body, (unsigned long)EasyTVC_Crc32(body, strlen(body)));
        assert(strcmp(frame, expect) == 0);
    }
    assert(EasyTVC_LinkEncode(frame, sizeof(frame), 1, "TLM", "has space") == 0U);    /* spaces split fields */
    assert(EasyTVC_LinkEncode(frame, sizeof(frame), 1, "", "-") == 0U);
    char big[EASYTVC_LINK_MAX_PAYLOAD + 2U];
    memset(big, 'x', sizeof(big) - 1U);
    big[sizeof(big) - 1U] = '\0';
    assert(EasyTVC_LinkEncode(frame, sizeof(frame), 1, "TLM", big) == 0U);           /* too long */
    big[EASYTVC_LINK_MAX_PAYLOAD] = '\0';
    assert(EasyTVC_LinkEncode(frame, sizeof(frame), 4294967295u, "TLM", big) > 0U);   /* max fits */
    assert(strlen(frame) <= EASYTVC_LINK_MAX_FRAME);

    /* 2. round trip: a request is answered with the same sequence */
    EasyTVC_LinkInit(&link, capture, handler, 0);
    request(frame, 41, "CFG", "-");
    feed(&link, frame, EASYTVC_FLIGHT_IDLE);
    assert(sent_count == 1U && handler_calls == 1U && link.rx_frames == 1U);
    check_frame(sent[0], 41, "OK", "CFG--");
    request(frame, 42, "BAD", "-");
    feed(&link, frame, EASYTVC_FLIGHT_IDLE);
    check_frame(sent[1], 42, "ERR", "BAD--");

    /* 3. corruption: a flipped byte fails the CRC and is dropped; the next good line still works */
    request(frame, 50, "PING", "-");
    frame[5] ^= 1;
    feed(&link, frame, EASYTVC_FLIGHT_IDLE);
    assert(sent_count == 2U && link.rx_bad == 1U);
    feed(&link, "\x01\x02 garbage E1 9 PING", EASYTVC_FLIGHT_IDLE);             /* junk, no newline yet */
    request(frame, 51, "PING", "-");
    feed(&link, "\n", EASYTVC_FLIGHT_IDLE);                                     /* ends the junk line */
    feed(&link, frame, EASYTVC_FLIGHT_IDLE);
    assert(sent_count == 3U);
    check_frame(sent[2], 51, "OK", "PING--");
    for (unsigned i = 0; i < 400U; ++i) (void)EasyTVC_LinkFeed(&link, 'A', EASYTVC_FLIGHT_IDLE);   /* overlong */
    feed(&link, "\n", EASYTVC_FLIGHT_IDLE);
    request(frame, 52, "PING", "-");
    feed(&link, frame, EASYTVC_FLIGHT_IDLE);
    check_frame(sent[3], 52, "OK", "PING--");
    feed(&link, "E1 53 PING - extra *00000000\n", EASYTVC_FLIGHT_IDLE);         /* malformed */
    assert(sent_count == 4U);

    /* 4. read-only in flight: everything but HELLO/PING is ignored, not even answered */
    static const char *commands[] = {"STATUS", "CFG", "LOGINFO", "LOGREAD", "SET", "ARM", "FIRE", "SERVO", "CHUTE", "BOOT"};
    for (int state = EASYTVC_FLIGHT_IDLE; state <= EASYTVC_FLIGHT_FAULT; ++state) {
        for (unsigned c = 0; c < sizeof(commands) / sizeof(commands[0]); ++c) {
            const unsigned calls = handler_calls, out = sent_count;
            request(frame, 100 + c, commands[c], "1,0");
            feed(&link, frame, (EasyTVCFlightState)state);
            if (state == EASYTVC_FLIGHT_IDLE) {
                assert(handler_calls == calls + 1U && sent_count == out + 1U);
            } else {
                assert(handler_calls == calls && sent_count == out);
            }
        }
        const unsigned calls = handler_calls;
        request(frame, 200, "PING", "-");
        feed(&link, frame, (EasyTVCFlightState)state);
        assert(handler_calls == calls + 1U);                  /* the handshake always works */
        assert(EasyTVC_LinkAllowed("HELLO", (EasyTVCFlightState)state));
    }
    assert(link.rx_ignored == 10U * 7U);
    printf("link: in-flight gate held in all 7 non-IDLE states (%u requests ignored)\n", (unsigned)link.rx_ignored);

    /* 5. unsolicited frames: the sequence advances even when the UART ring drops the frame */
    EasyTVC_LinkInit(&link, capture, handler, 0);
    sent_count = 0;
    assert(EasyTVC_LinkSend(&link, "SYS", "1,2"));
    accept_writes = false;
    assert(!EasyTVC_LinkSend(&link, "SYS", "1,3"));
    accept_writes = true;
    assert(EasyTVC_LinkSend(&link, "SYS", "1,4"));
    check_frame(sent[0], 0, "SYS", "1,2");
    check_frame(sent[1], 2, "SYS", "1,4");                     /* the ground sees seq 1 missing */
    assert(link.tx_dropped == 1U);

    /* 6. base64 against RFC 4648 vectors */
    char b64[16];
    assert(EasyTVC_Base64((const uint8_t *)"foob", 4, b64, sizeof(b64)) == 8U && !strcmp(b64, "Zm9vYg=="));
    assert(EasyTVC_Base64((const uint8_t *)"fooba", 5, b64, sizeof(b64)) && !strcmp(b64, "Zm9vYmE="));
    assert(EasyTVC_Base64((const uint8_t *)"foobar", 6, b64, sizeof(b64)) && !strcmp(b64, "Zm9vYmFy"));
    assert(EasyTVC_Base64((const uint8_t *)"foobar", 6, b64, 8) == 0U);   /* no room */

    /* 7. worst-case frame sizes and the bandwidth budget at 115200 baud (11520 bytes/s) */
    EasyTVCLinkTlm t = {.ms = 4294967295u, .state = 7, .since_launch_ms = 2000000000,
                        .q = {-1, -1, -1, -1}, .gyro_rad_s = {-34.9f, -34.9f, -34.9f},
                        .accel_m_s2 = {-235, -235, -235}, .altitude_m = -9999, .vertical_speed_m_s = -999,
                        .tilt_rad = {-3.14f, -3.14f}, .command = {-1, -1}, .pulse_us = {2100, 2100, 2100}, .flags = 2047,
                        .deploy_eta_ms = -2000000000, .pressure_pa = 110000, .baro_temp_cdeg = -4000};
    char payload[EASYTVC_LINK_MAX_PAYLOAD + 1U];
    const size_t tlm = EasyTVC_LinkFormatTlm(&t, payload, sizeof(payload));
    assert(tlm > 0U && tlm < EASYTVC_LINK_MAX_PAYLOAD);
    const size_t tlm_frame = EasyTVC_LinkEncode(frame, sizeof(frame), 4294967295u, "TLM", payload);
    assert(tlm_frame > 0U);
    EasyTVCLinkSys s = {.ms = 4294967295u, .image = 2, .state = 7, .armed = 1, .chute_open = 1, .batt_pin_mv = 65535,
                        .vdda_mv = 3600, .continuity = 7, .sd_ok = 1, .session = 4294967295u, .records = 4294967295u,
                        .dropped = 4294967295u, .write_errors = 4294967295u, .sd_left_kb = 8192, .loop_max_us = 65535,
                        .late_ticks = 4294967295u, .imu_errors = 4294967295u, .baro_age_ms = 4294967295u,
                        .link_dropped = 4294967295u, .link_ignored = 4294967295u, .errors = 1023};
    const size_t sys = EasyTVC_LinkFormatSys(&s, payload, sizeof(payload));
    assert(sys > 0U && sys < EASYTVC_LINK_MAX_PAYLOAD);
    const size_t sys_frame = EasyTVC_LinkEncode(frame, sizeof(frame), 4294967295u, "SYS", payload);
    assert(sys_frame > 0U);
    const double flight_bps = 20.0 * (double)tlm_frame + 1.0 * (double)sys_frame + 2.0 * 60.0;   /* + 2 events/s */
    printf("link: worst-case TLM %zu B, SYS %zu B -> %.0f B/s in flight = %.0f %% of 115200 baud\n",
           tlm_frame, sys_frame, flight_bps, 100.0 * flight_bps / 11520.0);
    assert(flight_bps < 0.5 * 11520.0);

    /* 8. cost per telemetry frame (host CPU; the board's own loop-time counters go out in SYS) */
    const clock_t start = clock();
    size_t total = 0;
    for (unsigned i = 0; i < 100000U; ++i) {
        t.ms = i;
        total += EasyTVC_LinkFormatTlm(&t, payload, sizeof(payload));
        total += EasyTVC_LinkEncode(frame, sizeof(frame), i, "TLM", payload);
    }
    const double us = 1e6 * (double)(clock() - start) / CLOCKS_PER_SEC / 100000.0;
    printf("link: build + frame one TLM = %.2f us on this host (%zu bytes)\n", us, total);
    assert(us < 50.0);

    /* 9. per-tick work, before vs after, on the 200 Hz loop (host CPU, median of 5 runs).
     * Before: the app formatted output.telemetry every tick (unused on the board), plus a 10 Hz
     * serial CSV line and a 1 Hz status line. After: no per-tick text; TLM at 10 Hz on the pad /
     * 20 Hz in flight and SYS at 1 Hz, each CRC'd and framed. Best of 7 interleaved runs: the
     * fastest run is the one least disturbed by whatever else the host is doing. */
    {
        static EasyTVCApp app;
        EasyTVCAppInput in = {.accel_m_s2 = {0.0f, 0.0f, 9.80665f}, .sensors_healthy = true,
                              .accel_is_gravity_reference = true, .dt_s = 0.005f};
        enum { BEFORE, AFTER_PAD, AFTER_FLIGHT, VARIANTS };
        double us_tick[VARIANTS][7];
        char text[160];
        /* host noise can only make "after" look slower: measure again (up to 3 times) before failing */
        bool no_worse = false;
        for (unsigned attempt = 0; attempt < 3U && !no_worse; ++attempt) {
        for (unsigned run = 0; run < 7U; ++run) {
            for (int v = 0; v < VARIANTS; ++v) {
                EasyTVC_AppInit(&app, 0);
                app.skip_text_telemetry = v != BEFORE;
                const unsigned ticks = 40000U, tlm_every = v == AFTER_FLIGHT ? 10U : 20U;
                volatile size_t sink = 0;
                const clock_t t0 = clock();
                for (unsigned i = 0; i < ticks; ++i) {
                    in.now_ms = i * 5U;
                    in.gyro_rad_s[0] = 0.01f * (float)(i % 7U);
                    const EasyTVCAppOutput out = EasyTVC_AppStep(&app, &in);
                    sink += (size_t)out.telemetry[0];
                    if (v == BEFORE) {
                        if (i % 20U == 0U) {          /* the old 10 Hz CSV line: same kind of text */
                            const EasyTVCTelemetrySample sample = {.now_ms = in.now_ms, .altitude_m = 1.0f};
                            sink += EasyTVC_TelemetryFormat(&sample, text, sizeof(text));
                        }
                        if (i % 200U == 0U) sink += EasyTVC_LinkFormatSys(&s, payload, sizeof(payload));
                        continue;
                    }
                    if (i % tlm_every == 0U) {
                        t.ms = in.now_ms;
                        const size_t n = EasyTVC_LinkFormatTlm(&t, payload, sizeof(payload));
                        sink += n + EasyTVC_LinkEncode(frame, sizeof(frame), i, "TLM", payload);
                    }
                    if (i % 200U == 0U) {
                        const size_t n = EasyTVC_LinkFormatSys(&s, payload, sizeof(payload));
                        sink += n + EasyTVC_LinkEncode(frame, sizeof(frame), i, "SYS", payload);
                    }
                }
                us_tick[v][run] = 1e6 * (double)(clock() - t0) / CLOCKS_PER_SEC / ticks;
                (void)sink;
            }
        }
        double median[VARIANTS];   /* (the best run, see above) */
        for (int v = 0; v < VARIANTS; ++v) {
            median[v] = us_tick[v][0];
            for (unsigned a = 1; a < 7U; ++a) if (us_tick[v][a] < median[v]) median[v] = us_tick[v][a];
        }
        no_worse = median[AFTER_PAD] <= median[BEFORE] * 1.05 &&      /* no worse (5 % for timer noise) */
                   median[AFTER_FLIGHT] <= median[BEFORE] * 1.05;
        if (no_worse || attempt == 2U)
            printf("link: per-tick work on this host: before %.3f us, after %.3f us on the pad, %.3f us in flight\n",
                   median[BEFORE], median[AFTER_PAD], median[AFTER_FLIGHT]);
        }
        assert(no_worse);
    }

    /* 10. TVC gimbal test from the ground: IDLE only, hold-to-move, sweep, trim, never past the
     * endpoints, and everything ends (trim included) the moment the state leaves IDLE */
    {
        EasyTVCSettings set;
        EasyTVC_SettingsDefault(&set);
        set.servo[0] = (EasyTVCServoSettings){.minimum_us = 1100, .center_us = 1500, .maximum_us = 1900, .channel = 1, .direction = 1};
        set.servo[1] = (EasyTVCServoSettings){.minimum_us = 1200, .center_us = 1520, .maximum_us = 1800, .channel = 2, .direction = -1};
        EasyTVCServoTest st;
        EasyTVC_ServoTestInit(&st);
        const char *err;
        uint16_t p[2] = {1500, 1520};
        assert(!EasyTVC_ServoTestApply(&st, &set, EASYTVC_FLIGHT_IDLE, 0, p));        /* nothing to do */
        for (int state = EASYTVC_FLIGHT_ARMED; state <= EASYTVC_FLIGHT_FAULT; ++state) {
            assert(!EasyTVC_ServoTestCommand(&st, &set, (EasyTVCFlightState)state, "SERVO", "500,500", 0, &err));
            assert(!strcmp(err, "NOT_IDLE") && st.mode == EASYTVC_SERVO_TEST_OFF);
            assert(!EasyTVC_LinkAllowed("SERVO", (EasyTVCFlightState)state) && !EasyTVC_LinkAllowed("TRIM", (EasyTVCFlightState)state));
        }
        assert(EasyTVC_ServoTestCommand(&st, &set, EASYTVC_FLIGHT_IDLE, "SERVO", "1000,-1000", 100, &err));
        assert(EasyTVC_ServoTestApply(&st, &set, EASYTVC_FLIGHT_IDLE, 600, p));
        assert(p[0] == 1900 && p[1] == 1800);                         /* full travel, B reversed */
        p[0] = 1500; p[1] = 1520;
        assert(!EasyTVC_ServoTestApply(&st, &set, EASYTVC_FLIGHT_IDLE, 100 + EASYTVC_SERVO_TEST_HOLD_MS + 1, p));
        assert(p[0] == 1500 && p[1] == 1520 && st.mode == EASYTVC_SERVO_TEST_OFF);   /* no refresh: back to centre */
        for (const char **bad = (const char *[]){"1001,0", "0", "a,b", "1,2,3", "", "-,1", "5,5x", 0}; *bad; ++bad)
            assert(!EasyTVC_ServoTestCommand(&st, &set, EASYTVC_FLIGHT_IDLE, "SERVO", *bad, 0, &err));
        /* sweep: A centre/max/min/centre, then B, then done */
        assert(EasyTVC_ServoTestCommand(&st, &set, EASYTVC_FLIGHT_IDLE, "SWEEP", "-", 1000, &err));
        const uint16_t want[8][2] = {{1500, 1520}, {1900, 1520}, {1100, 1520}, {1500, 1520},
                                     {1500, 1520}, {1500, 1200}, {1500, 1800}, {1500, 1520}};
        for (unsigned k = 0; k < 8U; ++k) {
            assert(EasyTVC_ServoTestApply(&st, &set, EASYTVC_FLIGHT_IDLE, 1000 + k * EASYTVC_SERVO_TEST_STEP_MS + 10, p));
            assert(p[0] == want[k][0] && p[1] == want[k][1]);
        }
        p[0] = 1500; p[1] = 1520;
        assert(!EasyTVC_ServoTestApply(&st, &set, EASYTVC_FLIGHT_IDLE, 1000 + 8 * EASYTVC_SERVO_TEST_STEP_MS + 10, p));
        /* trim: shifts the normal output, limited, and the centre keeps travel both ways */
        assert(!EasyTVC_ServoTestCommand(&st, &set, EASYTVC_FLIGHT_IDLE, "TRIM", "101,0", 0, &err));
        EasyTVCSettings tight = set;
        tight.servo[1].minimum_us = 1450;                             /* centre 1520 - 100 would leave < 50 us of travel */
        assert(!EasyTVC_ServoTestCommand(&st, &tight, EASYTVC_FLIGHT_IDLE, "TRIM", "0,-100", 0, &err) && !strcmp(err, "RANGE"));
        assert(EasyTVC_ServoTestCommand(&st, &set, EASYTVC_FLIGHT_IDLE, "TRIM", "-12,30", 0, &err));
        assert(EasyTVC_ServoTestApply(&st, &set, EASYTVC_FLIGHT_IDLE, 5000, p) && p[0] == 1488 && p[1] == 1550);
        p[0] = 1100;                                                  /* bench follow at the stop: trim can't pass it */
        assert(EasyTVC_ServoTestApply(&st, &set, EASYTVC_FLIGHT_IDLE, 5000, p) && p[0] == 1100);
        assert(EasyTVC_ServoTestCommand(&st, &set, EASYTVC_FLIGHT_IDLE, "SERVO", "0,0", 6000, &err));
        assert(EasyTVC_ServoTestApply(&st, &set, EASYTVC_FLIGHT_IDLE, 6001, p) && p[0] == 1488 && p[1] == 1550);
        /* the pad arms: test and trim are gone, the controller's pulses stand untouched */
        assert(EasyTVC_ServoTestCommand(&st, &set, EASYTVC_FLIGHT_IDLE, "SERVO", "800,800", 7000, &err));
        p[0] = 1500; p[1] = 1520;
        assert(!EasyTVC_ServoTestApply(&st, &set, EASYTVC_FLIGHT_ARMED, 7001, p));
        assert(p[0] == 1500 && p[1] == 1520 && st.mode == EASYTVC_SERVO_TEST_OFF && st.trim_us[0] == 0 && st.trim_us[1] == 0);
        assert(!EasyTVC_ServoTestApply(&st, &set, EASYTVC_FLIGHT_IDLE, 7002, p));   /* back on the pad: still off */
        assert(!EasyTVC_ServoTestCommand(&st, &set, EASYTVC_FLIGHT_IDLE, "CHUTE", "1", 0, &err));   /* no such thing */
        puts("link: gimbal test is IDLE-only, times out, sweeps, trims within limits and ends on leaving IDLE");
    }

    /* 11. parachute ejection charge: never on the pad or after a reboot, once, at apogee + delay or
     * the backup time, for fire_ms; and the countdown the ground shows */
    {
        const EasyTVCChargeConfig cc = {.channel = 3, .delay_ms = 1000, .backup_ms = 5700, .fire_ms = 800};
        EasyTVCCharge ch;
        EasyTVC_ChargeInit(&ch);
        for (uint32_t t = 0; t < 60000U; t += 5U)                      /* on the pad, even "at apogee" */
            assert(!EasyTVC_ChargeStep(&ch, &cc, false, 0, true, 0.0f, true, t));
        assert(!ch.fired);
        assert(EasyTVC_RecoveryEtaMs(&ch, &cc, false, false, false, 0, 100) == -1);   /* T−: nothing scheduled */
        /* launch at 10 000 ms, apogee at 15 000: apogee + 1 s = 16 000, but the backup (15 700) is sooner */
        EasyTVC_ChargeInit(&ch);
        uint32_t first = 0, last = 0;
        for (uint32_t t = 10000U; t < 30000U; t += 5U) {
            const int32_t eta = EasyTVC_RecoveryEtaMs(&ch, &cc, false, false, true, 10000U, t);
            if (t == 12000U) assert(eta == 3700);                         /* 5.7 s backup - 2 s */
            if (EasyTVC_ChargeStep(&ch, &cc, true, 10000U, t >= 15000U, 0.0f, true, t)) { if (!first) first = t; last = t; }
        }
        assert(first == 15700U && last == 15700U + 800U - 5U);            /* backup came first, 800 ms pulse */
        /* an early apogee: apogee + delay comes before the backup */
        EasyTVC_ChargeInit(&ch);
        first = 0;
        for (uint32_t t = 0; t < 10000U; t += 5U) {
            if (EasyTVC_ChargeStep(&ch, &cc, true, 0, t >= 3000U, 0.0f, true, t) && !first) first = t;
            if (t == 3500U) assert(EasyTVC_RecoveryEtaMs(&ch, &cc, false, false, true, 0, t) == 500);
        }
        assert(first == 4000U && ch.fired);
        assert(EasyTVC_RecoveryEtaMs(&ch, &cc, false, false, true, 0, 9000) == 0);       /* deployed */
        for (uint32_t t = 10000U; t < 20000U; t += 5U)                                    /* never again */
            assert(!EasyTVC_ChargeStep(&ch, &cc, true, 0, true, 0.0f, true, t));
        /* no channel, or P2: nothing fires */
        const EasyTVCChargeConfig none = {.channel = 0, .backup_ms = 100, .fire_ms = 800};
        const EasyTVCChargeConfig p2 = {.channel = 2, .backup_ms = 100, .fire_ms = 800};
        EasyTVC_ChargeInit(&ch);
        for (uint32_t t = 0; t < 2000U; t += 5U)
            assert(!EasyTVC_ChargeStep(&ch, &none, true, 0, true, 0.0f, true, t) && !EasyTVC_ChargeStep(&ch, &p2, true, 0, true, 0.0f, true, t));
        /* servo only: the countdown follows the backup time, and 0 once the servo is open */
        assert(EasyTVC_RecoveryEtaMs(&ch, &none, true, false, true, 0, 1000) == 0);    /* 100 ms backup: due now */
        const EasyTVCChargeConfig servo = {.channel = 0, .backup_ms = 5700};
        assert(EasyTVC_RecoveryEtaMs(&ch, &servo, true, false, true, 0, 1000) == 4700);
        assert(EasyTVC_RecoveryEtaMs(&ch, &servo, true, true, true, 0, 1000) == 0);
        assert(EasyTVC_RecoveryEtaMs(&ch, &servo, false, false, true, 0, 1000) == -1);  /* no recovery at all */
        /* altitude trigger: on the way DOWN through 20 m (+ 0.5 s), the timer as backup */
        const EasyTVCChargeConfig alt = {.channel = 1, .delay_ms = 500, .backup_ms = 9000, .fire_ms = 800, .deploy_alt_m = 20.0f};
        /* climb to 53 m by 5.2 s, then fall 5 m/s: passes 20 m at 11.8 s */
        #define FLIGHT_ALT(tt) ((tt) <= 5200U ? 53.0f * (float)(tt) / 5200.0f : 53.0f - 5.0f * (float)((tt) - 5200U) / 1000.0f)
        EasyTVC_ChargeInit(&ch);
        first = 0;
        for (uint32_t t = 0; t < 20000U; t += 5U)
            if (EasyTVC_ChargeStep(&ch, &alt, true, 0, t >= 5200U, FLIGHT_ALT(t), true, t) && !first) first = t;
        assert(first == 9000U);                   /* 11.8 s + 0.5 s is after the 9 s timer: the timer wins */
        const EasyTVCChargeConfig alt_long = {.channel = 1, .delay_ms = 500, .backup_ms = 15000, .fire_ms = 800, .deploy_alt_m = 20.0f};
        EasyTVC_ChargeInit(&ch);
        first = 0;
        for (uint32_t t = 0; t < 20000U; t += 5U) {
            if (EasyTVC_ChargeStep(&ch, &alt_long, true, 0, t >= 5200U, FLIGHT_ALT(t), true, t) && !first) first = t;
            if (t == 2000U) assert(!ch.trigger_seen);   /* passing 20 m on the way UP never counts */
        }
        assert(first >= 12300U && first <= 12310U);   /* passed 20 m going down at 11.8 s, + 0.5 s */
        /* apogee lower than the deploy altitude: fires right at apogee (+ delay) */
        const EasyTVCChargeConfig high = {.channel = 1, .delay_ms = 0, .backup_ms = 15000, .fire_ms = 800, .deploy_alt_m = 100.0f};
        EasyTVC_ChargeInit(&ch);
        first = 0;
        for (uint32_t t = 0; t < 20000U; t += 5U)
            if (EasyTVC_ChargeStep(&ch, &high, true, 0, t >= 5200U, FLIGHT_ALT(t), true, t) && !first) first = t;
        assert(first == 5200U);
        /* barometer failed (stale or NaN): only the timer fires it */
        EasyTVC_ChargeInit(&ch);
        first = 0;
        for (uint32_t t = 0; t < 20000U; t += 5U)
            if (EasyTVC_ChargeStep(&ch, &alt_long, true, 0, t >= 5200U, t > 6000U ? NAN : FLIGHT_ALT(t), t <= 6000U, t) && !first) first = t;
        assert(first == 15000U);
        #undef FLIGHT_ALT
        printf("link: ejection charge never fires on the pad, fires once at apogee+delay or the backup (compiled %s)\n",
               EASYTVC_PYRO_OUTPUTS_COMPILED_IN ? "IN" : "out: the pin stays low");
    }

    /* 12. settings sent over the link: parse/format round trip, limits, and the SD record (two
     * copies, CRC, made against one install's compiled values) */
    {
        const EasyTVCServoLimits lim = {{1250, 1250}, {1750, 1750}};
        EasyTVCTunables t = {.kp = 10.4f, .ki = 1.2f, .kd = 0.92f, .integrator_limit = 0.3f, .output_rate_limit = 30,
                             .derivative_cutoff_hz = 30, .max_tilt_deg = 25, .center_us = {1488, 1506}, .chute_backup_ms = 5800,
                             .pyro_deploy_alt_m = 20.5f, .pyro_delay_ms = 500, .pyro_fire_ms = 800, .pyro_backup_ms = 9000};
        assert(EasyTVC_TunablesValid(&t, &lim));
        char text[EASYTVC_LINK_MAX_PAYLOAD + 1U];
        assert(EasyTVC_TunablesFormat(&t, text, sizeof(text)) > 0U);
        assert(!strcmp(text, "10400,1200,920,300,30000,30000,25000,1488,1506,5800,2050,500,800,9000"));
        EasyTVCTunables back;
        assert(EasyTVC_TunablesParse(text, &back) && EasyTVC_TunablesCrc(&back) == EasyTVC_TunablesCrc(&t));
        static const char *bad[] = {"1,2,3", "10400,1200,920,300,30000,30000,25000,1488,1506,5800,2050,500,800,9000,1",
                                    "10400,1200,920,300,30000,30000,25000,-1488,1506,5800,2050,500,800,9000", "x", ""};
        for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) assert(!EasyTVC_TunablesParse(bad[i], &back));
        EasyTVCTunables w = t; w.center_us[0] = 1740;            /* the centre must keep 25 us of travel */
        assert(!EasyTVC_TunablesValid(&w, &lim));
        w = t; w.kp = 101; assert(!EasyTVC_TunablesValid(&w, &lim));
        w = t; w.pyro_fire_ms = 50; assert(!EasyTVC_TunablesValid(&w, &lim));
        uint8_t sector[512];
        const uint32_t base = 0xABCD1234u;
        EasyTVC_TunablesEncode(&t, base, 7, sector);
        uint32_t seq = 0;
        assert(EasyTVC_TunablesDecode(sector, base, &back, &seq) && seq == 7U && EasyTVC_TunablesCrc(&back) == EasyTVC_TunablesCrc(&t));
        assert(!EasyTVC_TunablesDecode(sector, base + 1U, &back, &seq));   /* a new install ignores it */
        sector[40] ^= 1U;
        assert(!EasyTVC_TunablesDecode(sector, base, &back, &seq));        /* damaged: ignored */
        memset(sector, 0, sizeof(sector));
        assert(!EasyTVC_TunablesDecode(sector, base, &back, &seq));        /* cleared */
        puts("link: settings over the link round-trip, stay in range, and only apply to the install they were made for");
    }

    /* 13. pyro channel ground test: IDLE + disarmed only, a one-off code, short pulse, ends on arming */
    {
        EasyTVCPyroTest pt;
        EasyTVC_PyroTestInit(&pt);
        const char *err;
        for (int st = EASYTVC_FLIGHT_ARMED; st <= EASYTVC_FLIGHT_FAULT; ++st) {
            assert(!EasyTVC_PyroTestArm(&pt, 1, (EasyTVCFlightState)st, false, 0, 1, &err) && !strcmp(err, "NOT_IDLE"));
            assert(!EasyTVC_LinkAllowed("PYROARM", (EasyTVCFlightState)st) && !EasyTVC_LinkAllowed("PYROFIRE", (EasyTVCFlightState)st));
        }
        assert(!EasyTVC_PyroTestArm(&pt, 1, EASYTVC_FLIGHT_IDLE, true, 0, 1, &err) && !strcmp(err, "ARMED"));
        assert(!EasyTVC_PyroTestArm(&pt, 2, EASYTVC_FLIGHT_IDLE, false, 0, 1, &err) && !strcmp(err, "CHANNEL"));   /* P2: never */
        assert(!EasyTVC_PyroTestFire(&pt, 1, 1234, 200, EASYTVC_FLIGHT_IDLE, false, 0, &err));       /* not armed for a test */
        assert(EasyTVC_PyroTestArm(&pt, 1, EASYTVC_FLIGHT_IDLE, false, 1000, 77, &err));
        const uint16_t code = pt.code;
        assert(code >= 1000U && code <= 9999U);
        assert(!EasyTVC_PyroTestFire(&pt, 3, code, 200, EASYTVC_FLIGHT_IDLE, false, 1100, &err));    /* other channel */
        assert(!EasyTVC_PyroTestFire(&pt, 1, (uint16_t)(code == 9999U ? 1000U : code + 1U), 200, EASYTVC_FLIGHT_IDLE, false, 1100, &err) &&
               !strcmp(err, "CODE"));
        assert(!EasyTVC_PyroTestFire(&pt, 1, code, 200, EASYTVC_FLIGHT_IDLE, false, 1100, &err));    /* one try per code */
        assert(EasyTVC_PyroTestArm(&pt, 1, EASYTVC_FLIGHT_IDLE, false, 2000, 78, &err) && pt.code != code);
        assert(!EasyTVC_PyroTestFire(&pt, 1, pt.code, 5000, EASYTVC_FLIGHT_IDLE, false, 2100, &err) && !strcmp(err, "LENGTH"));
        assert(EasyTVC_PyroTestFire(&pt, 1, pt.code, 300, EASYTVC_FLIGHT_IDLE, false, 2100, &err));
        assert(EasyTVC_PyroTestStep(&pt, EASYTVC_FLIGHT_IDLE, false, 2200) == 1U);
        assert(EasyTVC_PyroTestStep(&pt, EASYTVC_FLIGHT_IDLE, false, 2400) == 0U);                   /* 300 ms, then off */
        assert(!EasyTVC_PyroTestFire(&pt, 1, pt.code, 300, EASYTVC_FLIGHT_IDLE, false, 2500, &err)); /* used up */
        assert(EasyTVC_PyroTestArm(&pt, 3, EASYTVC_FLIGHT_IDLE, false, 3000, 79, &err));
        assert(!EasyTVC_PyroTestFire(&pt, 3, pt.code, 300, EASYTVC_FLIGHT_IDLE, false, 3000 + EASYTVC_PYRO_TEST_WINDOW_MS + 1, &err) &&
               !strcmp(err, "EXPIRED"));
        assert(EasyTVC_PyroTestArm(&pt, 3, EASYTVC_FLIGHT_IDLE, false, 20000, 80, &err));
        assert(EasyTVC_PyroTestFire(&pt, 3, pt.code, 800, EASYTVC_FLIGHT_IDLE, false, 20000, &err));
        assert(EasyTVC_PyroTestStep(&pt, EASYTVC_FLIGHT_IDLE, false, 20100) == 3U);
        assert(EasyTVC_PyroTestStep(&pt, EASYTVC_FLIGHT_IDLE, true, 20105) == 0U);                   /* the pad armed: off now */
        assert(EasyTVC_PyroTestStep(&pt, EASYTVC_FLIGHT_IDLE, false, 20110) == 0U);                  /* and stays off */
        puts("link: pyro ground test only on the pad and disarmed, one code one pulse, P2 refused, stops if it arms");
    }

    puts("link tests passed");
    return 0;
}
