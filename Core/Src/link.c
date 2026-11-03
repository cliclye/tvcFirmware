#include "link.h"

#include <math.h>
#include <string.h>

#include "settings.h"
#include "text_line.h"

static bool token_ok(const char *s, size_t max)
{
    size_t n = 0;
    for (; s[n] != '\0'; ++n) {
        if (s[n] <= ' ' || s[n] > '~' || n >= max) return false;
    }
    return n > 0;
}

void EasyTVC_LinkInit(EasyTVCLink *link, EasyTVCLinkWrite write, EasyTVCLinkHandler handler, void *context)
{
    memset(link, 0, sizeof(*link));
    link->write = write;
    link->handler = handler;
    link->context = context;
}

size_t EasyTVC_LinkEncode(char *out, size_t capacity, uint32_t sequence, const char *id, const char *payload)
{
    if (capacity < EASYTVC_LINK_MAX_FRAME + 1U || !token_ok(id, 8)) return 0;
    /* one pass over the payload (checked while copied), then the CRC: this runs for every frame */
    size_t n = 0;
    out[n++] = 'E'; out[n++] = '1'; out[n++] = ' ';
    char digits[10];
    unsigned d = 0;
    do { digits[d++] = (char)('0' + sequence % 10U); sequence /= 10U; } while (sequence != 0U);   /* unsigned */
    while (d) out[n++] = digits[--d];
    out[n++] = ' ';
    for (const char *p = id; *p; ++p) out[n++] = *p;
    out[n++] = ' ';
    const size_t start = n;
    for (const char *p = payload; *p; ++p) {
        if (*p <= ' ' || *p > '~' || n - start >= EASYTVC_LINK_MAX_PAYLOAD) return 0;   /* spaces split fields */
        out[n++] = *p;
    }
    if (n == start || n + 11U > EASYTVC_LINK_MAX_FRAME) return 0;
    const uint32_t crc = EasyTVC_Crc32(out, n);
    static const char hex[] = "0123456789abcdef";
    out[n++] = ' ';
    out[n++] = '*';
    for (unsigned i = 0; i < 8U; ++i) out[n++] = hex[(crc >> (28U - 4U * i)) & 15U];
    out[n++] = '\n';
    out[n] = '\0';
    return n;
}

bool EasyTVC_LinkSend(EasyTVCLink *link, const char *id, const char *payload)
{
    char frame[EASYTVC_LINK_MAX_FRAME + 1U];
    const size_t n = EasyTVC_LinkEncode(frame, sizeof(frame), link->tx_sequence++, id, payload);
    if (n == 0U || !link->write || !link->write(link->context, frame, n)) {
        ++link->tx_dropped;
        return false;
    }
    return true;
}

bool EasyTVC_LinkAllowed(const char *command, EasyTVCFlightState state)
{
    if (!strcmp(command, "HELLO") || !strcmp(command, "PING")) return true;
    return state == EASYTVC_FLIGHT_IDLE;
}

static int unhex(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Split "E1 seq CMD payload" in place. */
static bool parse_request(char *line, uint32_t *sequence, char **command, char **payload)
{
    char *star = strstr(line, " *");
    if (!star || strlen(star + 2) != 8U) return false;
    uint32_t crc = 0;
    for (unsigned i = 0; i < 8U; ++i) {
        const int h = unhex(star[2 + i]);
        if (h < 0) return false;
        crc = (crc << 4) | (uint32_t)h;
    }
    *star = '\0';
    if (EasyTVC_Crc32(line, strlen(line)) != crc) return false;
    char *fields[4];
    unsigned count = 0;
    for (char *p = line; *p != '\0' && count < 4U;) {
        fields[count++] = p;
        char *space = strchr(p, ' ');
        if (!space) break;
        *space = '\0';
        p = space + 1;
        if (count == 4U && *p != '\0') return false;   /* a fifth field */
    }
    if (count != 4U || strcmp(fields[0], "E1")) return false;
    uint64_t value = 0;
    for (const char *s = fields[1]; *s; ++s) {
        if (*s < '0' || *s > '9') return false;
        value = value * 10U + (uint64_t)(*s - '0');
        if (value > 0xFFFFFFFFull) return false;
    }
    if (fields[1][0] == '\0' || !token_ok(fields[2], 8) || !token_ok(fields[3], EASYTVC_LINK_MAX_PAYLOAD)) return false;
    *sequence = (uint32_t)value;
    *command = fields[2];
    *payload = fields[3];
    return true;
}

bool EasyTVC_LinkFeed(EasyTVCLink *link, uint8_t byte, EasyTVCFlightState state)
{
    if (byte == '\r') return false;
    if (byte != '\n') {
        if (link->dropping) return false;
        if (byte < 32U || byte > 126U || link->used >= EASYTVC_LINK_MAX_FRAME) {
            link->dropping = true;
            ++link->rx_bad;
            return false;
        }
        link->line[link->used++] = (char)byte;
        return false;
    }
    const bool was_dropping = link->dropping;
    link->line[link->used] = '\0';
    const size_t used = link->used;
    link->used = 0;
    link->dropping = false;
    if (was_dropping || used == 0U) return false;
    uint32_t sequence;
    char *command, *payload;
    if (!parse_request(link->line, &sequence, &command, &payload)) {
        ++link->rx_bad;
        return false;
    }
    ++link->rx_frames;
    if (!EasyTVC_LinkAllowed(command, state)) {
        ++link->rx_ignored;       /* in flight: not processed, not answered */
        return true;
    }
    char out[EASYTVC_LINK_MAX_PAYLOAD + 1U] = "COMMAND";
    const bool ok = link->handler && link->handler(link->context, command, payload, out, sizeof(out));
    char frame[EASYTVC_LINK_MAX_FRAME + 1U];
    const size_t n = EasyTVC_LinkEncode(frame, sizeof(frame), sequence, ok ? "OK" : "ERR", out[0] ? out : "-");
    if (n != 0U && link->write) (void)link->write(link->context, frame, n);
    return true;
}

size_t EasyTVC_Base64(const uint8_t *data, size_t length, char *out, size_t capacity)
{
    static const char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    const size_t need = (length + 2U) / 3U * 4U;
    if (need + 1U > capacity) return 0;
    size_t o = 0;
    for (size_t i = 0; i < length; i += 3U) {
        const uint32_t v = ((uint32_t)data[i] << 16) | (i + 1U < length ? (uint32_t)data[i + 1U] << 8 : 0U) |
                           (i + 2U < length ? (uint32_t)data[i + 2U] : 0U);
        out[o++] = table[(v >> 18) & 63U];
        out[o++] = table[(v >> 12) & 63U];
        out[o++] = i + 1U < length ? table[(v >> 6) & 63U] : '=';
        out[o++] = i + 2U < length ? table[v & 63U] : '=';
    }
    out[o] = '\0';
    return o;
}

static int32_t scaled(float value, float scale)
{
    const float v = value * scale;
    if (!isfinite(v)) return 0;
    if (v > 2.0e9f) return 2000000000;
    if (v < -2.0e9f) return -2000000000;
    /* round half away from zero, like lroundf but inline: 18 of these per TLM frame. Exact while
     * |v| < 2^23 (8 million), far beyond any telemetry value; above that v is a whole number anyway
     * to within one unit. */
    return (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

size_t EasyTVC_LinkFormatTlm(const EasyTVCLinkTlm *t, char *out, size_t capacity)
{
    EasyTVCTextLine line = {out, 0, capacity};
    EasyTVC_LineInt(&line, (int32_t)t->ms);
    EasyTVC_LineField(&line, t->state);
    EasyTVC_LineField(&line, t->since_launch_ms);
    for (unsigned i = 0; i < 4U; ++i) EasyTVC_LineField(&line, scaled(t->q[i], 1.0e4f));
    for (unsigned i = 0; i < 3U; ++i) EasyTVC_LineField(&line, scaled(t->gyro_rad_s[i], 1.0e3f));
    for (unsigned i = 0; i < 3U; ++i) EasyTVC_LineField(&line, scaled(t->accel_m_s2[i], 1000.0f / 9.80665f));
    EasyTVC_LineField(&line, scaled(t->altitude_m, 100.0f));
    EasyTVC_LineField(&line, scaled(t->vertical_speed_m_s, 100.0f));
    for (unsigned i = 0; i < 2U; ++i) EasyTVC_LineField(&line, scaled(t->tilt_rad[i], 5729.578f));
    for (unsigned i = 0; i < 2U; ++i) EasyTVC_LineField(&line, scaled(t->command[i], 1000.0f));
    for (unsigned i = 0; i < 3U; ++i) EasyTVC_LineField(&line, t->pulse_us[i]);
    EasyTVC_LineField(&line, t->flags);
    EasyTVC_LineField(&line, t->deploy_eta_ms);
    EasyTVC_LineField(&line, (int32_t)t->pressure_pa);
    EasyTVC_LineField(&line, t->baro_temp_cdeg);
    return line.used;
}

size_t EasyTVC_LinkFormatSys(const EasyTVCLinkSys *s, char *out, size_t capacity)
{
    EasyTVCTextLine line = {out, 0, capacity};
    EasyTVC_LineInt(&line, (int32_t)s->ms);
    const int32_t fields[] = {
        s->image, s->state, s->armed, s->chute_open, s->batt_pin_mv, s->vdda_mv, s->continuity,
        s->pyro_armed, s->pyro_fired, s->sd_ok, (int32_t)s->session, (int32_t)s->records,
        (int32_t)s->dropped, (int32_t)s->write_errors, (int32_t)s->sd_left_kb, (int32_t)s->loop_max_us,
        (int32_t)s->late_ticks, (int32_t)s->imu_errors, (int32_t)s->baro_age_ms, (int32_t)s->link_dropped,
        (int32_t)s->link_ignored, s->errors,
    };
    for (unsigned i = 0; i < sizeof(fields) / sizeof(fields[0]); ++i) EasyTVC_LineField(&line, fields[i]);
    return line.used;
}
