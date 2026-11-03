#include "tunables.h"

#include <math.h>
#include <string.h>

#include "settings.h"
#include "text_line.h"

static bool in(float v, float lo, float hi) { return isfinite(v) && v >= lo && v <= hi; }

bool EasyTVC_TunablesValid(const EasyTVCTunables *t, const EasyTVCServoLimits *limits)
{
    /* the same ranges as Studio's Rocket setup (studio/launch.py CONFIG_FIELDS) */
    if (!in(t->kp, 0, 100) || !in(t->ki, 0, 100) || !in(t->kd, 0, 100) || !in(t->integrator_limit, 0, 1) ||
        !in(t->output_rate_limit, 0.01f, 100) || !in(t->derivative_cutoff_hz, 0, 100) ||
        !in(t->max_tilt_deg, 5, 45) || !in(t->pyro_deploy_alt_m, 0, 1000)) return false;
    for (unsigned i = 0; i < 2U; ++i) {
        if (t->center_us[i] < limits->min_us[i] + 25U || t->center_us[i] + 25U > limits->max_us[i]) return false;
    }
    return t->chute_backup_ms >= 1000U && t->chute_backup_ms <= 30000U && t->pyro_delay_ms <= 10000U &&
           t->pyro_fire_ms >= 100U && t->pyro_fire_ms <= 3000U &&
           t->pyro_backup_ms >= 1000U && t->pyro_backup_ms <= 60000U;
}

static int32_t milli(float v) { return (int32_t)lroundf(v * 1000.0f); }

static void to_ints(const EasyTVCTunables *t, int32_t v[EASYTVC_TUNABLES_COUNT])
{
    v[0] = milli(t->kp); v[1] = milli(t->ki); v[2] = milli(t->kd); v[3] = milli(t->integrator_limit);
    v[4] = milli(t->output_rate_limit); v[5] = milli(t->derivative_cutoff_hz); v[6] = milli(t->max_tilt_deg);
    v[7] = t->center_us[0]; v[8] = t->center_us[1]; v[9] = (int32_t)t->chute_backup_ms;
    v[10] = (int32_t)lroundf(t->pyro_deploy_alt_m * 100.0f); v[11] = (int32_t)t->pyro_delay_ms;
    v[12] = (int32_t)t->pyro_fire_ms; v[13] = (int32_t)t->pyro_backup_ms;
}

static bool from_ints(const int32_t v[EASYTVC_TUNABLES_COUNT], EasyTVCTunables *t)
{
    for (unsigned i = 7; i < EASYTVC_TUNABLES_COUNT; ++i) {
        if (v[i] < 0) return false;
    }
    if (v[7] > 65535 || v[8] > 65535) return false;
    t->kp = (float)v[0] / 1000.0f; t->ki = (float)v[1] / 1000.0f; t->kd = (float)v[2] / 1000.0f;
    t->integrator_limit = (float)v[3] / 1000.0f; t->output_rate_limit = (float)v[4] / 1000.0f;
    t->derivative_cutoff_hz = (float)v[5] / 1000.0f; t->max_tilt_deg = (float)v[6] / 1000.0f;
    t->center_us[0] = (uint16_t)v[7]; t->center_us[1] = (uint16_t)v[8];
    t->chute_backup_ms = (uint32_t)v[9]; t->pyro_deploy_alt_m = (float)v[10] / 100.0f;
    t->pyro_delay_ms = (uint32_t)v[11]; t->pyro_fire_ms = (uint32_t)v[12]; t->pyro_backup_ms = (uint32_t)v[13];
    return true;
}

size_t EasyTVC_TunablesFormat(const EasyTVCTunables *t, char *out, size_t capacity)
{
    int32_t v[EASYTVC_TUNABLES_COUNT];
    to_ints(t, v);
    EasyTVCTextLine line = {out, 0, capacity};
    EasyTVC_LineInt(&line, v[0]);
    for (unsigned i = 1; i < EASYTVC_TUNABLES_COUNT; ++i) EasyTVC_LineField(&line, v[i]);
    return line.used;
}

bool EasyTVC_TunablesParse(const char *payload, EasyTVCTunables *out)
{
    int32_t v[EASYTVC_TUNABLES_COUNT];
    const char *p = payload;
    for (unsigned i = 0; i < EASYTVC_TUNABLES_COUNT; ++i) {
        bool negative = false;
        if (*p == '-') { negative = true; ++p; }
        if (*p < '0' || *p > '9') return false;
        int64_t x = 0;
        for (; *p >= '0' && *p <= '9'; ++p) {
            x = x * 10 + (*p - '0');
            if (x > 100000000) return false;
        }
        v[i] = (int32_t)(negative ? -x : x);
        if (i + 1U < EASYTVC_TUNABLES_COUNT && *p++ != ',') return false;
    }
    return *p == '\0' && from_ints(v, out);
}

uint32_t EasyTVC_TunablesCrc(const EasyTVCTunables *t)
{
    int32_t v[EASYTVC_TUNABLES_COUNT];
    to_ints(t, v);
    return EasyTVC_Crc32(v, sizeof(v));
}

static const char MAGIC[8] = {'E', 'T', 'V', 'C', 'S', 'E', 'T', '1'};

static void put32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static uint32_t get32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

/* magic[8], sequence, base_crc, values[14], crc over everything before it */
#define RECORD_BYTES (8U + 4U + 4U + 4U * EASYTVC_TUNABLES_COUNT)

void EasyTVC_TunablesEncode(const EasyTVCTunables *t, uint32_t base_crc, uint32_t sequence, uint8_t sector[512])
{
    memset(sector, 0, 512);
    memcpy(sector, MAGIC, sizeof(MAGIC));
    put32(sector + 8, sequence);
    put32(sector + 12, base_crc);
    int32_t v[EASYTVC_TUNABLES_COUNT];
    to_ints(t, v);
    for (unsigned i = 0; i < EASYTVC_TUNABLES_COUNT; ++i) put32(sector + 16 + 4U * i, (uint32_t)v[i]);
    put32(sector + RECORD_BYTES, EasyTVC_Crc32(sector, RECORD_BYTES));
}

bool EasyTVC_TunablesDecode(const uint8_t sector[512], uint32_t base_crc, EasyTVCTunables *out, uint32_t *sequence)
{
    if (memcmp(sector, MAGIC, sizeof(MAGIC)) != 0 || get32(sector + RECORD_BYTES) != EasyTVC_Crc32(sector, RECORD_BYTES) ||
        get32(sector + 12) != base_crc) return false;
    int32_t v[EASYTVC_TUNABLES_COUNT];
    for (unsigned i = 0; i < EASYTVC_TUNABLES_COUNT; ++i) v[i] = (int32_t)get32(sector + 16 + 4U * i);
    EasyTVCTunables t;
    if (!from_ints(v, &t)) return false;
    *out = t;
    *sequence = get32(sector + 8);
    return true;
}
