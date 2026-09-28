#include "settings.h"

#include <math.h>
#include <string.h>

_Static_assert(sizeof(float) == 4, "Protocol requires binary32 float");

static void put_u32(uint8_t *p, uint32_t n)
{
    for (unsigned i = 0; i < 4; ++i) p[i] = (uint8_t)(n >> (8U * i));
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static void put_float(uint8_t *p, float f)
{
    uint32_t n;
    memcpy(&n, &f, sizeof(n));
    put_u32(p, n);
}

static float get_float(const uint8_t *p)
{
    uint32_t n = get_u32(p);
    float f;
    memcpy(&f, &n, sizeof(f));
    return f;
}

uint32_t EasyTVC_Crc32(const void *data, size_t length)
{
    const uint8_t *p = data;
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < length; ++i) {
        crc ^= p[i];
        for (unsigned j = 0; j < 8; ++j) crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
    }
    return ~crc;
}

void EasyTVC_SettingsDefault(EasyTVCSettings *s)
{
    *s = (EasyTVCSettings){0};
    for (unsigned i = 0; i < 2; ++i) {
        s->axis[i] = (EasyTVCPidConfig){.kp = 1.2f, .ki = 0.0f, .kd = 0.2f,
            .integrator_limit = 0.15f, .output_limit = 1.0f,
            .output_rate_limit = 4.0f, .derivative_cutoff_hz = 20.0f};
        s->servo[i] = (EasyTVCServoSettings){.minimum_us = 1250, .center_us = 1500,
            .maximum_us = 1750, .channel = (uint8_t)(i + 1), .direction = 1};
    }
    s->maximum_tilt_deg = 25.0f;
}

bool EasyTVC_SettingsValid(const EasyTVCSettings *s)
{
    if (!s || !isfinite(s->maximum_tilt_deg) || s->maximum_tilt_deg < 5.0f ||
        s->maximum_tilt_deg > 45.0f || s->servo[0].channel == s->servo[1].channel) return false;
    for (unsigned i = 0; i < 2; ++i) {
        const EasyTVCServoSettings *v = &s->servo[i];
        if (!EasyTVC_PidConfigValid(&s->axis[i]) || s->axis[i].output_limit > 1.0f ||
            s->axis[i].output_rate_limit <= 0.0f || v->channel < 1 || v->channel > 4 ||
            (v->direction != 1 && v->direction != -1) || v->minimum_us < 900 ||
            v->maximum_us > 2100 || v->minimum_us + 25 > v->center_us ||
            v->center_us + 25 > v->maximum_us) return false;
    }
    return true;
}

bool EasyTVC_SettingsEncode(const EasyTVCSettings *s, uint8_t data[EASYTVC_SETTINGS_SIZE])
{
    if (!data || !EasyTVC_SettingsValid(s)) return false;
    memcpy(data, "ETC1", 4);
    put_u32(data + 4, EASYTVC_SETTINGS_VERSION);
    for (unsigned i = 0; i < 2; ++i) {
        const EasyTVCPidConfig *c = &s->axis[i];
        const float fields[] = {c->kp, c->ki, c->kd, c->integrator_limit,
            c->output_limit, c->output_rate_limit, c->derivative_cutoff_hz};
        for (unsigned j = 0; j < 7; ++j) put_float(data + 8 + i * 28 + j * 4, fields[j]);
        const EasyTVCServoSettings *v = &s->servo[i];
        uint8_t *p = data + 64 + i * 8;
        p[0] = v->minimum_us; p[1] = v->minimum_us >> 8;
        p[2] = v->center_us; p[3] = v->center_us >> 8;
        p[4] = v->maximum_us; p[5] = v->maximum_us >> 8;
        p[6] = v->channel; p[7] = (uint8_t)v->direction;
    }
    put_float(data + 80, s->maximum_tilt_deg);
    put_u32(data + 84, EasyTVC_Crc32(data, 84));
    return true;
}

bool EasyTVC_SettingsDecode(EasyTVCSettings *s, const uint8_t *data, size_t length)
{
    if (!s || !data || length != EASYTVC_SETTINGS_SIZE || memcmp(data, "ETC1", 4) ||
        get_u32(data + 4) != EASYTVC_SETTINGS_VERSION ||
        get_u32(data + 84) != EasyTVC_Crc32(data, 84)) return false;
    EasyTVCSettings candidate = {0};
    for (unsigned i = 0; i < 2; ++i) {
        const uint8_t *p = data + 8 + i * 28;
        candidate.axis[i] = (EasyTVCPidConfig){.kp = get_float(p), .ki = get_float(p + 4),
            .kd = get_float(p + 8), .integrator_limit = get_float(p + 12),
            .output_limit = get_float(p + 16), .output_rate_limit = get_float(p + 20),
            .derivative_cutoff_hz = get_float(p + 24)};
        p = data + 64 + i * 8;
        candidate.servo[i] = (EasyTVCServoSettings){
            .minimum_us = (uint16_t)(p[0] | p[1] << 8),
            .center_us = (uint16_t)(p[2] | p[3] << 8),
            .maximum_us = (uint16_t)(p[4] | p[5] << 8),
            .channel = p[6], .direction = p[7] == 255 ? -1 : (int8_t)p[7]};
    }
    candidate.maximum_tilt_deg = get_float(data + 80);
    if (!EasyTVC_SettingsValid(&candidate)) return false;
    *s = candidate;
    return true;
}

uint16_t EasyTVC_ServoPulse(const EasyTVCServoSettings *s, float command)
{
    if (!s || s->minimum_us < 900 || s->maximum_us > 2100 ||
        s->minimum_us >= s->center_us || s->center_us >= s->maximum_us ||
        (s->direction != 1 && s->direction != -1)) return 1500;
    if (!isfinite(command)) command = 0.0f;
    command = fmaxf(-1.0f, fminf(1.0f, command)) * s->direction;
    const float span = command >= 0.0f ? s->maximum_us - s->center_us : s->center_us - s->minimum_us;
    return (uint16_t)((float)s->center_us + span * command + 0.5f);
}
