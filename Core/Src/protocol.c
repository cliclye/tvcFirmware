#include "protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

static int unhex(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

void EasyTVC_ProtocolInit(EasyTVCProtocol *p)
{
    memset(p, 0, sizeof(*p));
    EasyTVC_SettingsDefault(&p->settings);
}

static size_t respond(char *out, uint32_t sequence, const char *status, const char *payload)
{
    int n = snprintf(out, EASYTVC_PROTOCOL_CAPACITY, "E1 %" PRIu32 " %s %s", sequence, status, payload);
    if (n < 0 || (unsigned)n + 11 >= EASYTVC_PROTOCOL_CAPACITY) return 0;
    const uint32_t crc = EasyTVC_Crc32(out, (size_t)n);
    int tail = snprintf(out + n, EASYTVC_PROTOCOL_CAPACITY - (size_t)n, " *%08" PRIx32 "\n", crc);
    return tail > 0 ? (size_t)n + (size_t)tail : 0;
}

static size_t process(EasyTVCProtocol *p, char *out)
{
    char *star = strchr(p->line, '*');
    if (!star || star == p->line || star[-1] != ' ' || strlen(star + 1) != 8) return 0;
    uint32_t received_crc = 0;
    for (unsigned i = 0; i < 8; ++i) {
        int h = unhex(star[1 + i]);
        if (h < 0) return 0;
        received_crc = (received_crc << 4) | (unsigned)h;
    }
    star[-1] = '\0';
    if (EasyTVC_Crc32(p->line, strlen(p->line)) != received_crc) return 0;
    char version[4], seq[12], command[12], payload[180], extra;
    if (sscanf(p->line, "%3s %11s %11s %179s %c", version, seq, command, payload, &extra) != 4 ||
        strcmp(version, "E1")) return 0;
    for (const char *s = seq; *s; ++s) if (*s < '0' || *s > '9') return 0;
    unsigned long long number = strtoull(seq, NULL, 10);
    if (number > UINT32_MAX) return 0;
    const uint32_t id = (uint32_t)number;
    if (!strcmp(command, "HELLO") && !strcmp(payload, "-"))
        return respond(out, id, "OK", p->hardware_verified ? "EASYTVC,1,VERIFIED" : "EASYTVC,1,UNVERIFIED");
    if (!strcmp(command, "STATUS") && !strcmp(payload, "-")) {
        char state[32];
        snprintf(state, sizeof(state), "%u,%u,%u", (unsigned)p->state,
            p->physical_arm ? 1U : 0U, p->hardware_verified ? 1U : 0U);
        return respond(out, id, "OK", state);
    }
    if (!strcmp(command, "GET") && !strcmp(payload, "-")) {
        uint8_t bytes[EASYTVC_SETTINGS_SIZE];
        char hex[EASYTVC_SETTINGS_SIZE * 2 + 1];
        if (!EasyTVC_SettingsEncode(&p->settings, bytes)) return respond(out, id, "ERR", "CONFIG");
        for (unsigned i = 0; i < sizeof(bytes); ++i) snprintf(hex + i * 2, 3, "%02x", bytes[i]);
        return respond(out, id, "OK", hex);
    }
    if (strcmp(command, "SET")) return respond(out, id, "ERR", "COMMAND");
    if (!p->hardware_verified) return respond(out, id, "ERR", "HARDWARE_UNVERIFIED");
    if (p->physical_arm || p->state != EASYTVC_FLIGHT_IDLE) return respond(out, id, "ERR", "ARMED_OR_ACTIVE");
    if (strlen(payload) != EASYTVC_SETTINGS_SIZE * 2) return respond(out, id, "ERR", "LENGTH");
    uint8_t bytes[EASYTVC_SETTINGS_SIZE];
    for (unsigned i = 0; i < sizeof(bytes); ++i) {
        const int a = unhex(payload[i * 2]), b = unhex(payload[i * 2 + 1]);
        if (a < 0 || b < 0) return respond(out, id, "ERR", "HEX");
        bytes[i] = (uint8_t)(a * 16 + b);
    }
    EasyTVCSettings candidate;
    if (!EasyTVC_SettingsDecode(&candidate, bytes, sizeof(bytes))) return respond(out, id, "ERR", "CONFIG");
    uint8_t current[EASYTVC_SETTINGS_SIZE];
    if (p->settings_persisted && EasyTVC_SettingsEncode(&p->settings, current) && !memcmp(current, bytes, sizeof(bytes)))
        return respond(out, id, "OK", "UNCHANGED");
    /* Adapter must atomically persist and verify a versioned record before returning true. */
    if (!p->save || !p->save(p->context, bytes, sizeof(bytes))) return respond(out, id, "ERR", "STORAGE");
    p->settings = candidate;
    p->settings_persisted = true;
    return respond(out, id, "OK", "SAVED");
}

size_t EasyTVC_ProtocolFeed(EasyTVCProtocol *p, char byte, char out[EASYTVC_PROTOCOL_CAPACITY])
{
    if (byte == '\n') {
        p->line[p->used] = '\0';
        size_t n = p->dropping ? 0 : process(p, out);
        p->used = 0;
        p->dropping = false;
        return n;
    }
    if (p->dropping) return 0;
    if (byte < 32 || byte > 126 || p->used + 1 >= sizeof(p->line)) {
        p->dropping = true;
        return 0;
    }
    p->line[p->used++] = byte;
    return 0;
}
