#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Settings that can be changed over the ground link (IDLE only) and kept on the SD card: the TVC
 * gains, the servo centres (alignment), and the recovery timing. Wiring (channels, directions,
 * endpoints, nose axis) is not here: a wrong value there could drive the gimbal into its stops, so
 * it changes only by building and installing over USB-C.
 *
 * Which values are used:
 * - Installing firmware: the values compiled from vehicle_config.h.
 * - After settings are sent over the link: those, until the next install. Each stored record
 *   carries a CRC of the compiled values it was made against; a new install (different compiled
 *   values) ignores it, so what you install is always what runs.
 *
 * On the card: two copies (sectors EASYTVC_TUNABLES_SECTOR and +1), alternately written, each with
 * a sequence number and CRC-32, so a power cut mid-write leaves the other copy intact.
 */

#define EASYTVC_TUNABLES_SECTOR 1000U     /* in the reserved area before the log directory (1024) */
#define EASYTVC_TUNABLES_COUNT  14U

typedef struct {
    float kp, ki, kd;
    float integrator_limit;
    float output_rate_limit;
    float derivative_cutoff_hz;
    float max_tilt_deg;
    uint16_t center_us[2];           /* servo A, B: the alignment "straight" */
    uint32_t chute_backup_ms;        /* release servo: at apogee, or this long after launch */
    float pyro_deploy_alt_m;         /* charge: on the way down at this height above the pad; 0 = at apogee */
    uint32_t pyro_delay_ms;          /* charge: extra delay after the trigger */
    uint32_t pyro_fire_ms;           /* charge: how long the output is on */
    uint32_t pyro_backup_ms;         /* charge: this long after launch at the latest */
} EasyTVCTunables;

/* The servo endpoints the centres must stay inside (compiled, not tunable). */
typedef struct {
    uint16_t min_us[2], max_us[2];
} EasyTVCServoLimits;

bool EasyTVC_TunablesValid(const EasyTVCTunables *t, const EasyTVCServoLimits *limits);

/* As integers for the link: kp,ki,kd,ilimit,slew,dcut,max_tilt (milli), centre_a, centre_b (us),
 * chute_backup_ms, deploy_alt (cm), pyro_delay_ms, pyro_fire_ms, pyro_backup_ms. */
size_t EasyTVC_TunablesFormat(const EasyTVCTunables *t, char *out, size_t capacity);
bool EasyTVC_TunablesParse(const char *payload, EasyTVCTunables *out);

/* CRC of a value set: identifies "the compiled values" a stored record belongs to. */
uint32_t EasyTVC_TunablesCrc(const EasyTVCTunables *t);

void EasyTVC_TunablesEncode(const EasyTVCTunables *t, uint32_t base_crc, uint32_t sequence, uint8_t sector[512]);
/* Valid record made against `base_crc`: fills `out` and `sequence`. */
bool EasyTVC_TunablesDecode(const uint8_t sector[512], uint32_t base_crc, EasyTVCTunables *out, uint32_t *sequence);
