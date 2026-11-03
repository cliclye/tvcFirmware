#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "easytvc_safety.h"

/*
 * Parachute ejection charge on a pyro terminal (P1 or P3), and the countdown to recovery.
 *
 * - Fires once, for fire_ms, at the first of:
 *     - deploy_alt_m > 0: delay_ms after the rocket, on its way DOWN (apogee detected), passes
 *       deploy_alt_m above the pad. If apogee is lower than that, right at apogee.
 *     - deploy_alt_m = 0: delay_ms after detected apogee.
 *     - backup_ms after launch: the timer, for when the barometer fails or apogee isn't detected.
 *       Altitude only counts while the barometer is fresh (altitude_ok).
 * - Never before launch has been detected in this boot. A reboot (watchdog, brown-out) never fires
 *   it: the firmware cannot tell a reset on the pad from one in the air, and on the pad a charge
 *   is dangerous. (The parachute servo, which is harmless on the pad, still opens after a reboot.)
 * - P2 is never allowed: it is the MCU's SWDIO pin, pulled up at every reset.
 * - This module only decides. The pin is driven only when EASYTVC_PYRO_OUTPUTS_COMPILED_IN is 1
 *   (easytvc_safety.h), which it is not until the board's pyro circuit has been bench-tested.
 */

typedef struct {
    uint8_t channel;        /* 0 = no charge, 1 or 3 */
    uint32_t delay_ms;      /* after the trigger (apogee, or the deploy altitude) */
    uint32_t backup_ms;     /* after launch, at the latest */
    uint32_t fire_ms;       /* how long the output stays on */
    float deploy_alt_m;     /* 0 = at apogee */
} EasyTVCChargeConfig;

typedef struct {
    bool apogee_seen;
    uint32_t apogee_ms;
    bool trigger_seen;      /* apogee (deploy_alt_m 0) or passing the deploy altitude on the way down */
    uint32_t trigger_ms;
    bool fired;             /* the fire time has come (the output may still be compiled out) */
    uint32_t fire_start_ms;
} EasyTVCCharge;

void EasyTVC_ChargeInit(EasyTVCCharge *charge);

/* Every loop tick. `launched`/`launch_ms`: launch detected in this boot. `apogee`: the flight state
 * has reached APOGEE or later (or the FSM's drogue request). `altitude_m` above the pad, trusted
 * only when `altitude_ok` (fresh barometer). Returns true while the output should be on. */
bool EasyTVC_ChargeStep(EasyTVCCharge *charge, const EasyTVCChargeConfig *config, bool launched,
                        uint32_t launch_ms, bool apogee, float altitude_m, bool altitude_ok, uint32_t now_ms);

/* Milliseconds until the parachute deploys: -1 before launch (or with no recovery configured),
 * 0 once it has. Covers the release servo (opens at apogee or the backup time) and the charge. */
int32_t EasyTVC_RecoveryEtaMs(const EasyTVCCharge *charge, const EasyTVCChargeConfig *config, bool servo_configured,
                              bool servo_open, bool launched, uint32_t launch_ms, uint32_t now_ms);
