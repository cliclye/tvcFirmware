#include "recovery_charge.h"

#include <string.h>

void EasyTVC_ChargeInit(EasyTVCCharge *charge)
{
    memset(charge, 0, sizeof(*charge));
}

static bool configured(const EasyTVCChargeConfig *config)
{
    return config->channel == 1U || config->channel == 3U;
}

/* ms after launch when the charge is due, given what is known now */
static uint32_t due_after_launch(const EasyTVCCharge *charge, const EasyTVCChargeConfig *config, uint32_t launch_ms)
{
    uint32_t due = config->backup_ms;
    if (charge->trigger_seen) {
        const uint32_t at_trigger = charge->trigger_ms - launch_ms + config->delay_ms;
        if (at_trigger < due) due = at_trigger;
    }
    return due;
}

bool EasyTVC_ChargeStep(EasyTVCCharge *charge, const EasyTVCChargeConfig *config, bool launched,
                        uint32_t launch_ms, bool apogee, float altitude_m, bool altitude_ok, uint32_t now_ms)
{
    if (!configured(config) || !launched) {
        return false;
    }
    if (apogee && !charge->apogee_seen) {
        charge->apogee_seen = true;
        charge->apogee_ms = now_ms;
    }
    if (charge->apogee_seen && !charge->trigger_seen) {
        /* on the way down: at apogee, or at the deploy altitude (only with a fresh barometer) */
        const bool at_height = config->deploy_alt_m <= 0.0f ||
                               (altitude_ok && altitude_m == altitude_m && altitude_m <= config->deploy_alt_m);
        if (at_height) {
            charge->trigger_seen = true;
            charge->trigger_ms = now_ms;
        }
    }
    if (!charge->fired && now_ms - launch_ms >= due_after_launch(charge, config, launch_ms)) {
        charge->fired = true;               /* once per boot */
        charge->fire_start_ms = now_ms;
    }
    return charge->fired && now_ms - charge->fire_start_ms < config->fire_ms;
}

int32_t EasyTVC_RecoveryEtaMs(const EasyTVCCharge *charge, const EasyTVCChargeConfig *config, bool servo_configured,
                              bool servo_open, bool launched, uint32_t launch_ms, uint32_t now_ms)
{
    const bool charge_configured = configured(config);
    if (!launched || (!servo_configured && !charge_configured)) {
        return -1;
    }
    if ((servo_configured && servo_open) || (charge_configured && charge->fired)) {
        return 0;
    }
    const uint32_t since = now_ms - launch_ms;
    /* the servo opens at apogee (it will have by the next tick) or at the backup time */
    uint32_t due = charge_configured ? due_after_launch(charge, config, launch_ms) : config->backup_ms;
    if (servo_configured && config->backup_ms < due) due = config->backup_ms;
    return due > since ? (int32_t)(due - since) : 0;
}
