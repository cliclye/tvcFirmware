#include "runtime.h"

static bool persist_and_apply(void *context, const uint8_t *data, size_t length)
{
    EasyTVCRuntime *r = context;
    EasyTVCSettings candidate;
    if (!r->protocol.hardware_verified || r->protocol.physical_arm ||
        r->app.flight.state != EASYTVC_FLIGHT_IDLE || !r->persist ||
        !EasyTVC_SettingsDecode(&candidate, data, length)) return false;
    if (!r->persist(r->persist_context, data, length)) return false;
    return EasyTVC_AppApplySettings(&r->app, &candidate, r->protocol.physical_arm);
}

void EasyTVC_RuntimeInit(EasyTVCRuntime *r, uint32_t now_ms, bool hardware_verified,
                        EasyTVCSaveSettings persist, void *context)
{
    EasyTVC_AppInit(&r->app, now_ms);
    EasyTVC_ProtocolInit(&r->protocol);
    r->persist = persist;
    r->persist_context = context;
    r->protocol.context = r;
    r->protocol.save = persist_and_apply;
    r->protocol.hardware_verified = hardware_verified;
}

bool EasyTVC_RuntimeLoadSettings(EasyTVCRuntime *r, const uint8_t *data, size_t size,
                                bool physical_arm)
{
    EasyTVCSettings settings;
    if (!EasyTVC_SettingsDecode(&settings, data, size) ||
        !EasyTVC_AppApplySettings(&r->app, &settings, physical_arm)) return false;
    r->protocol.settings = settings;
    r->protocol.settings_persisted = true;
    return true;
}

EasyTVCAppOutput EasyTVC_RuntimeStep(EasyTVCRuntime *r, const EasyTVCAppInput *input)
{
    EasyTVCAppInput sample = *input;
    if (!r->protocol.hardware_verified) {
        sample.sensors_healthy = false;
        sample.software_arm_request = false;
    }
    const EasyTVCAppOutput result = EasyTVC_AppStep(&r->app, &sample);
    r->protocol.state = result.flight.state;
    r->protocol.physical_arm = sample.physical_arm_qualified;
    return result;
}

size_t EasyTVC_RuntimeFeed(EasyTVCRuntime *r, char byte, bool physical_arm,
                          char response[EASYTVC_PROTOCOL_CAPACITY])
{
    r->protocol.state = r->app.flight.state;
    r->protocol.physical_arm = physical_arm;
    return EasyTVC_ProtocolFeed(&r->protocol, byte, response);
}
