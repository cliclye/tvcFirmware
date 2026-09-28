#include "app.h"
#include "controller.h"
#include "protocol.h"
#include "settings.h"
#include "runtime.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void settings_roundtrip(void)
{
    EasyTVCSettings a, b;
    uint8_t packet[EASYTVC_SETTINGS_SIZE], copy[EASYTVC_SETTINGS_SIZE];
    EasyTVC_SettingsDefault(&a);
    a.servo[1].direction = -1;
    a.servo[1].channel = 4;
    assert(EasyTVC_SettingsEncode(&a, packet));
    assert(EasyTVC_SettingsDecode(&b, packet, sizeof(packet)));
    assert(EasyTVC_SettingsEncode(&b, copy));
    assert(!memcmp(packet, copy, sizeof(packet)));
    assert(EasyTVC_Crc32("123456789", 9) == 0xcbf43926U);
    for (unsigned i = 0; i < sizeof(packet); ++i) {
        packet[i] ^= 0x80;
        assert(!EasyTVC_SettingsDecode(&b, packet, sizeof(packet)));
        packet[i] ^= 0x80;
    }
    assert(!EasyTVC_SettingsDecode(&b, packet, sizeof(packet) - 1));
    a.axis[0].kp = NAN;
    assert(!EasyTVC_SettingsValid(&a));
    EasyTVC_SettingsDefault(&a);
    a.servo[1].channel = a.servo[0].channel;
    assert(!EasyTVC_SettingsValid(&a));
    EasyTVC_SettingsDefault(&a);
    a.servo[0].minimum_us = 1600;
    assert(!EasyTVC_SettingsValid(&a));
}

static void controller_faults_and_axes(void)
{
    EasyTVCController c;
    EasyTVCSettings s;
    EasyTVC_SettingsDefault(&s);
    EasyTVC_ControllerReset(&c);
    EasyTVCQuaternion q = {.w = cosf(0.05f), .x = sinf(0.05f)};
    EasyTVCControlOutput out = EasyTVC_ControllerStep(&c, &s, &q, 0, 0, 0.005f, true, true, true);
    assert(out.enabled && out.command[0] < 0 && fabsf(out.command[1]) < 0.00001f);
    out = EasyTVC_ControllerStep(&c, &s, &q, 5, 0, 0.005f, true, true, true);
    assert(out.fault && !out.enabled && out.pulse_us[0] == 1500);
    out = EasyTVC_ControllerStep(&c, &s, &q, 10, 10, 0.005f, true, true, true);
    assert(out.fault && !out.enabled); /* Faults do not silently clear. */
    EasyTVC_ControllerReset(&c);
    q = (EasyTVCQuaternion){.w = cosf(0.5f), .z = sinf(0.5f)};
    out = EasyTVC_ControllerStep(&c, &s, &q, 0, 0, 0.005f, true, true, true);
    assert(out.enabled && out.command[0] == 0 && out.command[1] == 0); /* Axial spin. */
    out = EasyTVC_ControllerStep(&c, &s, &q, 5, 5, 0.005f, true, false, true);
    assert(out.fault && !out.enabled);
    EasyTVC_ControllerReset(&c);
    q = (EasyTVCQuaternion){.w = cosf(0.4f), .y = sinf(0.4f)};
    assert(EasyTVC_ControllerStep(&c, &s, &q, 0, 0, 0.005f, true, true, true).fault);
    EasyTVC_ControllerReset(&c);
    q = (EasyTVCQuaternion){.w = 1};
    assert(EasyTVC_ControllerStep(&c, &s, &q, 30, 0, 0.005f, true, true, true).fault);
    EasyTVC_ControllerReset(&c);
    assert(!EasyTVC_ControllerStep(&c, &s, &q, UINT32_MAX - 2, UINT32_MAX - 2,
        0.005f, true, true, true).fault);
    assert(!EasyTVC_ControllerStep(&c, &s, &q, 2, 2, 0.005f, true, true, true).fault);
    q.w = NAN;
    assert(EasyTVC_ControllerStep(&c, &s, &q, 7, 7, 0.005f, true, true, true).fault);
}

static void pid_saturation_and_invalid(void)
{
    EasyTVCSettings s;
    EasyTVC_SettingsDefault(&s);
    s.axis[0].ki = 1;
    s.axis[0].output_rate_limit = 0;
    EasyTVCPid pid;
    EasyTVC_PidReset(&pid);
    for (unsigned i = 0; i < 10000; ++i) {
        float out = EasyTVC_PidUpdate(&pid, &s.axis[0], 10, 0, 0.005f);
        assert(out <= 1 && out >= -1 && isfinite(out));
    }
    assert(pid.integrator == 0); /* Saturation must not wind up. */
    assert(EasyTVC_PidUpdate(&pid, &s.axis[0], 0, NAN, 0.005f) == 0);
    assert(EasyTVC_PidUpdate(&pid, &s.axis[0], 0, 0, NAN) == 0);
    s.axis[0].kp = INFINITY;
    assert(EasyTVC_PidUpdate(&pid, &s.axis[0], 0, 0, 0.005f) == 0);
    assert(EasyTVC_ServoPulse(&s.servo[0], INFINITY) == 1500);
    assert(EasyTVC_ServoPulse(&s.servo[0], 100) == 1750);
    s.servo[0].direction = -1;
    assert(EasyTVC_ServoPulse(&s.servo[0], 100) == 1250);
}

static size_t send_command(EasyTVCProtocol *p, const char *command, const char *payload, char *out)
{
    char line[256];
    int n = snprintf(line, sizeof(line), "E1 42 %s %s", command, payload);
    uint32_t crc = EasyTVC_Crc32(line, (size_t)n);
    snprintf(line + n, sizeof(line) - (size_t)n, " *%08x\n", (unsigned)crc);
    size_t response = 0;
    for (size_t i = 0; line[i]; ++i) response += EasyTVC_ProtocolFeed(p, line[i], out);
    return response;
}

static unsigned saves;
static bool save_ok(void *context, const uint8_t *data, size_t length)
{
    (void)context;
    EasyTVCSettings s;
    assert(EasyTVC_SettingsDecode(&s, data, length));
    ++saves;
    return true;
}

static void protocol_guards(void)
{
    EasyTVCProtocol p;
    EasyTVC_ProtocolInit(&p);
    char out[256], hex[177];
    uint8_t packet[88];
    assert(send_command(&p, "HELLO", "-", out) && strstr(out, "UNVERIFIED"));
    assert(send_command(&p, "GET", "-", out) && strstr(out, " OK "));
    EasyTVCSettings s = p.settings;
    s.axis[0].kp = 2;
    assert(EasyTVC_SettingsEncode(&s, packet));
    for (unsigned i = 0; i < sizeof(packet); ++i) snprintf(hex + i*2, 3, "%02x", packet[i]);
    assert(send_command(&p, "SET", hex, out) && strstr(out, "HARDWARE_UNVERIFIED"));
    p.hardware_verified = true;
    p.physical_arm = true;
    assert(send_command(&p, "SET", hex, out) && strstr(out, "ARMED_OR_ACTIVE"));
    p.physical_arm = false;
    p.state = EASYTVC_FLIGHT_BOOST;
    assert(send_command(&p, "SET", hex, out) && strstr(out, "ARMED_OR_ACTIVE"));
    p.state = EASYTVC_FLIGHT_IDLE;
    assert(send_command(&p, "SET", hex, out) && strstr(out, "STORAGE"));
    assert(p.settings.axis[0].kp != 2);
    p.save = save_ok;
    assert(send_command(&p, "SET", hex, out) && strstr(out, "SAVED"));
    assert(p.settings.axis[0].kp == 2 && saves == 1);
    assert(send_command(&p, "SET", hex, out) && strstr(out, "UNCHANGED") && saves == 1);
    for (unsigned i = 0; i < 1000; ++i) assert(!EasyTVC_ProtocolFeed(&p, 'A', out));
    assert(!EasyTVC_ProtocolFeed(&p, '\n', out));
    assert(send_command(&p, "HELLO", "-", out));
    assert(send_command(&p, "ARM", "-", out) && strstr(out, "COMMAND"));
    const char *bad = "E1 42 SET - *00000000\n";
    for (unsigned i = 0; bad[i]; ++i) assert(!EasyTVC_ProtocolFeed(&p, bad[i], out));
    assert(saves == 1);
}

static void app_rejects_bad_sample(void)
{
    EasyTVCApp app;
    EasyTVC_AppInit(&app, 0);
    EasyTVCAppInput in = {.accel_m_s2 = {0, 0, 9.80665f}, .dt_s = 0.005f,
        .sensors_healthy = true, .physical_arm_qualified = true, .software_arm_request = true};
    assert(EasyTVC_AppStep(&app, &in).flight.state == EASYTVC_FLIGHT_ARMED);
    in.now_ms = 5;
    in.gyro_rad_s[0] = NAN;
    EasyTVCAppOutput out = EasyTVC_AppStep(&app, &in);
    assert(out.flight.state == EASYTVC_FLIGHT_FAULT && !out.control.enabled);
    assert(isfinite(out.euler.roll_rad) && isfinite(out.nav.altitude_m));
    assert(!EasyTVC_AppApplySettings(&app, &app.settings, false));
}

static void runtime_applies_only_persisted_disarmed_settings(void)
{
    EasyTVCRuntime runtime;
    EasyTVC_RuntimeInit(&runtime, 0, true, save_ok, NULL);
    EasyTVCSettings s = runtime.app.settings;
    s.axis[0].kp = 4.0f;
    uint8_t packet[88];
    char hex[177], out[256];
    assert(EasyTVC_SettingsEncode(&s, packet));
    for (unsigned i = 0; i < sizeof(packet); ++i) snprintf(hex + i*2, 3, "%02x", packet[i]);
    assert(send_command(&runtime.protocol, "SET", hex, out));
    assert(strstr(out, "SAVED") && runtime.app.settings.axis[0].kp == 4.0f);
    assert(!EasyTVC_RuntimeLoadSettings(&runtime, packet, sizeof(packet), true));
    EasyTVC_RuntimeInit(&runtime, 0, false, save_ok, NULL);
    const EasyTVCAppInput sample = {.accel_m_s2 = {0, 0, 9.80665f}, .dt_s = .005f,
        .sensors_healthy = true, .software_arm_request = true, .physical_arm_qualified = true};
    const EasyTVCAppOutput result = EasyTVC_RuntimeStep(&runtime, &sample);
    assert(result.flight.state == EASYTVC_FLIGHT_IDLE && !result.control.enabled);
}

int main(void)
{
    settings_roundtrip();
    controller_faults_and_axes();
    pid_saturation_and_invalid();
    protocol_guards();
    app_rejects_bad_sample();
    runtime_applies_only_persisted_disarmed_settings();
    puts("Reliability tests passed: CRC, config, axes, stale/invalid samples, faults, PID, protocol.");
    return 0;
}
