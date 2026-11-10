/* End-to-end flight logic on synthetic sensors: pad arming, gyro bias capture, launch, closed-loop
 * TVC on a single-axis rigid body, burnout, apogee and parachute release. Same core code as the
 * flight image (app + pad_arm); only the sensors and the vehicle are simulated. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app.h"
#include "pad_arm.h"

#define G 9.80665f
#define DT_MS 5U

static const EasyTVCPadArmConfig pad_config = {
    .hold_ms = 3000U,
    .cos_arm_tilt = 0.98480775f,     /* 10 deg */
    .cos_disarm_tilt = 0.93969262f,  /* 20 deg */
    .still_gyro_rad_s = 0.05f,
    .disarm_confirm_ms = 500U,
};

/* Set by a test to replay a failed IMU read: the firmware passes the same sample again, older. */
static uint32_t stale_sensor_age_ms;

typedef struct {
    EasyTVCApp app;
    EasyTVCPadArm pad;
    float bias[3];          /* what the firmware believes */
    bool launched;
    uint32_t launch_ms;
    bool chute_open;
    uint32_t chute_ms;
    uint32_t armed_ms;
    EasyTVCAppOutput out;
} Vehicle;

static void vehicle_init(Vehicle *v)
{
    memset(v, 0, sizeof(*v));
    EasyTVC_AppInit(&v->app, 0);
    v->app.flight_config.failsafe_apogee_ms = 7000U;
    EasyTVC_FlightInit(&v->app.flight, &v->app.flight_config, 0);
    EasyTVC_PadArmInit(&v->pad);
}

/* One 200 Hz loop, mirroring main.c. accel/gyro_raw are sensor = body frame here. */
static void vehicle_step(Vehicle *v, uint32_t now, const float accel[3], const float gyro_raw[3],
                         float baro_alt)
{
    float gyro[3];
    for (unsigned i = 0; i < 3; ++i) gyro[i] = gyro_raw[i] - v->bias[i];
    float new_bias[3];
    if (EasyTVC_PadArmStep(&v->pad, &pad_config, v->app.flight.state, true, true, accel, gyro,
                           gyro_raw, now, new_bias)) {
        memcpy(v->bias, new_bias, sizeof(new_bias));
        for (unsigned i = 0; i < 3; ++i) {
            gyro[i] = gyro_raw[i] - v->bias[i];
            v->app.attitude.integral_error[i] = 0.0f;
        }
        v->armed_ms = now;
    }
    EasyTVCAppInput in = {
        .baro_altitude_m = baro_alt, .sensors_healthy = true,
        .physical_arm_qualified = v->pad.armed, .software_arm_request = v->pad.armed,
        .accel_is_gravity_reference = true, .now_ms = now, .sensor_age_ms = stale_sensor_age_ms,
        .dt_s = DT_MS * 1e-3f,
    };
    memcpy(in.accel_m_s2, accel, sizeof(in.accel_m_s2));
    memcpy(in.gyro_rad_s, gyro, sizeof(in.gyro_rad_s));
    v->out = EasyTVC_AppStep(&v->app, &in);
    if (v->out.flight.state == EASYTVC_FLIGHT_IDLE) v->pad.armed = false;
    if (!v->launched && v->out.flight.state == EASYTVC_FLIGHT_BOOST) {
        v->launched = true;
        v->launch_ms = now;
    }
    if (!v->chute_open && EasyTVC_ChuteShouldOpen(v->launched, now - v->launch_ms,
            v->out.flight.state, v->out.flight.drogue_deploy_requested, 7000U)) {
        v->chute_open = true;
        v->chute_ms = now;
    }
}

static void test_full_flight(void)
{
    Vehicle v;
    vehicle_init(&v);
    const float gyro_bias_true[3] = {0.012f, -0.008f, 0.004f};   /* ~0.7 deg/s, uncalibrated */
    uint32_t t = 0;

    /* 1. On the pad: still and vertical. Must arm after 3 s, not before, and learn the bias. */
    for (; t < 4000; t += DT_MS) {
        const float a[3] = {0, 0, G};
        vehicle_step(&v, t, a, gyro_bias_true, 0.0f);
        if (t < 2900) assert(v.out.flight.state == EASYTVC_FLIGHT_IDLE);
    }
    assert(v.out.flight.state == EASYTVC_FLIGHT_ARMED);
    assert(v.armed_ms >= 3000 && v.armed_ms <= 3100);
    for (unsigned i = 0; i < 3; ++i) assert(fabsf(v.bias[i] - gyro_bias_true[i]) < 1e-4f);
    assert(!v.out.control.enabled);

    /* 2. Boost: 3 g specific force (E12, ~0.4 kg) for 2.4 s, with a 0.6 rad/s kick about body X at ignition.
     *    The vehicle: theta'' = K * command (positive command = positive torque, as documented). */
    const float K = 31.0f;           /* rad/s^2 per full deflection: E12, ~0.15 m arm, 5 deg gimbal */
    float theta = 0.0f, omega = 0.6f, peak = 0.0f;
    float vz = 0.0f, h = 0.0f;
    bool saw_tvc = false, sign_ok = true;
    const uint32_t ignition = t;
    for (; t < ignition + 2400; t += DT_MS) {
        const float a[3] = {0, 0, 3.0f * G};
        const float gyro_raw[3] = {omega + gyro_bias_true[0], gyro_bias_true[1], gyro_bias_true[2]};
        vehicle_step(&v, t, a, gyro_raw, h);
        float cmd = 0.0f;
        if (v.out.control.enabled) {
            saw_tvc = true;
            cmd = v.out.control.command[0];
            if (fabsf(theta) > 0.02f && cmd * theta > 0.0f && cmd * omega > 0.0f) sign_ok = false;
        }
        omega += K * cmd * DT_MS * 1e-3f;
        theta += omega * DT_MS * 1e-3f;
        peak = fmaxf(peak, fabsf(theta));
        vz += (3.0f * G - G) * DT_MS * 1e-3f;
        h += vz * DT_MS * 1e-3f;
    }
    assert(v.launched && v.launch_ms - ignition <= 60);
    assert(saw_tvc && sign_ok);
    assert(v.out.flight.state == EASYTVC_FLIGHT_BOOST);
    assert(peak < 0.25f);                 /* uncontrolled it would be 0.6 rad/s * 2.4 s = 1.4 rad */
    assert(fabsf(theta) < 0.03f && fabsf(omega) < 0.1f);   /* settled back to vertical */
    printf("  boost: peak tilt %.1f deg, final %.2f deg\n", peak * 57.3f, theta * 57.3f);

    /* 3. Coast: drag gives -0.3 g. Apogee when vz crosses 0 (~3.7 s later). */
    bool coast_seen = false;
    for (; t < ignition + 9000 && !v.chute_open; t += DT_MS) {
        const float a[3] = {0, 0, -0.3f * G};
        const float gyro_raw[3] = {gyro_bias_true[0], gyro_bias_true[1], gyro_bias_true[2]};
        vehicle_step(&v, t, a, gyro_raw, h);
        coast_seen = coast_seen || v.out.flight.state == EASYTVC_FLIGHT_COAST;
        assert(!v.out.control.enabled);
        vz += (-0.3f * G - G) * DT_MS * 1e-3f;
        h += vz * DT_MS * 1e-3f;
    }
    assert(coast_seen);
    assert(v.chute_open);
    const float apogee_s = 2.4f + (2.0f * G * 2.4f) / (1.3f * G);
    const float chute_s = (float)(v.chute_ms - v.launch_ms) * 1e-3f;
    printf("  apogee at %.2f s, chute released at %.2f s after launch\n", apogee_s, chute_s);
    assert(chute_s > apogee_s - 0.3f && chute_s < apogee_s + 0.6f && chute_s < 7.0f);
}

static void test_backup_timer_opens_chute_after_fault(void)
{
    Vehicle v;
    vehicle_init(&v);
    const float zero[3] = {0};
    uint32_t t = 0;
    for (; t < 3500; t += DT_MS) {
        const float a[3] = {0, 0, G};
        vehicle_step(&v, t, a, zero, 0.0f);
    }
    assert(v.out.flight.state == EASYTVC_FLIGHT_ARMED);
    for (; t < 4000; t += DT_MS) {
        const float a[3] = {0, 0, 3.0f * G};
        vehicle_step(&v, t, a, zero, 0.0f);
    }
    assert(v.out.flight.state == EASYTVC_FLIGHT_BOOST);
    /* Tumble past the tilt limit: TVC gives up, state machine goes FAULT. */
    for (; t < 5000; t += DT_MS) {
        const float a[3] = {0, 0, 3.0f * G};
        const float g[3] = {2.0f, 0, 0};
        vehicle_step(&v, t, a, g, 0.0f);
    }
    assert(v.out.flight.state == EASYTVC_FLIGHT_FAULT && !v.out.control.enabled);
    assert(!v.chute_open);
    for (; t < 15000 && !v.chute_open; t += DT_MS) {
        const float a[3] = {0, 0, 0};
        vehicle_step(&v, t, a, zero, 0.0f);
    }
    assert(v.chute_open && v.chute_ms - v.launch_ms >= 7000 && v.chute_ms - v.launch_ms <= 7010);
}

/* Missed IMU reads in the middle of the burn (a failed SPI transfer reuses the last sample):
 * TVC must keep steering on its last command, not switch off for the rest of the flight. Only a
 * dropout longer than EASYTVC_CONTROL_HOLD_MS gives up. */
static void test_imu_dropout_in_boost(void)
{
    for (int run = 0; run < 2; ++run) {
        const uint32_t dropout_ms = run == 0 ? 15U : 150U;     /* 3 missed reads, then 30 */
        Vehicle v;
        vehicle_init(&v);
        const float zero[3] = {0};
        uint32_t t = 0;
        for (; t < 3500; t += DT_MS) {
            const float a[3] = {0, 0, G};
            vehicle_step(&v, t, a, zero, 0.0f);
        }
        for (; t < 4200; t += DT_MS) {
            const float a[3] = {0, 0, 3.0f * G};
            const float g[3] = {0.05f, 0, 0};                 /* a slow tilt the TVC is fighting */
            vehicle_step(&v, t, a, g, 0.0f);
        }
        assert(v.out.flight.state == EASYTVC_FLIGHT_BOOST && v.out.control.enabled);
        const float before = v.out.control.command[0];
        assert(before != 0.0f);
        const uint32_t start = t;
        for (; t < start + dropout_ms; t += DT_MS) {
            stale_sensor_age_ms = t - start + DT_MS;
            const float a[3] = {0, 0, 3.0f * G};
            const float g[3] = {0.05f, 0, 0};
            vehicle_step(&v, t, a, g, 0.0f);
            if (run == 0) assert(v.out.control.enabled && v.out.control.command[0] == before &&
                                 v.out.flight.state == EASYTVC_FLIGHT_BOOST);
        }
        stale_sensor_age_ms = 0;
        const float a[3] = {0, 0, 3.0f * G};
        const float g[3] = {0.05f, 0, 0};
        vehicle_step(&v, t, a, g, 0.0f);
        if (run == 0) {
            assert(v.out.flight.state == EASYTVC_FLIGHT_BOOST && v.out.control.enabled && !v.out.control.fault);
        } else {
            assert(v.out.flight.state == EASYTVC_FLIGHT_FAULT && !v.out.control.enabled);
        }
    }
    printf("flight: 15 ms IMU dropout in boost held the gimbal and kept TVC on; 150 ms gave up\n");
}

/* A barometer that freezes in flight must not drag the speed estimate to zero (a false apogee
 * while still climbing), and an attitude gap must not lose the rotation that happened in it. */
static void test_stale_baro_and_attitude_gap(void)
{
    EasyTVCNavFilter frozen, ignored;
    EasyTVC_NavInit(&frozen, 0.0f);
    EasyTVC_NavInit(&ignored, 0.0f);
    frozen.vertical_speed_m_s = ignored.vertical_speed_m_s = 19.0f;      /* burnout, coasting up */
    float lowest = 19.0f;
    for (unsigned i = 0; i < 100U; ++i) {                                 /* 0.5 s, baro stuck at 0 m */
        EasyTVC_NavUpdate(&frozen, 0.0f, true, -G, 0.005f);
        EasyTVC_NavUpdate(&ignored, 0.0f, false, -G, 0.005f);
        if (frozen.vertical_speed_m_s < lowest) lowest = frozen.vertical_speed_m_s;
    }
    assert(lowest <= 0.0f);                                               /* following it: a false apogee */
    assert(ignored.vertical_speed_m_s > 13.0f && ignored.vertical_speed_m_s < 15.0f);   /* 19 - g*0.5 */

    EasyTVCApp app;
    EasyTVC_AppInit(&app, 0);
    EasyTVCAppInput in = {.accel_m_s2 = {0, 0, G}, .gyro_rad_s = {0.5f, 0, 0}, .sensors_healthy = true,
                          .dt_s = 0.005f, .now_ms = 5};
    (void)EasyTVC_AppStep(&app, &in);
    in.sensors_healthy = false;                                           /* 40 ms with no usable IMU */
    for (in.now_ms = 10; in.now_ms <= 45; in.now_ms += 5) (void)EasyTVC_AppStep(&app, &in);
    in.sensors_healthy = true;
    in.now_ms = 50;
    in.accel_m_s2[2] = 3.0f * G;                                          /* no accel correction */
    (void)EasyTVC_AppStep(&app, &in);
    float tilt[2];
    EasyTVC_ControllerTilt(&app.attitude.orientation, tilt);
    assert(tilt[0] > 0.5f * 0.045f && tilt[0] < 0.5f * 0.055f);          /* ~0.5 rad/s over the whole 50 ms */
    puts("flight: frozen barometer ignored (no false apogee); attitude covers an IMU gap");
}

static void test_pad_safety(void)
{
    Vehicle v;
    vehicle_init(&v);
    const float zero[3] = {0};
    uint32_t t = 0;

    /* Leaning 15 deg never arms. */
    for (; t < 5000; t += DT_MS) {
        const float a[3] = {G * sinf(0.26f), 0, G * cosf(0.26f)};
        vehicle_step(&v, t, a, zero, 0.0f);
        assert(v.out.flight.state == EASYTVC_FLIGHT_IDLE);
    }
    /* Being handled (rotating) never arms. */
    for (; t < 10000; t += DT_MS) {
        const float a[3] = {0, 0, G};
        const float g[3] = {0.2f * sinf(t * 0.003f), 0, 0.1f};
        vehicle_step(&v, t, a, g, 0.0f);
        assert(v.out.flight.state == EASYTVC_FLIGHT_IDLE);
    }
    for (; t < 14000; t += DT_MS) {
        const float a[3] = {0, 0, G};
        vehicle_step(&v, t, a, zero, 0.0f);
    }
    assert(v.out.flight.state == EASYTVC_FLIGHT_ARMED);

    /* A 20 ms knock does not count as a launch. */
    for (uint32_t end = t + 20; t < end; t += DT_MS) {
        const float a[3] = {0, 0, 5.0f * G};
        vehicle_step(&v, t, a, zero, 0.0f);
    }
    for (uint32_t end = t + 500; t < end; t += DT_MS) {
        const float a[3] = {0, 0, G};
        vehicle_step(&v, t, a, zero, 0.0f);
    }
    assert(v.out.flight.state == EASYTVC_FLIGHT_ARMED && !v.launched);

    /* Picking it up (tipped 40 deg for > 0.5 s) disarms. */
    for (uint32_t end = t + 700; t < end; t += DT_MS) {
        const float a[3] = {G * sinf(0.7f), 0, G * cosf(0.7f)};
        vehicle_step(&v, t, a, zero, 0.0f);
    }
    assert(v.out.flight.state == EASYTVC_FLIGHT_IDLE && !v.pad.armed && !v.chute_open);
}

int main(void)
{
    test_imu_dropout_in_boost();
    test_stale_baro_and_attitude_gap();
    test_full_flight();
    test_backup_timer_opens_chute_after_fault();
    test_pad_safety();
    puts("flight tests passed: arming, bias capture, launch, closed-loop TVC, apogee, chute");
    return 0;
}
