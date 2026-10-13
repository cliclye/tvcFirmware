#include "flight_sim3d.h"

#include "app.h"
#include "controller.h"
#include "pad_arm.h"

#include <math.h>
#include <string.h>

#define G0          9.80665f
#define RHO         1.225f
#define DEG         0.01745329252f
#define PHYS_DT     0.001f
#define CONTROL_DIV 5U          /* 200 Hz flight software */
#define MAX_TIME_S  90.0f

typedef struct { float w, x, y, z; } Quat;

/* ---- small math ---- */

static Quat qmul(Quat a, Quat b)
{
    return (Quat){a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z, a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
                  a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x, a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w};
}

static void qnorm(Quat *q)
{
    const float n = sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    q->w /= n; q->x /= n; q->y /= n; q->z /= n;
}

/* body -> world */
static void rotate(Quat q, const float v[3], float out[3])
{
    const Quat p = {0, v[0], v[1], v[2]};
    const Quat r = qmul(qmul(q, p), (Quat){q.w, -q.x, -q.y, -q.z});
    out[0] = r.x; out[1] = r.y; out[2] = r.z;
}

/* world -> body */
static void unrotate(Quat q, const float v[3], float out[3])
{
    rotate((Quat){q.w, -q.x, -q.y, -q.z}, v, out);
}

static float norm3(const float v[3]) { return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]); }

static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

/* deterministic noise (xorshift + Box-Muller) */
static unsigned rng_state;
static float uniform(void)
{
    rng_state ^= rng_state << 13; rng_state ^= rng_state >> 17; rng_state ^= rng_state << 5;
    return ((rng_state >> 8) + 0.5f) / 16777216.0f;
}
static float gauss(void)
{
    return sqrtf(-2.0f * logf(uniform())) * cosf(6.283185307f * uniform());
}

/* Thrust curve with a unit mean, tau in [0, 1]. The "Estes" shape is an ignition spike of about
 * 2.1x the average falling to a 0.84x sustain, roughly like an E12. Use real curve data in
 * OpenRocket for anything precise. */
static float thrust_shape(int profile, float tau)
{
    if (tau < 0.0f || tau > 1.0f) return 0.0f;
    if (profile == 0) return 1.0f;
    const float sustain = 1.0f / 1.1875f, peak = 2.5f * sustain;
    if (tau < 0.04f) return peak * tau / 0.04f;
    if (tau < 0.12f) return peak;
    if (tau < 0.25f) return peak + (sustain - peak) * (tau - 0.12f) / 0.13f;
    if (tau < 0.92f) return sustain;
    return sustain * (1.0f - tau) / 0.08f;
}

static bool valid_params(const float *p)
{
    for (unsigned i = 0; i < SIM3D_PARAM_COUNT; ++i) if (!isfinite(p[i])) return false;
    return p[SIM3D_DRY_MASS_KG] > 0.01f && p[SIM3D_MOTOR_MASS_KG] >= p[SIM3D_PROP_MASS_KG] &&
        p[SIM3D_PROP_MASS_KG] >= 0.0f && p[SIM3D_AVG_THRUST_N] > 0.0f && p[SIM3D_BURN_S] > 0.05f &&
        p[SIM3D_BURN_S] < 30.0f && p[SIM3D_DIAMETER_M] > 0.005f && p[SIM3D_DIAMETER_M] < 1.0f &&
        p[SIM3D_LENGTH_M] > 0.05f && p[SIM3D_LENGTH_M] < 5.0f && p[SIM3D_GIMBAL_ARM_M] > 0.005f &&
        p[SIM3D_GIMBAL_ARM_M] < p[SIM3D_LENGTH_M] && p[SIM3D_INERTIA] >= 0.0f && p[SIM3D_INERTIA_BURNOUT] >= 0.0f &&
        p[SIM3D_LINKAGE_RATIO] > 0.005f && p[SIM3D_LINKAGE_RATIO] <= 1.0f &&
        p[SIM3D_SERVO_SPEED_DPS] > 1.0f && p[SIM3D_SERVO_DEG_PER_US] > 0.01f && p[SIM3D_SERVO_DEG_PER_US] < 0.5f &&
        p[SIM3D_GIMBAL_STOP_DEG] > 0.2f && p[SIM3D_GIMBAL_STOP_DEG] <= 30.0f && p[SIM3D_PLANFORM_AREA_M2] >= 0.0f &&
        fabsf(p[SIM3D_CG_SHIFT_M]) < p[SIM3D_LENGTH_M] && p[SIM3D_SERVO_RATE_HZ] >= 20.0f &&
        p[SIM3D_CHUTE_DIAMETER_M] >= 0.0f && p[SIM3D_WIND_M_S] >= 0.0f &&
        p[SIM3D_GUST_M_S] >= p[SIM3D_WIND_M_S] && p[SIM3D_IGNITION_S] >= 0.5f &&
        p[SIM3D_IGNITION_S] <= 60.0f && p[SIM3D_TOWER_M] >= 0.0f;
}

int EasyTVC_Sim3D(const float *p, unsigned param_count, float *rows, unsigned max_rows,
                  float *summary, unsigned summary_count)
{
    if (!p || param_count != SIM3D_PARAM_COUNT || !rows || max_rows < 2 || !summary ||
        summary_count != SIM3D_SUMMARY_COUNT || !valid_params(p)) return -1;

    /* ---- flight software, configured exactly like main.c apply_vehicle_config() ---- */
    static EasyTVCApp app;   /* large; keep it off the stack */
    EasyTVC_AppInit(&app, 0);
    EasyTVCSettings s;
    EasyTVC_SettingsDefault(&s);
    for (unsigned i = 0; i < 2; ++i) {
        s.axis[i] = (EasyTVCPidConfig){.kp = p[SIM3D_KP], .ki = p[SIM3D_KI], .kd = p[SIM3D_KD],
            .integrator_limit = p[SIM3D_INTEGRATOR_LIMIT], .output_limit = 1.0f,
            .output_rate_limit = p[SIM3D_RATE_LIMIT], .derivative_cutoff_hz = p[SIM3D_DERIVATIVE_CUTOFF]};
    }
    const float *servo_p[2] = {&p[SIM3D_A_MIN], &p[SIM3D_B_MIN]};
    for (unsigned i = 0; i < 2; ++i) {
        s.servo[i] = (EasyTVCServoSettings){.minimum_us = (uint16_t)servo_p[i][0],
            .center_us = (uint16_t)servo_p[i][1], .maximum_us = (uint16_t)servo_p[i][2],
            .channel = (uint8_t)(i + 1), .direction = (int8_t)(servo_p[i][3] < 0 ? -1 : 1)};
    }
    s.maximum_tilt_deg = p[SIM3D_MAX_TILT_DEG];
    app.flight_config.launch_accel_g = p[SIM3D_LAUNCH_G];
    app.flight_config.launch_confirm_ms = (uint32_t)p[SIM3D_LAUNCH_CONFIRM_MS];
    app.flight_config.tvc_enable_delay_ms = (uint32_t)p[SIM3D_TVC_DELAY_MS];
    app.flight_config.failsafe_apogee_ms = (uint32_t)p[SIM3D_CHUTE_BACKUP_MS];
    EasyTVC_FlightInit(&app.flight, &app.flight_config, 0);
    if (!EasyTVC_AppApplySettings(&app, &s, false)) return -2;

    EasyTVCPadArm pad;
    EasyTVC_PadArmInit(&pad);
    const EasyTVCPadArmConfig pad_config = {
        .hold_ms = (uint32_t)p[SIM3D_ARM_HOLD_MS], .cos_arm_tilt = cosf(p[SIM3D_ARM_TILT_DEG] * DEG),
        .cos_disarm_tilt = cosf(p[SIM3D_DISARM_TILT_DEG] * DEG), .still_gyro_rad_s = 0.05f,
        .disarm_confirm_ms = 500U};
    float fw_bias[3] = {0, 0, 0};
    bool launched = false, chute_open = false;
    uint32_t launch_ms = 0;

    /* ---- vehicle ---- */
    rng_state = 2463534242u ^ (unsigned)(p[SIM3D_SEED] * 7919.0f + 1.0f);
    const float noise = p[SIM3D_NOISE];
    const float area = 3.14159265f * 0.25f * p[SIM3D_DIAMETER_M] * p[SIM3D_DIAMETER_M];
    const float chute_area = 3.14159265f * 0.25f * p[SIM3D_CHUTE_DIAMETER_M] * p[SIM3D_CHUTE_DIAMETER_M];
    const float liftoff_mass = p[SIM3D_DRY_MASS_KG] + p[SIM3D_MOTOR_MASS_KG];
    const float L = p[SIM3D_LENGTH_M], r = 0.5f * p[SIM3D_DIAMETER_M];
    const float true_bias[3] = {p[SIM3D_GYRO_BIAS_DPS] * DEG, -0.7f * p[SIM3D_GYRO_BIAS_DPS] * DEG,
                                0.4f * p[SIM3D_GYRO_BIAS_DPS] * DEG};
    const float wind_dir = p[SIM3D_WIND_DIR_DEG] * DEG, gust_phase = 6.283f * uniform();
    float servo_dir_true[2] = {p[SIM3D_A_DIRECTION] < 0 ? -1.0f : 1.0f, p[SIM3D_B_DIRECTION] < 0 ? -1.0f : 1.0f};
    if (p[SIM3D_REVERSE_SERVO_A] > 0.5f) servo_dir_true[0] = -servo_dir_true[0];

    /* stand tilted about world X */
    const float pad_tilt = p[SIM3D_PAD_TILT_DEG] * DEG;
    Quat q = {cosf(pad_tilt / 2), sinf(pad_tilt / 2), 0, 0};
    float pos[3] = {0, 0, 0}, vel[3] = {0, 0, 0}, w[3] = {0, 0, 0};
    float rail_axis[3];
    { const float z[3] = {0, 0, 1}; rotate(q, z, rail_axis); }
    float gimbal[2] = {0, 0}, horn[2] = {0, 0}, horn_target[2] = {0, 0};
    uint16_t pulse_hold[2] = {s.servo[0].center_us, s.servo[1].center_us};
    bool off_pad = false, landed = false;
    float baro_alt = 0.0f, last_specific[3] = {0, 0, G0};

    for (unsigned i = 0; i < SIM3D_SUMMARY_COUNT; ++i) summary[i] = 0.0f;
    summary[SIM3D_SUM_CHUTE_T] = summary[SIM3D_SUM_ARMED_T] = summary[SIM3D_SUM_LAUNCH_T] = -1.0f;
    unsigned tvc_ticks = 0, saturated_ticks = 0, row_count = 0;
    float chute_open_time = -1.0f, land_time = -1.0f;
    const unsigned servo_period = (unsigned)lroundf(1.0f / (p[SIM3D_SERVO_RATE_HZ] * PHYS_DT));
    const float ignition = p[SIM3D_IGNITION_S];

    for (unsigned step = 0;; ++step) {
        const float t = step * PHYS_DT;
        const float t_burn = t - ignition;
        float impulse_fraction = 0.0f;
        {
            /* integrate the unit-mean shape up to t_burn (cheap: 64 samples) */
            const float tau = clampf(t_burn / p[SIM3D_BURN_S], 0.0f, 1.0f);
            float acc = 0.0f;
            for (unsigned k = 0; k < 64U; ++k) acc += thrust_shape((int)p[SIM3D_THRUST_PROFILE], tau * (k + 0.5f) / 64.0f);
            impulse_fraction = acc / 64.0f * tau;
        }
        const float mass = liftoff_mass - p[SIM3D_PROP_MASS_KG] * clampf(impulse_fraction, 0.0f, 1.0f);
        /* CG moves forward as propellant burns: arm and CP distance follow it */
        const float frac = clampf(impulse_fraction, 0.0f, 1.0f);
        const float inertia_wet = p[SIM3D_INERTIA] > 0.0f ? p[SIM3D_INERTIA] : liftoff_mass * (L * L / 12.0f + r * r / 4.0f);
        const float dry_mass = liftoff_mass - p[SIM3D_PROP_MASS_KG];
        const float inertia_dry = p[SIM3D_INERTIA_BURNOUT] > 0.0f ? p[SIM3D_INERTIA_BURNOUT] : dry_mass * (L * L / 12.0f + r * r / 4.0f);
        const float inertia = inertia_wet + (inertia_dry - inertia_wet) * frac;
        const float cg_shift = p[SIM3D_CG_SHIFT_M] * frac;
        const float inertia_z = fmaxf(0.5f * mass * r * r, 1e-6f);
        const float thrust = t_burn > 0 ? p[SIM3D_AVG_THRUST_N] * thrust_shape((int)p[SIM3D_THRUST_PROFILE], t_burn / p[SIM3D_BURN_S]) : 0.0f;

        /* ---- wind ---- */
        const float gust = fmaxf(0.0f, sinf(0.9f * t + gust_phase)) * fmaxf(0.0f, sinf(0.37f * t + 1.3f * gust_phase));
        const float wind_speed = p[SIM3D_WIND_M_S] + (p[SIM3D_GUST_M_S] - p[SIM3D_WIND_M_S]) * gust;
        const float h_factor = pos[2] > 1.0f ? powf(pos[2], 0.14f) / powf(10.0f, 0.14f) : 0.72f;   /* wind shear */
        const float wind[3] = {wind_speed * h_factor * cosf(wind_dir), wind_speed * h_factor * sinf(wind_dir), 0};

        /* ---- forces in body frame ---- */
        const float da = gimbal[0] + p[SIM3D_MISALIGN_DEG] * DEG, db = gimbal[1];
        const float thrust_b[3] = {-thrust * sinf(db), thrust * sinf(da) * cosf(db), thrust * cosf(da) * cosf(db)};
        const float v_air_w[3] = {vel[0] - wind[0], vel[1] - wind[1], vel[2] - wind[2]};
        float v_air_b[3];
        unrotate(q, v_air_w, v_air_b);
        const float speed_air = norm3(v_air_w);
        float aero_b[3] = {0, 0, 0}, normal_b[3] = {0, 0, 0}, lift_b[3] = {0, 0, 0};
        const float v_lat = sqrtf(v_air_b[0] * v_air_b[0] + v_air_b[1] * v_air_b[1]);
        const float alpha = speed_air > 0.5f ? atan2f(v_lat, fabsf(v_air_b[2])) : 0.0f;
        if (off_pad && !chute_open) {
            aero_b[2] = -0.5f * RHO * p[SIM3D_BODY_CD] * area * speed_air * v_air_b[2];
            /* Barrowman nose normal force, linear in alpha, at the CP */
            normal_b[0] = -0.5f * RHO * p[SIM3D_CN_ALPHA] * area * speed_air * v_air_b[0];
            normal_b[1] = -0.5f * RHO * p[SIM3D_CN_ALPHA] * area * speed_air * v_air_b[1];
            /* Body lift (Galejs): 1.1 * planform/ref * sin^2(alpha), at the planform centroid */
            if (v_lat > 1e-4f) {
                const float s2 = sinf(alpha) * sinf(alpha);
                const float lift = 0.5f * RHO * speed_air * speed_air * 1.1f * p[SIM3D_PLANFORM_AREA_M2] * s2;
                lift_b[0] = -lift * v_air_b[0] / v_lat;
                lift_b[1] = -lift * v_air_b[1] / v_lat;
            }
        }
        float force_b[3] = {thrust_b[0] + aero_b[0] + normal_b[0] + lift_b[0],
                            thrust_b[1] + aero_b[1] + normal_b[1] + lift_b[1],
                            thrust_b[2] + aero_b[2] + normal_b[2]};
        float force_w[3];
        rotate(q, force_b, force_w);
        if (chute_open && off_pad) {
            const float inflate = clampf((t - chute_open_time) / 0.6f, 0.0f, 1.0f);
            const float k = 0.5f * RHO * (p[SIM3D_CHUTE_CD] * chute_area * inflate + p[SIM3D_BODY_CD] * area) * speed_air;
            for (unsigned i = 0; i < 3; ++i) force_w[i] += -k * v_air_w[i];
        }

        /* ---- moments in body frame: thrust at the gimbal, normal force at the CP, damping ---- */
        const float l = p[SIM3D_GIMBAL_ARM_M] + cg_shift, d = p[SIM3D_CP_AHEAD_M] - cg_shift;
        const float dp = p[SIM3D_PLANFORM_AHEAD_M] - cg_shift;
        float torque[3] = {l * thrust_b[1] - d * normal_b[1] - dp * lift_b[1],
                           -l * thrust_b[0] + d * normal_b[0] + dp * lift_b[0], 0};
        const float damping = 0.5f * RHO * speed_air * area * L * L * 0.6f;
        torque[0] -= damping * w[0];
        torque[1] -= damping * w[1];

        /* ---- integrate ---- */
        const float weight = mass * G0;
        if (!off_pad) {
            /* on the stand: held until thrust along the rail beats gravity */
            const float along = force_w[0] * rail_axis[0] + force_w[1] * rail_axis[1] + force_w[2] * rail_axis[2];
            if (along > weight * rail_axis[2] + 1e-3f) off_pad = true;
        }
        const float on_rail = off_pad && (pos[0] * rail_axis[0] + pos[1] * rail_axis[1] + pos[2] * rail_axis[2]) < p[SIM3D_TOWER_M];
        float accel_w[3] = {force_w[0] / mass, force_w[1] / mass, force_w[2] / mass - G0};
        if (!off_pad) {
            accel_w[0] = accel_w[1] = accel_w[2] = 0.0f;
        } else if (on_rail) {
            const float a = accel_w[0] * rail_axis[0] + accel_w[1] * rail_axis[1] + accel_w[2] * rail_axis[2];
            for (unsigned i = 0; i < 3; ++i) accel_w[i] = a * rail_axis[i];
        }
        if (!landed) {
            for (unsigned i = 0; i < 3; ++i) { vel[i] += accel_w[i] * PHYS_DT; pos[i] += vel[i] * PHYS_DT; }
            if (off_pad && !on_rail) {
                const float Iw[3] = {inertia * w[0], inertia * w[1], inertia_z * w[2]};
                const float gyro_term[3] = {w[1]*Iw[2] - w[2]*Iw[1], w[2]*Iw[0] - w[0]*Iw[2], w[0]*Iw[1] - w[1]*Iw[0]};
                w[0] += (torque[0] - gyro_term[0]) / inertia * PHYS_DT;
                w[1] += (torque[1] - gyro_term[1]) / inertia * PHYS_DT;
                w[2] += (torque[2] - gyro_term[2]) / inertia_z * PHYS_DT;
                if (chute_open) for (unsigned i = 0; i < 3; ++i) w[i] *= 0.995f;
                const Quat dq = {0, w[0], w[1], w[2]};
                const Quat qd = qmul(q, dq);
                q.w += 0.5f * qd.w * PHYS_DT; q.x += 0.5f * qd.x * PHYS_DT;
                q.y += 0.5f * qd.y * PHYS_DT; q.z += 0.5f * qd.z * PHYS_DT;
                qnorm(&q);
            }
            if (off_pad && pos[2] <= 0.0f && t_burn > 0.3f && vel[2] < 0.0f) {
                landed = true;
                land_time = t;
                summary[SIM3D_SUM_LANDING_SPEED] = norm3(vel);
                summary[SIM3D_SUM_DRIFT_M] = sqrtf(pos[0] * pos[0] + pos[1] * pos[1]);
                summary[SIM3D_SUM_UNDER_CHUTE] = chute_open ? 1.0f : 0.0f;
                pos[2] = 0.0f;
                vel[0] = vel[1] = vel[2] = 0.0f;
                w[0] = w[1] = w[2] = 0.0f;
            }
        }

        /* specific force the accelerometer would feel (body frame) */
        float specific_w[3] = {accel_w[0], accel_w[1], accel_w[2] + G0};
        if (!off_pad || landed) { specific_w[0] = 0; specific_w[1] = 0; specific_w[2] = G0; }
        unrotate(q, specific_w, last_specific);

        /* ---- servos: PWM frame rate, horn slew, linkage, mechanical stop ---- */
        for (unsigned i = 0; i < 2; ++i) {
            const float step_max = p[SIM3D_SERVO_SPEED_DPS] * DEG * PHYS_DT;
            horn[i] += clampf(horn_target[i] - horn[i], -step_max, step_max);
            const float stop = p[SIM3D_GIMBAL_STOP_DEG] * DEG;
            gimbal[i] = clampf(asinf(clampf(p[SIM3D_LINKAGE_RATIO] * sinf(horn[i]), -1.0f, 1.0f)), -stop, stop);
        }

        /* ---- flight software at 200 Hz ---- */
        if (step % CONTROL_DIV == 0U) {
            const uint32_t now_ms = (uint32_t)lroundf(t * 1000.0f);
            float accel[3], gyro_raw[3], gyro[3];
            /* BMI088-like noise (~4 mg, ~0.15 deg/s RMS), plus motor vibration while burning. */
            const float vib = thrust > 0.0f ? 1.0f : 0.0f;
            for (unsigned i = 0; i < 3; ++i) {
                accel[i] = clampf(last_specific[i] + noise * (0.004f + 0.3f * vib) * G0 * gauss(), -24.0f * G0, 24.0f * G0);
                gyro_raw[i] = w[i] + true_bias[i] + noise * (0.0026f + 0.03f * vib) * gauss();
                gyro[i] = gyro_raw[i] - fw_bias[i];
            }
            if ((step / CONTROL_DIV) % 4U == 0U) baro_alt = pos[2] + noise * 0.25f * gauss();
            float new_bias[3];
            if (EasyTVC_PadArmStep(&pad, &pad_config, app.flight.state, true, true, accel, gyro, gyro_raw,
                                   now_ms, new_bias)) {
                for (unsigned i = 0; i < 3; ++i) {
                    fw_bias[i] = new_bias[i];
                    gyro[i] = gyro_raw[i] - fw_bias[i];
                    app.attitude.integral_error[i] = 0.0f;
                }
                if (summary[SIM3D_SUM_ARMED_T] < 0) summary[SIM3D_SUM_ARMED_T] = t;
            }
            EasyTVCAppInput in = {.baro_altitude_m = baro_alt, .sensors_healthy = true,
                .physical_arm_qualified = pad.armed, .software_arm_request = pad.armed,
                .accel_is_gravity_reference = true, .now_ms = now_ms, .sensor_age_ms = 0,
                .dt_s = CONTROL_DIV * PHYS_DT};
            memcpy(in.accel_m_s2, accel, sizeof(accel));
            memcpy(in.gyro_rad_s, gyro, sizeof(gyro));
            const EasyTVCAppOutput out = EasyTVC_AppStep(&app, &in);
            if (out.flight.state == EASYTVC_FLIGHT_IDLE) pad.armed = false;
            if (!launched && out.flight.state == EASYTVC_FLIGHT_BOOST) {
                launched = true;
                launch_ms = now_ms;
                summary[SIM3D_SUM_LAUNCH_T] = t - ignition;
            }
            if (out.control.fault) summary[SIM3D_SUM_TVC_FAULT] = 1.0f;
            if (out.control.enabled) {
                ++tvc_ticks;
                if (fabsf(out.control.command[0]) > 0.98f || fabsf(out.control.command[1]) > 0.98f) ++saturated_ticks;
            }
            if (p[SIM3D_CHUTE_ENABLED] > 0.5f && !chute_open &&
                EasyTVC_ChuteShouldOpen(launched, now_ms - launch_ms, out.flight.state,
                                        out.flight.drogue_deploy_requested, (uint32_t)p[SIM3D_CHUTE_BACKUP_MS])) {
                chute_open = true;
                chute_open_time = t;
                summary[SIM3D_SUM_CHUTE_T] = t - ignition;
                summary[SIM3D_SUM_CHUTE_SPEED] = norm3(vel);
            }
            if (step % (servo_period ? servo_period : 1U) < CONTROL_DIV) {
                pulse_hold[0] = out.control.pulse_us[0];
                pulse_hold[1] = out.control.pulse_us[1];
            }
            for (unsigned i = 0; i < 2; ++i) {
                /* Pulse -> horn angle as the hardware does it. servo_dir_true is how the servo is
                 * really installed: the configured direction after a correct bench test, flipped
                 * when simulating a reversed servo. */
                const float offset = (float)pulse_hold[i] - s.servo[i].center_us;
                horn_target[i] = servo_dir_true[i] * offset * p[SIM3D_SERVO_DEG_PER_US] * DEG;
            }

            /* ---- record ---- */
            const float tilt_now = acosf(clampf(1.0f - 2.0f * (q.x * q.x + q.y * q.y), -1.0f, 1.0f)) / DEG;
            if (out.flight.state == EASYTVC_FLIGHT_BOOST) {
                summary[SIM3D_SUM_MAX_BOOST_TILT] = fmaxf(summary[SIM3D_SUM_MAX_BOOST_TILT], tilt_now);
                float est[2], truth[2];
                EasyTVC_ControllerTilt(&app.attitude.orientation, est);
                const EasyTVCQuaternion tq = {q.w, q.x, q.y, q.z};
                EasyTVC_ControllerTilt(&tq, truth);
                summary[SIM3D_SUM_EST_ERROR_DEG] = fmaxf(summary[SIM3D_SUM_EST_ERROR_DEG],
                    fmaxf(fabsf(est[0] - truth[0]), fabsf(est[1] - truth[1])) / DEG);
            }
            if (pos[2] > summary[SIM3D_SUM_APOGEE_M]) {
                summary[SIM3D_SUM_APOGEE_M] = pos[2];
                summary[SIM3D_SUM_APOGEE_T] = t - ignition;
            }
            summary[SIM3D_SUM_MAX_SPEED] = fmaxf(summary[SIM3D_SUM_MAX_SPEED], norm3(vel));
            summary[SIM3D_SUM_FINAL_STATE] = (float)out.flight.state;
            if (off_pad) summary[SIM3D_SUM_LIFTOFF] = 1.0f;

            if ((step % (unsigned)lroundf(SIM3D_ROW_PERIOD_S / PHYS_DT)) == 0U && row_count < max_rows) {
                float *row = rows + row_count * SIM3D_COLUMN_COUNT;
                float est[2], truth[2];
                EasyTVC_ControllerTilt(&app.attitude.orientation, est);
                const EasyTVCQuaternion tq = {q.w, q.x, q.y, q.z};
                EasyTVC_ControllerTilt(&tq, truth);
                row[SIM3D_COL_T] = t; row[SIM3D_COL_X] = pos[0]; row[SIM3D_COL_Y] = pos[1]; row[SIM3D_COL_Z] = pos[2];
                row[SIM3D_COL_QW] = q.w; row[SIM3D_COL_QX] = q.x; row[SIM3D_COL_QY] = q.y; row[SIM3D_COL_QZ] = q.z;
                row[SIM3D_COL_SPEED] = norm3(vel); row[SIM3D_COL_VZ] = vel[2];
                row[SIM3D_COL_TILT_X] = truth[0] / DEG; row[SIM3D_COL_TILT_Y] = truth[1] / DEG;
                row[SIM3D_COL_GIMBAL_A] = gimbal[0] / DEG; row[SIM3D_COL_GIMBAL_B] = gimbal[1] / DEG;
                row[SIM3D_COL_STATE] = (float)out.flight.state; row[SIM3D_COL_TVC] = out.control.enabled ? 1.0f : 0.0f;
                row[SIM3D_COL_CHUTE] = chute_open ? 1.0f : 0.0f; row[SIM3D_COL_THRUST] = thrust;
                row[SIM3D_COL_EST_TILT_X] = est[0] / DEG; row[SIM3D_COL_EST_TILT_Y] = est[1] / DEG;
                row[SIM3D_COL_ARMED] = pad.armed ? 1.0f : 0.0f;
                row[SIM3D_COL_PULSE_A] = pulse_hold[0]; row[SIM3D_COL_PULSE_B] = pulse_hold[1];
                row[SIM3D_COL_HORN_A] = horn[0] / DEG; row[SIM3D_COL_HORN_B] = horn[1] / DEG;
                row[SIM3D_COL_ALPHA] = alpha / DEG;
                ++row_count;
            }
        }

        const bool done = (landed && t - land_time > 1.0f) || t > MAX_TIME_S || row_count >= max_rows ||
                          (!off_pad && t_burn > p[SIM3D_BURN_S] + 2.0f);
        if (done) break;
    }
    summary[SIM3D_SUM_SATURATION_PCT] = tvc_ticks ? 100.0f * saturated_ticks / tvc_ticks : 0.0f;
    summary[SIM3D_SUM_FLIGHT_TIME] = land_time >= 0 ? land_time - ignition : -1.0f;
    return (int)row_count;
}
