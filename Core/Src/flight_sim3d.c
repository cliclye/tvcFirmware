#include "flight_sim3d.h"

#include "app.h"
#include "controller.h"
#include "pad_arm.h"

#include <math.h>
#include <string.h>

/*
 * Closed loop, every millisecond:
 *
 *   rocket state --> simulated IMU / baro (noise, bias, vibration)
 *        ^                       |
 *        |                       v  200 Hz
 *   rigid-body dynamics     REAL flight software (pad arm, attitude, FSM, PID)  -> pulse us
 *        ^                                                                          |
 *        |                                                                          v
 *   torque = thrust x gimbal arm + aero   <--  gimbal  <--  linkage  <--  servo model (frame
 *                                                                          rate, deadband, loop,
 *                                                                          speed, accel, backlash)
 *
 * Nothing about the rocket's attitude is scripted: it only turns because of the moments.
 */

#define G0          9.80665f
#define RHO         1.225f
#define DEG         0.01745329252f
#define PHYS_DT     0.001f
#define CONTROL_DIV 5U          /* 200 Hz flight software */
#define MAX_TIME_S  90.0f
#define ROW_DIV     20U         /* 50 Hz output rows */
#define SERVO_ACCEL_DPS2 40000.0f   /* motor + gear inertia: ~15 ms to reach full speed */

typedef struct { float w, x, y, z; } Quat;

typedef struct {
    float p[SIM3D_PARAM_COUNT];
    /* flight software */
    EasyTVCApp app;
    EasyTVCSettings settings;
    EasyTVCPadArm pad;
    EasyTVCPadArmConfig pad_config;
    float fw_bias[3];
    bool launched, chute_open;
    uint32_t launch_ms;
    /* vehicle */
    Quat q;
    float pos[3], vel[3], w[3], rail_axis[3];
    float true_bias[3], servo_dir_true[2], wind_dir, gust_phase, liftoff_mass;
    bool off_pad, landed;
    float baro_alt, last_specific[3];
    /* servos: PWM latch -> deadband -> setpoint -> position loop (speed + accel limit) -> gears with
     * backlash -> horn -> linkage -> gimbal */
    uint16_t pulse_latched[2];
    float setpoint[2];         /* horn angle the servo is trying to reach, rad */
    float motor[2];            /* gear-train input angle, rad */
    float motor_rate[2];       /* rad/s */
    float horn[2];             /* output (after backlash), rad */
    float gimbal[2];
    bool jammed[2];
    /* disturbances */
    float gust_extra, gust_until;
    float turb[3];             /* Dryden-like turbulence, world frame, m/s */
    /* manual TVC */
    bool manual;
    float manual_cmd[2];
    /* sensors: BMI088 internal low-pass filters */
    float gyro_f[3], accel_f[3];
    /* gimbal dynamics for the reaction torque */
    float gimbal_prev[2], gimbal_rate[2], gimbal_acc[2];
    float w_dot[3];
    float rho, wind_h;
    /* bookkeeping */
    unsigned step, tvc_ticks, saturated_ticks;
    float chute_open_time, land_time;
    float summary[SIM3D_SUMMARY_COUNT];
    bool done, ready;
    unsigned rng;
    /* last moments for logging */
    float tq_tvc[2], tq_aero[2], alpha;
    float thrust;
} Sim;

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
static float uniform(Sim *s)
{
    s->rng ^= s->rng << 13; s->rng ^= s->rng >> 17; s->rng ^= s->rng << 5;
    return ((s->rng >> 8) + 0.5f) / 16777216.0f;
}
static float gauss(Sim *s)
{
    return sqrtf(-2.0f * logf(uniform(s))) * cosf(6.283185307f * uniform(s));
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

/* measured thrust curve (RASP .eng), shared by every run */
#define CURVE_MAX 130
static float curve_t[CURVE_MAX], curve_f[CURVE_MAX], curve_cum[CURVE_MAX];
static unsigned curve_n;

int EasyTVC_Sim3DSetCurve(const float *times, const float *newtons, unsigned count)
{
    if (!times || !newtons || count < 2 || count > CURVE_MAX - 2) return -1;
    float last = 0.0f;
    for (unsigned i = 0; i < count; ++i) {
        if (!isfinite(times[i]) || !isfinite(newtons[i]) || newtons[i] < 0.0f || times[i] <= last || times[i] > 30.0f) return -1;
        last = times[i];
    }
    /* RASP curves start implicitly at (0, 0) */
    curve_t[0] = 0.0f; curve_f[0] = 0.0f; curve_cum[0] = 0.0f;
    for (unsigned i = 0; i < count; ++i) { curve_t[i + 1] = times[i]; curve_f[i + 1] = newtons[i]; }
    curve_n = count + 1;
    for (unsigned i = 1; i < curve_n; ++i)
        curve_cum[i] = curve_cum[i - 1] + 0.5f * (curve_f[i] + curve_f[i - 1]) * (curve_t[i] - curve_t[i - 1]);
    return curve_cum[curve_n - 1] > 0.0f ? 0 : -1;
}

static float burn_time(const float *p)
{
    return (int)p[SIM3D_THRUST_PROFILE] == 2 && curve_n > 1 ? curve_t[curve_n - 1] : p[SIM3D_BURN_S];
}

/* thrust (N) and the fraction of the total impulse delivered so far, at t seconds after ignition */
static void motor_at(const float *p, float t, float *thrust, float *fraction)
{
    if (t <= 0.0f) { *thrust = 0.0f; *fraction = 0.0f; return; }
    const int profile = (int)p[SIM3D_THRUST_PROFILE];
    if (profile == 2 && curve_n > 1) {
        if (t >= curve_t[curve_n - 1]) { *thrust = 0.0f; *fraction = 1.0f; return; }
        unsigned i = 1;
        while (curve_t[i] < t) ++i;
        const float u = (t - curve_t[i - 1]) / (curve_t[i] - curve_t[i - 1]);
        const float f = curve_f[i - 1] + u * (curve_f[i] - curve_f[i - 1]);
        const float imp = curve_cum[i - 1] + 0.5f * (curve_f[i - 1] + f) * (t - curve_t[i - 1]);
        *thrust = f * p[SIM3D_THRUST_SCALE];
        *fraction = imp / curve_cum[curve_n - 1];
        return;
    }
    const float burn = p[SIM3D_BURN_S], tau = clampf(t / burn, 0.0f, 1.0f);
    float acc = 0.0f;   /* integrate the unit-mean shape up to t (cheap: 64 samples) */
    for (unsigned k = 0; k < 64U; ++k) acc += thrust_shape(profile, tau * (k + 0.5f) / 64.0f);
    *fraction = acc / 64.0f * tau;
    *thrust = p[SIM3D_AVG_THRUST_N] * p[SIM3D_THRUST_SCALE] * thrust_shape(profile, t / burn);
}

/* International Standard Atmosphere, anchored to the site's altitude and temperature */
static float air_density(const float *p, float height)
{
    const float t_site = p[SIM3D_TEMP_C] + 273.15f;
    const float t_here = t_site - 0.0065f * height;
    const float p_site = 101325.0f * powf(1.0f - 2.25577e-5f * p[SIM3D_SITE_ALT_M], 5.25588f);
    const float pressure = p_site * powf(t_here / t_site, 5.25588f);
    return pressure / (287.05f * t_here);
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
        p[SIM3D_IGNITION_S] <= 60.0f && p[SIM3D_TOWER_M] >= 0.0f &&
        p[SIM3D_SERVO_DEADBAND_US] >= 0.0f && p[SIM3D_SERVO_DEADBAND_US] <= 50.0f &&
        p[SIM3D_SERVO_BACKLASH_DEG] >= 0.0f && p[SIM3D_SERVO_BACKLASH_DEG] <= 10.0f &&
        p[SIM3D_SERVO_TAU_S] >= 0.001f && p[SIM3D_SERVO_TAU_S] <= 0.2f &&
        p[SIM3D_THRUST_SCALE] > 0.5f && p[SIM3D_THRUST_SCALE] < 1.5f &&
        p[SIM3D_SITE_ALT_M] > -500.0f && p[SIM3D_SITE_ALT_M] < 5000.0f && p[SIM3D_TEMP_C] > -40.0f && p[SIM3D_TEMP_C] < 60.0f &&
        p[SIM3D_NOZZLE_ARM_M] >= 0.0f && p[SIM3D_NOZZLE_ARM_M] < p[SIM3D_LENGTH_M] &&
        p[SIM3D_MOTOR_GIMBAL_INERTIA] >= 0.0f && p[SIM3D_MOTOR_GIMBAL_INERTIA] < 0.1f &&
        p[SIM3D_SERVO_STALL_NM] > 0.005f && p[SIM3D_SERVO_STALL_NM] < 5.0f &&
        fabsf(p[SIM3D_IMU_AHEAD_M]) < p[SIM3D_LENGTH_M] && p[SIM3D_TURBULENCE_M_S] >= 0.0f && p[SIM3D_TURBULENCE_M_S] <= 10.0f &&
        p[SIM3D_GYRO_LPF_HZ] >= 5.0f && p[SIM3D_GYRO_LPF_HZ] <= 500.0f && p[SIM3D_ACCEL_LPF_HZ] >= 5.0f && p[SIM3D_ACCEL_LPF_HZ] <= 500.0f &&
        ((int)p[SIM3D_THRUST_PROFILE] != 2 || curve_n > 1);
}

/* ---------------------------------------------------------------------------------------------
 * setup: the flight software is configured exactly like main.c apply_vehicle_config()
 * --------------------------------------------------------------------------------------------- */
static int sim_begin(Sim *s, const float *p, unsigned param_count)
{
    s->ready = false;
    if (!p || param_count != SIM3D_PARAM_COUNT || !valid_params(p)) return -1;
    memset(s, 0, sizeof(*s));
    memcpy(s->p, p, sizeof(s->p));

    EasyTVC_AppInit(&s->app, 0);
    EasyTVC_SettingsDefault(&s->settings);
    for (unsigned i = 0; i < 2; ++i) {
        s->settings.axis[i] = (EasyTVCPidConfig){.kp = p[SIM3D_KP], .ki = p[SIM3D_KI], .kd = p[SIM3D_KD],
            .integrator_limit = p[SIM3D_INTEGRATOR_LIMIT], .output_limit = 1.0f,
            .output_rate_limit = p[SIM3D_RATE_LIMIT], .derivative_cutoff_hz = p[SIM3D_DERIVATIVE_CUTOFF]};
    }
    const float *servo_p[2] = {&p[SIM3D_A_MIN], &p[SIM3D_B_MIN]};
    for (unsigned i = 0; i < 2; ++i) {
        s->settings.servo[i] = (EasyTVCServoSettings){.minimum_us = (uint16_t)servo_p[i][0],
            .center_us = (uint16_t)servo_p[i][1], .maximum_us = (uint16_t)servo_p[i][2],
            .channel = (uint8_t)(i + 1), .direction = (int8_t)(servo_p[i][3] < 0 ? -1 : 1)};
    }
    s->settings.maximum_tilt_deg = p[SIM3D_MAX_TILT_DEG];
    s->app.flight_config.launch_accel_g = p[SIM3D_LAUNCH_G];
    s->app.flight_config.launch_confirm_ms = (uint32_t)p[SIM3D_LAUNCH_CONFIRM_MS];
    s->app.flight_config.tvc_enable_delay_ms = (uint32_t)p[SIM3D_TVC_DELAY_MS];
    s->app.flight_config.failsafe_apogee_ms = (uint32_t)p[SIM3D_CHUTE_BACKUP_MS];
    EasyTVC_FlightInit(&s->app.flight, &s->app.flight_config, 0);
    if (!EasyTVC_AppApplySettings(&s->app, &s->settings, false)) return -2;

    EasyTVC_PadArmInit(&s->pad);
    s->pad_config = (EasyTVCPadArmConfig){
        .hold_ms = (uint32_t)p[SIM3D_ARM_HOLD_MS], .cos_arm_tilt = cosf(p[SIM3D_ARM_TILT_DEG] * DEG),
        .cos_disarm_tilt = cosf(p[SIM3D_DISARM_TILT_DEG] * DEG), .still_gyro_rad_s = 0.05f,
        .disarm_confirm_ms = 500U};

    s->rng = 2463534242u ^ (unsigned)(p[SIM3D_SEED] * 7919.0f + 1.0f);
    s->liftoff_mass = p[SIM3D_DRY_MASS_KG] + p[SIM3D_MOTOR_MASS_KG];
    s->true_bias[0] = p[SIM3D_GYRO_BIAS_DPS] * DEG;
    s->true_bias[1] = -0.7f * p[SIM3D_GYRO_BIAS_DPS] * DEG;
    s->true_bias[2] = 0.4f * p[SIM3D_GYRO_BIAS_DPS] * DEG;
    s->wind_dir = p[SIM3D_WIND_DIR_DEG] * DEG;
    s->gust_phase = 6.283f * uniform(s);
    s->servo_dir_true[0] = p[SIM3D_A_DIRECTION] < 0 ? -1.0f : 1.0f;
    s->servo_dir_true[1] = p[SIM3D_B_DIRECTION] < 0 ? -1.0f : 1.0f;
    if (p[SIM3D_REVERSE_SERVO_A] > 0.5f) s->servo_dir_true[0] = -s->servo_dir_true[0];

    const float pad_tilt = p[SIM3D_PAD_TILT_DEG] * DEG;   /* stand tilted about world X */
    s->q = (Quat){cosf(pad_tilt / 2), sinf(pad_tilt / 2), 0, 0};
    { const float z[3] = {0, 0, 1}; rotate(s->q, z, s->rail_axis); }
    s->last_specific[2] = G0;
    s->accel_f[2] = G0;
    s->pulse_latched[0] = s->settings.servo[0].center_us;
    s->pulse_latched[1] = s->settings.servo[1].center_us;
    s->summary[SIM3D_SUM_CHUTE_T] = s->summary[SIM3D_SUM_ARMED_T] = s->summary[SIM3D_SUM_LAUNCH_T] = -1.0f;
    s->chute_open_time = s->land_time = -1.0f;
    s->ready = true;
    return 0;
}

/* ---------------------------------------------------------------------------------------------
 * the servo, as hardware: it latches a pulse once per PWM frame, ignores changes inside its
 * deadband, runs a position loop limited by motor speed and acceleration, and drives the horn
 * through gears with backlash. The gimbal's mechanical stop also stops the horn.
 * --------------------------------------------------------------------------------------------- */
static void servo_step(Sim *s, unsigned i)
{
    const float *p = s->p;
    const EasyTVCServoSettings *sv = &s->settings.servo[i];
    if (!s->jammed[i]) {
        const float target = s->servo_dir_true[i] * ((float)s->pulse_latched[i] - sv->center_us) * p[SIM3D_SERVO_DEG_PER_US] * DEG;
        const float deadband = p[SIM3D_SERVO_DEADBAND_US] * p[SIM3D_SERVO_DEG_PER_US] * DEG;
        if (fabsf(target - s->setpoint[i]) > deadband) s->setpoint[i] = target;
        /* torque the horn needs to swing the motor: tau_h = I_motor * ratio^2 * alpha_h (virtual work);
         * a real servo slows as its load approaches stall torque */
        const float ratio = p[SIM3D_LINKAGE_RATIO];
        const float load_inertia = p[SIM3D_MOTOR_GIMBAL_INERTIA] * ratio * ratio + 2e-7f;   /* + gear-train rotor */
        const float amax = fminf(SERVO_ACCEL_DPS2 * DEG, p[SIM3D_SERVO_STALL_NM] / load_inertia);
        const float load = load_inertia * fabsf(s->motor_rate[i]) / p[SIM3D_SERVO_TAU_S];
        const float vmax = p[SIM3D_SERVO_SPEED_DPS] * DEG * clampf(1.0f - load / p[SIM3D_SERVO_STALL_NM], 0.05f, 1.0f);
        const float v_want = clampf((s->setpoint[i] - s->motor[i]) / p[SIM3D_SERVO_TAU_S], -vmax, vmax);
        s->motor_rate[i] += clampf(v_want - s->motor_rate[i], -amax * PHYS_DT, amax * PHYS_DT);
        s->motor[i] += s->motor_rate[i] * PHYS_DT;
    } else {
        s->motor_rate[i] = 0.0f;
    }
    /* backlash: the horn only moves once the gear train has taken up the slop */
    const float half = 0.5f * p[SIM3D_SERVO_BACKLASH_DEG] * DEG;
    if (s->motor[i] - s->horn[i] > half) s->horn[i] = s->motor[i] - half;
    else if (s->horn[i] - s->motor[i] > half) s->horn[i] = s->motor[i] + half;
    /* linkage and mechanical stop (the stop also blocks the horn and motor) */
    const float stop = p[SIM3D_GIMBAL_STOP_DEG] * DEG, ratio = p[SIM3D_LINKAGE_RATIO];
    const float raw = asinf(clampf(ratio * sinf(s->horn[i]), -1.0f, 1.0f));
    if (fabsf(raw) > stop) {
        const float horn_limit = asinf(clampf(sinf(stop) / ratio, -1.0f, 1.0f));
        s->horn[i] = copysignf(horn_limit, s->horn[i]);
        if (fabsf(s->motor[i]) > horn_limit + half) { s->motor[i] = copysignf(horn_limit + half, s->motor[i]); s->motor_rate[i] = 0.0f; }
    }
    s->gimbal[i] = clampf(asinf(clampf(ratio * sinf(s->horn[i]), -1.0f, 1.0f)), -stop, stop);
}

/* ---------------------------------------------------------------------------------------------
 * one millisecond
 * --------------------------------------------------------------------------------------------- */
static void sim_step(Sim *s, float *rows, unsigned max_rows, unsigned *row_count)
{
    const float *p = s->p;
    const float t = s->step * PHYS_DT;
    const float t_burn = t - p[SIM3D_IGNITION_S];
    const float L = p[SIM3D_LENGTH_M], r = 0.5f * p[SIM3D_DIAMETER_M];
    const float area = 3.14159265f * 0.25f * p[SIM3D_DIAMETER_M] * p[SIM3D_DIAMETER_M];
    const float chute_area = 3.14159265f * 0.25f * p[SIM3D_CHUTE_DIAMETER_M] * p[SIM3D_CHUTE_DIAMETER_M];

    float thrust, impulse_fraction;
    motor_at(p, t_burn, &thrust, &impulse_fraction);
    const float frac = clampf(impulse_fraction, 0.0f, 1.0f);
    /* propellant mass flow, for jet damping */
    float thrust_next, frac_next;
    motor_at(p, t_burn + PHYS_DT, &thrust_next, &frac_next);
    const float mdot = p[SIM3D_PROP_MASS_KG] * fmaxf(0.0f, frac_next - frac) / PHYS_DT;
    const float mass = s->liftoff_mass - p[SIM3D_PROP_MASS_KG] * frac;
    /* CG moves forward as propellant burns: arm and CP distance follow it */
    const float inertia_wet = p[SIM3D_INERTIA] > 0.0f ? p[SIM3D_INERTIA] : s->liftoff_mass * (L * L / 12.0f + r * r / 4.0f);
    const float dry_mass = s->liftoff_mass - p[SIM3D_PROP_MASS_KG];
    const float inertia_dry = p[SIM3D_INERTIA_BURNOUT] > 0.0f ? p[SIM3D_INERTIA_BURNOUT] : dry_mass * (L * L / 12.0f + r * r / 4.0f);
    const float inertia = inertia_wet + (inertia_dry - inertia_wet) * frac;
    const float cg_shift = p[SIM3D_CG_SHIFT_M] * frac;
    const float inertia_z = fmaxf(0.5f * mass * r * r, 1e-6f);
    s->thrust = thrust;
    const float rho = air_density(p, fmaxf(0.0f, s->pos[2]));
    s->rho = rho;

    /* ---- wind ---- */
    const float gust = fmaxf(0.0f, sinf(0.9f * t + s->gust_phase)) * fmaxf(0.0f, sinf(0.37f * t + 1.3f * s->gust_phase));
    const float extra = t < s->gust_until ? s->gust_extra : 0.0f;
    const float wind_speed = p[SIM3D_WIND_M_S] + (p[SIM3D_GUST_M_S] - p[SIM3D_WIND_M_S]) * gust + extra;
    const float h_factor = s->pos[2] > 1.0f ? powf(s->pos[2], 0.14f) / powf(10.0f, 0.14f) : 0.72f;   /* wind shear */
    {
        /* Dryden-like turbulence: first-order Gauss-Markov per axis, length scale ~30 m near the ground */
        const float scale = 30.0f, speed = fmaxf(1.0f, norm3(s->vel) + wind_speed), sigma = p[SIM3D_TURBULENCE_M_S];
        const float a = speed / scale;
        for (unsigned i = 0; i < 3; ++i)
            s->turb[i] += -a * s->turb[i] * PHYS_DT + sigma * sqrtf(2.0f * a * PHYS_DT) * gauss(s) * (i == 2 ? 0.5f : 1.0f);
    }
    const float wind[3] = {wind_speed * h_factor * cosf(s->wind_dir) + s->turb[0],
                           wind_speed * h_factor * sinf(s->wind_dir) + s->turb[1], s->turb[2]};
    s->wind_h = sqrtf(wind[0] * wind[0] + wind[1] * wind[1]);

    /* ---- forces in body frame: thrust along the gimballed motor axis ---- */
    const float da = s->gimbal[0] + p[SIM3D_MISALIGN_DEG] * DEG, db = s->gimbal[1];
    const float thrust_b[3] = {-thrust * sinf(db), thrust * sinf(da) * cosf(db), thrust * cosf(da) * cosf(db)};
    const float v_air_w[3] = {s->vel[0] - wind[0], s->vel[1] - wind[1], s->vel[2] - wind[2]};
    float v_air_b[3];
    unrotate(s->q, v_air_w, v_air_b);
    const float speed_air = norm3(v_air_w);
    float aero_b[3] = {0, 0, 0}, normal_b[3] = {0, 0, 0}, lift_b[3] = {0, 0, 0};
    const float v_lat = sqrtf(v_air_b[0] * v_air_b[0] + v_air_b[1] * v_air_b[1]);
    s->alpha = speed_air > 0.5f ? atan2f(v_lat, fabsf(v_air_b[2])) : 0.0f;
    if (s->off_pad && !s->chute_open) {
        aero_b[2] = -0.5f * rho * p[SIM3D_BODY_CD] * area * speed_air * v_air_b[2];
        /* Barrowman nose normal force, linear in alpha, at the CP */
        normal_b[0] = -0.5f * rho * p[SIM3D_CN_ALPHA] * area * speed_air * v_air_b[0];
        normal_b[1] = -0.5f * rho * p[SIM3D_CN_ALPHA] * area * speed_air * v_air_b[1];
        /* Body lift (Galejs): 1.1 * planform/ref * sin^2(alpha), at the planform centroid */
        if (v_lat > 1e-4f) {
            const float s2 = sinf(s->alpha) * sinf(s->alpha);
            const float lift = 0.5f * rho * speed_air * speed_air * 1.1f * p[SIM3D_PLANFORM_AREA_M2] * s2;
            lift_b[0] = -lift * v_air_b[0] / v_lat;
            lift_b[1] = -lift * v_air_b[1] / v_lat;
        }
    }
    const float force_b[3] = {thrust_b[0] + aero_b[0] + normal_b[0] + lift_b[0],
                              thrust_b[1] + aero_b[1] + normal_b[1] + lift_b[1],
                              thrust_b[2] + aero_b[2] + normal_b[2]};
    float force_w[3];
    rotate(s->q, force_b, force_w);
    if (s->chute_open && s->off_pad) {
        const float inflate = clampf((t - s->chute_open_time) / 0.6f, 0.0f, 1.0f);
        const float k = 0.5f * rho * (p[SIM3D_CHUTE_CD] * chute_area * inflate + p[SIM3D_BODY_CD] * area) * speed_air;
        for (unsigned i = 0; i < 3; ++i) force_w[i] += -k * v_air_w[i];
    }

    /* ---- moments about the CG: thrust at the gimbal, normal force at the CP, damping ---- */
    const float l = p[SIM3D_GIMBAL_ARM_M] + cg_shift, d = p[SIM3D_CP_AHEAD_M] - cg_shift;
    const float dp = p[SIM3D_PLANFORM_AHEAD_M] - cg_shift;
    s->tq_tvc[0] = l * thrust_b[1];
    s->tq_tvc[1] = -l * thrust_b[0];
    s->tq_aero[0] = -d * normal_b[1] - dp * lift_b[1];
    s->tq_aero[1] = d * normal_b[0] + dp * lift_b[0];
    /* gimballed motor swung by the servos: the airframe feels the reaction (Newton III) */
    s->tq_tvc[0] += p[SIM3D_MOTOR_GIMBAL_INERTIA] * s->gimbal_acc[0];
    s->tq_tvc[1] += p[SIM3D_MOTOR_GIMBAL_INERTIA] * s->gimbal_acc[1];
    /* pitch damping from physics: the nose sees cross-flow w*d when the body turns (Barrowman
     * normal force at the CP), and the exhaust carries away angular momentum (jet damping) */
    const float aero_damping = s->off_pad && !s->chute_open ? 0.5f * rho * speed_air * area * p[SIM3D_CN_ALPHA] * d * d : 0.0f;
    const float nozzle = p[SIM3D_NOZZLE_ARM_M] + cg_shift;
    const float jet_damping = mdot * nozzle * nozzle;
    const float damping = aero_damping + jet_damping;
    const float torque[3] = {s->tq_tvc[0] + s->tq_aero[0] - damping * s->w[0],
                             s->tq_tvc[1] + s->tq_aero[1] - damping * s->w[1], 0};

    /* ---- integrate ---- */
    const float weight = mass * G0;
    if (!s->off_pad) {
        /* on the stand: held until thrust along the rail beats gravity */
        const float along = force_w[0] * s->rail_axis[0] + force_w[1] * s->rail_axis[1] + force_w[2] * s->rail_axis[2];
        if (along > weight * s->rail_axis[2] + 1e-3f) s->off_pad = true;
    }
    const bool on_rail = s->off_pad &&
        (s->pos[0] * s->rail_axis[0] + s->pos[1] * s->rail_axis[1] + s->pos[2] * s->rail_axis[2]) < p[SIM3D_TOWER_M];
    float accel_w[3] = {force_w[0] / mass, force_w[1] / mass, force_w[2] / mass - G0};
    if (!s->off_pad) {
        accel_w[0] = accel_w[1] = accel_w[2] = 0.0f;
    } else if (on_rail) {
        const float a = accel_w[0] * s->rail_axis[0] + accel_w[1] * s->rail_axis[1] + accel_w[2] * s->rail_axis[2];
        for (unsigned i = 0; i < 3; ++i) accel_w[i] = a * s->rail_axis[i];
    }
    if (!s->landed) {
        for (unsigned i = 0; i < 3; ++i) { s->vel[i] += accel_w[i] * PHYS_DT; s->pos[i] += s->vel[i] * PHYS_DT; }
        if (s->off_pad && !on_rail) {
            const float *w = s->w;
            const float Iw[3] = {inertia * w[0], inertia * w[1], inertia_z * w[2]};
            const float gyro_term[3] = {w[1]*Iw[2] - w[2]*Iw[1], w[2]*Iw[0] - w[0]*Iw[2], w[0]*Iw[1] - w[1]*Iw[0]};
            s->w_dot[0] = (torque[0] - gyro_term[0]) / inertia;
            s->w_dot[1] = (torque[1] - gyro_term[1]) / inertia;
            s->w_dot[2] = (torque[2] - gyro_term[2]) / inertia_z;
            for (unsigned i = 0; i < 3; ++i) s->w[i] += s->w_dot[i] * PHYS_DT;
            if (s->chute_open) for (unsigned i = 0; i < 3; ++i) s->w[i] *= 0.995f;
            /* exact rotation for a constant rate over the step (exponential map) */
            const float rate = norm3(s->w), angle = rate * PHYS_DT;
            if (angle > 1e-9f) {
                const float k = sinf(0.5f * angle) / rate;
                s->q = qmul(s->q, (Quat){cosf(0.5f * angle), s->w[0] * k, s->w[1] * k, s->w[2] * k});
                qnorm(&s->q);
            }
        } else {
            s->w_dot[0] = s->w_dot[1] = s->w_dot[2] = 0.0f;
        }
        if (s->off_pad && s->pos[2] <= 0.0f && t_burn > 0.3f && s->vel[2] < 0.0f) {
            s->landed = true;
            s->land_time = t;
            s->summary[SIM3D_SUM_LANDING_SPEED] = norm3(s->vel);
            s->summary[SIM3D_SUM_DRIFT_M] = sqrtf(s->pos[0] * s->pos[0] + s->pos[1] * s->pos[1]);
            s->summary[SIM3D_SUM_UNDER_CHUTE] = s->chute_open ? 1.0f : 0.0f;
            s->pos[2] = 0.0f;
            s->vel[0] = s->vel[1] = s->vel[2] = 0.0f;
            s->w[0] = s->w[1] = s->w[2] = 0.0f;
        }
    }

    /* specific force the accelerometer would feel (body frame) */
    float specific_w[3] = {accel_w[0], accel_w[1], accel_w[2] + G0};
    if (!s->off_pad || s->landed) { specific_w[0] = 0; specific_w[1] = 0; specific_w[2] = G0; }
    unrotate(s->q, specific_w, s->last_specific);
    {
        /* the IMU is not at the CG: it also feels w_dot x r + w x (w x r) */
        const float r_imu[3] = {0, 0, p[SIM3D_IMU_AHEAD_M] - cg_shift};
        const float *w = s->w, *wd = s->w_dot;
        const float wxr[3] = {w[1]*r_imu[2] - w[2]*r_imu[1], w[2]*r_imu[0] - w[0]*r_imu[2], w[0]*r_imu[1] - w[1]*r_imu[0]};
        const float tang[3] = {wd[1]*r_imu[2] - wd[2]*r_imu[1], wd[2]*r_imu[0] - wd[0]*r_imu[2], wd[0]*r_imu[1] - wd[1]*r_imu[0]};
        const float cent[3] = {w[1]*wxr[2] - w[2]*wxr[1], w[2]*wxr[0] - w[0]*wxr[2], w[0]*wxr[1] - w[1]*wxr[0]};
        if (s->off_pad && !s->landed) for (unsigned i = 0; i < 3; ++i) s->last_specific[i] += tang[i] + cent[i];
    }
    /* BMI088 internal filters run at the sensor, before the 200 Hz reads */
    {
        const float ag = 1.0f - expf(-6.2831853f * p[SIM3D_GYRO_LPF_HZ] * PHYS_DT);
        const float aa = 1.0f - expf(-6.2831853f * p[SIM3D_ACCEL_LPF_HZ] * PHYS_DT);
        for (unsigned i = 0; i < 3; ++i) {
            s->gyro_f[i] += ag * (s->w[i] - s->gyro_f[i]);
            s->accel_f[i] += aa * (s->last_specific[i] - s->accel_f[i]);
        }
    }

    /* ---- servos (1 kHz plant) ---- */
    servo_step(s, 0);
    servo_step(s, 1);
    for (unsigned i = 0; i < 2; ++i) {
        /* gimbal angular acceleration, lightly filtered (backlash makes the raw difference spiky) */
        const float rate = (s->gimbal[i] - s->gimbal_prev[i]) / PHYS_DT;
        const float acc = (rate - s->gimbal_rate[i]) / PHYS_DT;
        s->gimbal_acc[i] += 0.3f * (acc - s->gimbal_acc[i]);
        s->gimbal_rate[i] = rate;
        s->gimbal_prev[i] = s->gimbal[i];
    }

    /* ---- flight software at 200 Hz ---- */
    if (s->step % CONTROL_DIV == 0U) {
        const uint32_t now_ms = (uint32_t)lroundf(t * 1000.0f);
        const float noise = p[SIM3D_NOISE];
        float accel[3], gyro_raw[3], gyro[3];
        /* BMI088-like noise (~4 mg, ~0.15 deg/s RMS), plus motor vibration while burning. */
        const float vib = thrust > 0.0f ? 1.0f : 0.0f;
        /* quantised like the real registers: +-24 g and +-2000 deg/s in 16 bits */
        const float accel_lsb = 24.0f * G0 / 32768.0f, gyro_lsb = 2000.0f * DEG / 32768.0f;
        for (unsigned i = 0; i < 3; ++i) {
            const float a = clampf(s->accel_f[i] + noise * (0.004f + 0.3f * vib) * G0 * gauss(s), -24.0f * G0, 24.0f * G0);
            const float g = clampf(s->gyro_f[i] + s->true_bias[i] + noise * (0.0026f + 0.03f * vib) * gauss(s), -34.9f, 34.9f);
            accel[i] = roundf(a / accel_lsb) * accel_lsb;
            gyro_raw[i] = roundf(g / gyro_lsb) * gyro_lsb;
            gyro[i] = gyro_raw[i] - s->fw_bias[i];
        }
        if ((s->step / CONTROL_DIV) % 4U == 0U) s->baro_alt = s->pos[2] + noise * 0.25f * gauss(s);
        float new_bias[3];
        if (EasyTVC_PadArmStep(&s->pad, &s->pad_config, s->app.flight.state, true, true, accel, gyro, gyro_raw,
                               now_ms, new_bias)) {
            for (unsigned i = 0; i < 3; ++i) {
                s->fw_bias[i] = new_bias[i];
                gyro[i] = gyro_raw[i] - s->fw_bias[i];
                s->app.attitude.integral_error[i] = 0.0f;
            }
            if (s->summary[SIM3D_SUM_ARMED_T] < 0) s->summary[SIM3D_SUM_ARMED_T] = t;
        }
        EasyTVCAppInput in = {.baro_altitude_m = s->baro_alt, .sensors_healthy = true,
            .physical_arm_qualified = s->pad.armed, .software_arm_request = s->pad.armed,
            .accel_is_gravity_reference = true, .now_ms = now_ms, .sensor_age_ms = 0,
            .dt_s = CONTROL_DIV * PHYS_DT};
        memcpy(in.accel_m_s2, accel, sizeof(accel));
        memcpy(in.gyro_rad_s, gyro, sizeof(gyro));
        const EasyTVCAppOutput out = EasyTVC_AppStep(&s->app, &in);
        if (out.flight.state == EASYTVC_FLIGHT_IDLE) s->pad.armed = false;
        if (!s->launched && out.flight.state == EASYTVC_FLIGHT_BOOST) {
            s->launched = true;
            s->launch_ms = now_ms;
            s->summary[SIM3D_SUM_LAUNCH_T] = t - p[SIM3D_IGNITION_S];
        }
        if (out.control.fault) s->summary[SIM3D_SUM_TVC_FAULT] = 1.0f;
        if (out.control.enabled) {
            ++s->tvc_ticks;
            if (fabsf(out.control.command[0]) > 0.98f || fabsf(out.control.command[1]) > 0.98f) ++s->saturated_ticks;
        }
        if (p[SIM3D_CHUTE_ENABLED] > 0.5f && !s->chute_open &&
            EasyTVC_ChuteShouldOpen(s->launched, now_ms - s->launch_ms, out.flight.state,
                                    out.flight.drogue_deploy_requested, (uint32_t)p[SIM3D_CHUTE_BACKUP_MS])) {
            s->chute_open = true;
            s->chute_open_time = t;
            s->summary[SIM3D_SUM_CHUTE_T] = t - p[SIM3D_IGNITION_S];
            s->summary[SIM3D_SUM_CHUTE_SPEED] = norm3(s->vel);
        }
        /* the timer hardware outputs a new pulse only once per PWM frame; the servo latches it */
        const unsigned frame = (unsigned)lroundf(1.0f / (p[SIM3D_SERVO_RATE_HZ] * PHYS_DT));
        if (s->step % (frame ? frame : 1U) < CONTROL_DIV) {
            for (unsigned i = 0; i < 2; ++i)
                s->pulse_latched[i] = s->manual ? EasyTVC_ServoPulse(&s->settings.servo[i], s->manual_cmd[i])
                                                : out.control.pulse_us[i];
        }

        /* ---- record ---- */
        const Quat q = s->q;
        const float tilt_now = acosf(clampf(1.0f - 2.0f * (q.x * q.x + q.y * q.y), -1.0f, 1.0f)) / DEG;
        float est[2], truth[2];
        EasyTVC_ControllerTilt(&s->app.attitude.orientation, est);
        const EasyTVCQuaternion tq = {q.w, q.x, q.y, q.z};
        EasyTVC_ControllerTilt(&tq, truth);
        if (out.flight.state == EASYTVC_FLIGHT_BOOST) {
            s->summary[SIM3D_SUM_MAX_BOOST_TILT] = fmaxf(s->summary[SIM3D_SUM_MAX_BOOST_TILT], tilt_now);
            s->summary[SIM3D_SUM_EST_ERROR_DEG] = fmaxf(s->summary[SIM3D_SUM_EST_ERROR_DEG],
                fmaxf(fabsf(est[0] - truth[0]), fabsf(est[1] - truth[1])) / DEG);
        }
        if (s->pos[2] > s->summary[SIM3D_SUM_APOGEE_M]) {
            s->summary[SIM3D_SUM_APOGEE_M] = s->pos[2];
            s->summary[SIM3D_SUM_APOGEE_T] = t - p[SIM3D_IGNITION_S];
        }
        s->summary[SIM3D_SUM_MAX_SPEED] = fmaxf(s->summary[SIM3D_SUM_MAX_SPEED], norm3(s->vel));
        s->summary[SIM3D_SUM_FINAL_STATE] = (float)out.flight.state;
        if (s->off_pad) s->summary[SIM3D_SUM_LIFTOFF] = 1.0f;

        if (s->step % ROW_DIV == 0U && rows && *row_count < max_rows) {
            float *row = rows + (*row_count) * SIM3D_COLUMN_COUNT;
            row[SIM3D_COL_T] = t; row[SIM3D_COL_X] = s->pos[0]; row[SIM3D_COL_Y] = s->pos[1]; row[SIM3D_COL_Z] = s->pos[2];
            row[SIM3D_COL_QW] = q.w; row[SIM3D_COL_QX] = q.x; row[SIM3D_COL_QY] = q.y; row[SIM3D_COL_QZ] = q.z;
            row[SIM3D_COL_SPEED] = norm3(s->vel); row[SIM3D_COL_VZ] = s->vel[2];
            row[SIM3D_COL_TILT_X] = truth[0] / DEG; row[SIM3D_COL_TILT_Y] = truth[1] / DEG;
            row[SIM3D_COL_GIMBAL_A] = s->gimbal[0] / DEG; row[SIM3D_COL_GIMBAL_B] = s->gimbal[1] / DEG;
            row[SIM3D_COL_STATE] = (float)out.flight.state; row[SIM3D_COL_TVC] = out.control.enabled ? 1.0f : 0.0f;
            row[SIM3D_COL_CHUTE] = s->chute_open ? 1.0f : 0.0f; row[SIM3D_COL_THRUST] = thrust;
            row[SIM3D_COL_EST_TILT_X] = est[0] / DEG; row[SIM3D_COL_EST_TILT_Y] = est[1] / DEG;
            row[SIM3D_COL_ARMED] = s->pad.armed ? 1.0f : 0.0f;
            row[SIM3D_COL_PULSE_A] = s->pulse_latched[0]; row[SIM3D_COL_PULSE_B] = s->pulse_latched[1];
            row[SIM3D_COL_HORN_A] = s->horn[0] / DEG; row[SIM3D_COL_HORN_B] = s->horn[1] / DEG;
            row[SIM3D_COL_ALPHA] = s->alpha / DEG;
            row[SIM3D_COL_TQ_TVC_X] = s->tq_tvc[0]; row[SIM3D_COL_TQ_TVC_Y] = s->tq_tvc[1];
            row[SIM3D_COL_TQ_AERO_X] = s->tq_aero[0]; row[SIM3D_COL_TQ_AERO_Y] = s->tq_aero[1];
            row[SIM3D_COL_SERVO_FAULT] = (float)((s->jammed[0] ? 1 : 0) | (s->jammed[1] ? 2 : 0) | (s->manual ? 4 : 0));
            row[SIM3D_COL_WIND] = s->wind_h;
            ++(*row_count);
        }
    }

    ++s->step;
    s->done = (s->landed && t - s->land_time > 1.0f) || t > MAX_TIME_S ||
              (!s->off_pad && t_burn > burn_time(p) + 2.0f);
}

static void finish_summary(Sim *s, float *summary)
{
    s->summary[SIM3D_SUM_SATURATION_PCT] = s->tvc_ticks ? 100.0f * s->saturated_ticks / s->tvc_ticks : 0.0f;
    s->summary[SIM3D_SUM_FLIGHT_TIME] = s->land_time >= 0 ? s->land_time - s->p[SIM3D_IGNITION_S] : -1.0f;
    s->summary[SIM3D_SUM_DONE] = s->done ? 1.0f : 0.0f;
    s->summary[SIM3D_SUM_TIME] = s->step * PHYS_DT;
    memcpy(summary, s->summary, sizeof(s->summary));
}

/* ---------------------------------------------------------------------------------------------
 * public API
 * --------------------------------------------------------------------------------------------- */
static Sim batch_sim, live_sim;   /* large: static, not on the stack */

int EasyTVC_Sim3D(const float *p, unsigned param_count, float *rows, unsigned max_rows,
                  float *summary, unsigned summary_count)
{
    if (!rows || max_rows < 2 || !summary || summary_count != SIM3D_SUMMARY_COUNT) return -1;
    const int status = sim_begin(&batch_sim, p, param_count);
    if (status < 0) return status;
    unsigned row_count = 0;
    while (!batch_sim.done && row_count < max_rows) sim_step(&batch_sim, rows, max_rows, &row_count);
    finish_summary(&batch_sim, summary);
    return (int)row_count;
}

int EasyTVC_Sim3DBegin(const float *p, unsigned param_count)
{
    return sim_begin(&live_sim, p, param_count);
}

int EasyTVC_Sim3DAdvance(float seconds, float *rows, unsigned max_rows, float *summary, unsigned summary_count)
{
    if (!live_sim.ready || !summary || summary_count != SIM3D_SUMMARY_COUNT || !isfinite(seconds) || seconds < 0.0f) return -1;
    const unsigned steps = (unsigned)lroundf(clampf(seconds, 0.0f, MAX_TIME_S) / PHYS_DT);
    unsigned row_count = 0;
    for (unsigned i = 0; i < steps && !live_sim.done; ++i) sim_step(&live_sim, rows, max_rows, &row_count);
    finish_summary(&live_sim, summary);
    return (int)row_count;
}

int EasyTVC_Sim3DInject(int kind, float a, float b)
{
    Sim *s = &live_sim;
    if (!s->ready || s->done || !isfinite(a) || !isfinite(b)) return -1;
    switch (kind) {
    case SIM3D_INJECT_KICK:
        if (!s->off_pad) return -2;   /* a held rocket cannot be kicked */
        s->w[0] += clampf(a, -500.0f, 500.0f) * DEG;
        s->w[1] += clampf(b, -500.0f, 500.0f) * DEG;
        return 0;
    case SIM3D_INJECT_GUST:
        s->gust_extra = clampf(a, -30.0f, 30.0f);
        s->gust_until = s->step * PHYS_DT + clampf(b, 0.0f, 10.0f);
        return 0;
    case SIM3D_INJECT_MANUAL:
        s->manual = true;
        s->manual_cmd[0] = clampf(a, -1.0f, 1.0f);
        s->manual_cmd[1] = clampf(b, -1.0f, 1.0f);
        return 0;
    case SIM3D_INJECT_AUTO:
        s->manual = false;
        return 0;
    case SIM3D_INJECT_JAM_SERVO:
    case SIM3D_INJECT_FREE_SERVO:
        if (a != 0.0f && a != 1.0f) return -1;
        s->jammed[(int)a] = kind == SIM3D_INJECT_JAM_SERVO;
        return 0;
    default:
        return -1;
    }
}
