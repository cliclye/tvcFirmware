/* The 3D sim runs the real flight software; these checks pin down that a sane E12 rocket flies,
 * that the TVC sign convention is right (a reversed servo must lose control), and that failure
 * modes show up. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "flight_sim3d.h"

static float rows[6000 * SIM3D_COLUMN_COUNT];

static void defaults(float *p)
{
    memset(p, 0, sizeof(float) * SIM3D_PARAM_COUNT);
    p[SIM3D_KP] = 8.0f; p[SIM3D_KI] = 0.5f; p[SIM3D_KD] = 0.8f; p[SIM3D_INTEGRATOR_LIMIT] = 0.15f; p[SIM3D_RATE_LIMIT] = 20.0f;
    p[SIM3D_DERIVATIVE_CUTOFF] = 20.0f; p[SIM3D_MAX_TILT_DEG] = 25.0f;
    p[SIM3D_A_MIN] = 1250; p[SIM3D_A_CENTER] = 1500; p[SIM3D_A_MAX] = 1750; p[SIM3D_A_DIRECTION] = 1;
    p[SIM3D_B_MIN] = 1250; p[SIM3D_B_CENTER] = 1500; p[SIM3D_B_MAX] = 1750; p[SIM3D_B_DIRECTION] = -1;
    p[SIM3D_SERVO_RATE_HZ] = 50; p[SIM3D_LAUNCH_G] = 2.5f; p[SIM3D_LAUNCH_CONFIRM_MS] = 40;
    p[SIM3D_TVC_DELAY_MS] = 80; p[SIM3D_CHUTE_ENABLED] = 1; p[SIM3D_CHUTE_BACKUP_MS] = 7000;
    p[SIM3D_ARM_HOLD_MS] = 3000; p[SIM3D_ARM_TILT_DEG] = 10; p[SIM3D_DISARM_TILT_DEG] = 20;
    p[SIM3D_DRY_MASS_KG] = 0.322f; p[SIM3D_MOTOR_MASS_KG] = 0.058f; p[SIM3D_PROP_MASS_KG] = 0.036f;
    p[SIM3D_AVG_THRUST_N] = 12.0f; p[SIM3D_BURN_S] = 2.5f; p[SIM3D_THRUST_PROFILE] = 1;
    p[SIM3D_DIAMETER_M] = 0.066f; p[SIM3D_BODY_CD] = 0.6f; p[SIM3D_CHUTE_DIAMETER_M] = 0.45f; p[SIM3D_CHUTE_CD] = 0.8f;
    /* the default 610 mm / 380 g rocket (80 g nose ballast) from Studio's rocket model */
    p[SIM3D_LENGTH_M] = 0.61f; p[SIM3D_GIMBAL_ARM_M] = 0.2403f; p[SIM3D_CP_AHEAD_M] = 0.2548f; p[SIM3D_CN_ALPHA] = 2.0f;
    p[SIM3D_INERTIA] = 0.01606f; p[SIM3D_INERTIA_BURNOUT] = 0.01377f; p[SIM3D_CG_SHIFT_M] = 0.02489f;
    p[SIM3D_PLANFORM_AREA_M2] = 0.0363f; p[SIM3D_PLANFORM_AHEAD_M] = 0.0015f;
    p[SIM3D_LINKAGE_RATIO] = 0.35f; p[SIM3D_SERVO_SPEED_DPS] = 600; p[SIM3D_SERVO_DEG_PER_US] = 0.09f;
    p[SIM3D_GIMBAL_STOP_DEG] = 8; p[SIM3D_WIND_M_S] = 2; p[SIM3D_GUST_M_S] = 4;
    p[SIM3D_SERVO_DEADBAND_US] = 4; p[SIM3D_SERVO_BACKLASH_DEG] = 0.8f; p[SIM3D_SERVO_TAU_S] = 0.01f;
    p[SIM3D_THRUST_SCALE] = 1; p[SIM3D_SITE_ALT_M] = 0; p[SIM3D_TEMP_C] = 15; p[SIM3D_NOZZLE_ARM_M] = 0.285f;
    p[SIM3D_MOTOR_GIMBAL_INERTIA] = 4.4e-5f; p[SIM3D_SERVO_STALL_NM] = 0.2f; p[SIM3D_IMU_AHEAD_M] = -0.005f;
    p[SIM3D_TURBULENCE_M_S] = 0.5f; p[SIM3D_GYRO_LPF_HZ] = 47; p[SIM3D_ACCEL_LPF_HZ] = 145;
    p[SIM3D_WIND_DIR_DEG] = 30; p[SIM3D_PAD_TILT_DEG] = 1; p[SIM3D_MISALIGN_DEG] = 0.5f; p[SIM3D_GYRO_BIAS_DPS] = 0.5f;
    p[SIM3D_NOISE] = 1; p[SIM3D_SEED] = 1; p[SIM3D_TOWER_M] = 0.3f; p[SIM3D_IGNITION_S] = 5;
}

static int run(const float *p, float *sum)
{
    return EasyTVC_Sim3D(p, SIM3D_PARAM_COUNT, rows, 6000, sum, SIM3D_SUMMARY_COUNT);
}

int main(void)
{
    float p[SIM3D_PARAM_COUNT], sum[SIM3D_SUMMARY_COUNT];

    defaults(p);
    int n = run(p, sum);
    assert(n > 100);
    printf("  nominal: armed %.1fs, apogee %.0f m at %.1f s, boost tilt %.1f deg, chute at %.1f s (%.0f m/s), "
           "lands %.1f m/s, drift %.0f m, est err %.2f deg, sat %.0f%%\n",
           sum[SIM3D_SUM_ARMED_T], sum[SIM3D_SUM_APOGEE_M], sum[SIM3D_SUM_APOGEE_T], sum[SIM3D_SUM_MAX_BOOST_TILT],
           sum[SIM3D_SUM_CHUTE_T], sum[SIM3D_SUM_CHUTE_SPEED], sum[SIM3D_SUM_LANDING_SPEED], sum[SIM3D_SUM_DRIFT_M],
           sum[SIM3D_SUM_EST_ERROR_DEG], sum[SIM3D_SUM_SATURATION_PCT]);
    assert(sum[SIM3D_SUM_ARMED_T] > 2.9f && sum[SIM3D_SUM_ARMED_T] < 3.3f);
    assert(sum[SIM3D_SUM_LAUNCH_T] >= 0.0f && sum[SIM3D_SUM_LAUNCH_T] < 0.2f);
    assert(sum[SIM3D_SUM_TVC_FAULT] == 0.0f);
    assert(sum[SIM3D_SUM_MAX_BOOST_TILT] < 10.0f);
    assert(sum[SIM3D_SUM_APOGEE_M] > 60.0f);
    assert(sum[SIM3D_SUM_CHUTE_T] > 0.0f && sum[SIM3D_SUM_UNDER_CHUTE] == 1.0f);
    assert(sum[SIM3D_SUM_LANDING_SPEED] < 8.0f);
    assert(sum[SIM3D_SUM_EST_ERROR_DEG] < 3.0f);

    /* The original default gains (1.2 / 0 / 0.2, slew 4/s) lose this rocket. */
    defaults(p);
    p[SIM3D_KP] = 1.2f; p[SIM3D_KI] = 0.0f; p[SIM3D_KD] = 0.2f; p[SIM3D_RATE_LIMIT] = 4.0f;
    run(p, sum);
    printf("  old gains: boost tilt %.0f deg, fault %g\n", sum[SIM3D_SUM_MAX_BOOST_TILT], sum[SIM3D_SUM_TVC_FAULT]);
    assert(sum[SIM3D_SUM_TVC_FAULT] == 1.0f);

    /* Without the nose ballast (CG 76 mm further aft) the rocket is too unstable to hold. */
    defaults(p);
    p[SIM3D_DRY_MASS_KG] = 0.242f; p[SIM3D_GIMBAL_ARM_M] = 0.1644f; p[SIM3D_CP_AHEAD_M] = 0.3307f;
    p[SIM3D_INERTIA] = 0.00782f; p[SIM3D_INERTIA_BURNOUT] = 0.00671f; p[SIM3D_CG_SHIFT_M] = 0.02208f;
    p[SIM3D_PLANFORM_AHEAD_M] = 0.0774f;
    run(p, sum);
    printf("  no nose ballast: boost tilt %.0f deg, fault %g\n", sum[SIM3D_SUM_MAX_BOOST_TILT], sum[SIM3D_SUM_TVC_FAULT]);
    assert(sum[SIM3D_SUM_TVC_FAULT] == 1.0f);

    /* Same rocket with no TVC authority: an unstable finless rocket must tumble. */
    defaults(p);
    p[SIM3D_LINKAGE_RATIO] = 0.01f;
    run(p, sum);
    printf("  no authority: boost tilt %.0f deg, fault %g\n", sum[SIM3D_SUM_MAX_BOOST_TILT], sum[SIM3D_SUM_TVC_FAULT]);
    assert(sum[SIM3D_SUM_MAX_BOOST_TILT] > 20.0f && sum[SIM3D_SUM_TVC_FAULT] == 1.0f);

    /* A servo installed the wrong way round makes it worse, not better: the bench tilt test matters. */
    defaults(p);
    p[SIM3D_REVERSE_SERVO_A] = 1;
    run(p, sum);
    printf("  reversed servo A: boost tilt %.0f deg, fault %g\n", sum[SIM3D_SUM_MAX_BOOST_TILT], sum[SIM3D_SUM_TVC_FAULT]);
    assert(sum[SIM3D_SUM_TVC_FAULT] == 1.0f && sum[SIM3D_SUM_MAX_BOOST_TILT] > 20.0f);

    /* Pad tilted past the arming limit: never arms, never launches TVC. */
    defaults(p);
    p[SIM3D_PAD_TILT_DEG] = 15;
    run(p, sum);
    assert(sum[SIM3D_SUM_ARMED_T] < 0.0f && sum[SIM3D_SUM_LAUNCH_T] < 0.0f && sum[SIM3D_SUM_CHUTE_T] < 0.0f);

    /* Too heavy for the motor: does not leave the stand. */
    defaults(p);
    p[SIM3D_DRY_MASS_KG] = 4.0f;
    run(p, sum);
    assert(sum[SIM3D_SUM_LIFTOFF] == 0.0f);

    /* Invalid input is rejected. */
    defaults(p);
    p[SIM3D_BURN_S] = NAN;
    assert(run(p, sum) < 0);
    defaults(p);
    p[SIM3D_A_CENTER] = 1240;   /* firmware settings check */
    assert(run(p, sum) < 0);
    /* ---- the rocket moves because of the TVC, not the other way round ---- */
    static float live_rows[6000 * SIM3D_COLUMN_COUNT];
    float live_sum[SIM3D_SUMMARY_COUNT];

    /* 1. stepping the live sim in uneven slices gives bit-identical results to the batch run */
    defaults(p);
    const int batch_rows = run(p, sum);
    assert(EasyTVC_Sim3DBegin(p, SIM3D_PARAM_COUNT) == 0);
    int got = 0;
    for (unsigned k = 0; got < 6000; ++k) {
        const int n = EasyTVC_Sim3DAdvance(k % 2 ? 0.017f : 0.033f, live_rows + got * SIM3D_COLUMN_COUNT, 6000 - got,
                                           live_sum, SIM3D_SUMMARY_COUNT);
        assert(n >= 0); got += n;
        if (live_sum[SIM3D_SUM_DONE] > 0.5f) break;
    }
    assert(got == batch_rows);
    assert(memcmp(rows, live_rows, sizeof(float) * SIM3D_COLUMN_COUNT * (size_t)got) == 0);

    /* 2. jam servo A half a second into boost: that axis loses control */
    defaults(p);
    p[SIM3D_MISALIGN_DEG] = 1.0f;
    assert(EasyTVC_Sim3DBegin(p, SIM3D_PARAM_COUNT) == 0);
    got = 0;
    got += EasyTVC_Sim3DAdvance(p[SIM3D_IGNITION_S] + 0.5f, live_rows, 6000, live_sum, SIM3D_SUMMARY_COUNT);
    assert(EasyTVC_Sim3DInject(SIM3D_INJECT_JAM_SERVO, 0, 0) == 0);
    while (live_sum[SIM3D_SUM_DONE] < 0.5f && got < 6000)
        got += EasyTVC_Sim3DAdvance(0.5f, live_rows + got * SIM3D_COLUMN_COUNT, 6000 - got, live_sum, SIM3D_SUMMARY_COUNT);
    float worst_x = 0, worst_y = 0, horn_a_moves = 0, last_horn_a = NAN;
    for (int i = 0; i < got; ++i) {
        const float *r = live_rows + i * SIM3D_COLUMN_COUNT;
        if (r[SIM3D_COL_STATE] != 2.0f) continue;
        if (r[SIM3D_COL_T] > p[SIM3D_IGNITION_S] + 0.55f) {
            worst_x = fmaxf(worst_x, fabsf(r[SIM3D_COL_TILT_X])); worst_y = fmaxf(worst_y, fabsf(r[SIM3D_COL_TILT_Y]));
            if (!isnan(last_horn_a) && r[SIM3D_COL_HORN_A] != last_horn_a) ++horn_a_moves;
            last_horn_a = r[SIM3D_COL_HORN_A];
        }
    }
    printf("  servo A jammed in boost: worst tilt X %.1f deg (Y stays %.1f), jammed horn moved %g times\n", worst_x, worst_y, horn_a_moves);
    assert(horn_a_moves == 0);
    assert(live_sum[SIM3D_SUM_TVC_FAULT] == 1.0f || worst_x > 3.0f * worst_y);

    /* 3. while TVC is active the gimbal's torque opposes the aerodynamic upset torque */
    defaults(p);
    p[SIM3D_WIND_M_S] = 3; p[SIM3D_GUST_M_S] = 6; p[SIM3D_MISALIGN_DEG] = 0;
    const int n3 = run(p, sum);
    double tvc_dot_aero = 0, aero_sq = 0;
    for (int i = 0; i < n3; ++i) {
        const float *r = rows + i * SIM3D_COLUMN_COUNT;
        if (r[SIM3D_COL_TVC] < 0.5f) continue;
        tvc_dot_aero += (double)r[SIM3D_COL_TQ_TVC_X] * r[SIM3D_COL_TQ_AERO_X] + (double)r[SIM3D_COL_TQ_TVC_Y] * r[SIM3D_COL_TQ_AERO_Y];
        aero_sq += (double)r[SIM3D_COL_TQ_AERO_X] * r[SIM3D_COL_TQ_AERO_X] + (double)r[SIM3D_COL_TQ_AERO_Y] * r[SIM3D_COL_TQ_AERO_Y];
    }
    printf("  TVC vs aero torque correlation during boost: %.2f (negative = TVC fights the upset)\n", tvc_dot_aero / aero_sq);
    assert(tvc_dot_aero < 0.0);

    /* 4. a kick in flight is caught and corrected (still air, so the kick is the only upset) */
    defaults(p);
    p[SIM3D_TURBULENCE_M_S] = 0; p[SIM3D_WIND_M_S] = 0; p[SIM3D_GUST_M_S] = 0;
    assert(EasyTVC_Sim3DBegin(p, SIM3D_PARAM_COUNT) == 0);
    got = EasyTVC_Sim3DAdvance(p[SIM3D_IGNITION_S] + 0.6f, live_rows, 6000, live_sum, SIM3D_SUMMARY_COUNT);
    assert(EasyTVC_Sim3DInject(SIM3D_INJECT_KICK, 40.0f, 0.0f) == 0);
    got += EasyTVC_Sim3DAdvance(1.2f, live_rows + got * SIM3D_COLUMN_COUNT, 6000 - got, live_sum, SIM3D_SUMMARY_COUNT);
    const float *end = live_rows + (got - 1) * SIM3D_COLUMN_COUNT;
    float peak = 0;
    for (int i = 0; i < got; ++i) peak = fmaxf(peak, fabsf(live_rows[i * SIM3D_COLUMN_COUNT + SIM3D_COL_TILT_X]));
    printf("  40 deg/s kick in boost: peak %.1f deg, 1.2 s later %.1f deg\n", peak, end[SIM3D_COL_TILT_X]);
    assert(peak > 2.0f && fabsf(end[SIM3D_COL_TILT_X]) < 0.5f * peak && live_sum[SIM3D_SUM_TVC_FAULT] == 0.0f);

    /* 5. manual TVC: the PID is out of the loop, the gimbal follows the stick */
    defaults(p);
    p[SIM3D_TURBULENCE_M_S] = 0; p[SIM3D_WIND_M_S] = 0; p[SIM3D_GUST_M_S] = 0; p[SIM3D_MISALIGN_DEG] = 0; p[SIM3D_PAD_TILT_DEG] = 0;
    assert(EasyTVC_Sim3DBegin(p, SIM3D_PARAM_COUNT) == 0);
    got = EasyTVC_Sim3DAdvance(p[SIM3D_IGNITION_S] + 0.4f, live_rows, 6000, live_sum, SIM3D_SUMMARY_COUNT);
    assert(EasyTVC_Sim3DInject(SIM3D_INJECT_MANUAL, 0.5f, 0.0f) == 0);
    got += EasyTVC_Sim3DAdvance(0.4f, live_rows + got * SIM3D_COLUMN_COUNT, 6000 - got, live_sum, SIM3D_SUMMARY_COUNT);
    const float *m = live_rows + (got - 1) * SIM3D_COLUMN_COUNT;
    printf("  manual stick A +0.5 for 0.4 s: gimbal A %.2f deg, tilt X %.1f deg (manual flag %g)\n",
           m[SIM3D_COL_GIMBAL_A], m[SIM3D_COL_TILT_X], m[SIM3D_COL_SERVO_FAULT]);
    assert(((int)m[SIM3D_COL_SERVO_FAULT] & 4) && m[SIM3D_COL_GIMBAL_A] > 2.0f && m[SIM3D_COL_TILT_X] > 5.0f);
    assert(EasyTVC_Sim3DInject(SIM3D_INJECT_AUTO, 0, 0) == 0);

    /* 6. measured E12 curve (NAR data) through SetCurve */
    static const float et[] = {0.052f, 0.096f, 0.196f, 0.251f, 0.287f, 0.3f, 0.344f, 0.37f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f,
        1.101f, 1.2f, 1.3f, 1.4f, 1.5f, 1.6f, 1.7f, 1.8f, 1.9f, 2.0f, 2.1f, 2.2f, 2.3f, 2.375f, 2.4f, 2.44f};
    static const float ef[] = {5.045f, 9.91f, 24.144f, 31.351f, 32.973f, 29.91f, 17.117f, 14.414f, 12.973f, 11.712f, 11.171f, 10.631f,
        10.09f, 9.73f, 9.55f, 9.91f, 9.55f, 9.73f, 9.73f, 9.73f, 9.73f, 9.55f, 9.73f, 9.73f, 9.55f, 9.55f, 9.73f, 9.19f, 9.37f, 5.95f, 0.0f};
    assert(EasyTVC_Sim3DSetCurve(et, ef, sizeof(et) / sizeof(et[0])) == 0);
    defaults(p);
    p[SIM3D_THRUST_PROFILE] = 2;
    run(p, sum);
    printf("  measured E12 curve: apogee %.0f m at %.1f s, boost tilt %.1f deg\n", sum[SIM3D_SUM_APOGEE_M], sum[SIM3D_SUM_APOGEE_T], sum[SIM3D_SUM_MAX_BOOST_TILT]);
    assert(sum[SIM3D_SUM_LAUNCH_T] >= 0 && sum[SIM3D_SUM_APOGEE_M] > 50.0f);
    const float bad_t[] = {0.1f, 0.05f};
    const float bad_f[] = {1.0f, 2.0f};
    assert(EasyTVC_Sim3DSetCurve(bad_t, bad_f, 2) < 0);

    puts("3D sim tests passed: nominal flight, no-authority tumble, reversed servo, pad tilt, overweight");
    return 0;
}
