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
    puts("3D sim tests passed: nominal flight, no-authority tumble, reversed servo, pad tilt, overweight");
    return 0;
}
