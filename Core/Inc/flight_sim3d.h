#pragma once

/*
 * 6-DOF flight simulation with the REAL flight software in the loop: pad arming (pad_arm.c),
 * attitude estimation, flight state machine, TVC controller (app.c) and parachute logic run
 * exactly as on the board, fed with simulated IMU/baro samples at 200 Hz. The vehicle is a rigid
 * body with a gimballed motor, thrust curve, mass burn-off, drag, normal force at the CP, wind,
 * servo slew/frame-rate limits and a parachute. It is a planning tool, not a flight predictor.
 *
 * Scalar-only FFI for Studio (ctypes). Parameter and column orders are fixed; keep
 * studio/launch.py (SIM3D_PARAMS / SIM3D_COLUMNS) in sync.
 */

enum {
    /* firmware configuration (vehicle_config.h) */
    SIM3D_KP, SIM3D_KI, SIM3D_KD, SIM3D_INTEGRATOR_LIMIT, SIM3D_RATE_LIMIT, SIM3D_DERIVATIVE_CUTOFF,
    SIM3D_MAX_TILT_DEG,
    SIM3D_A_MIN, SIM3D_A_CENTER, SIM3D_A_MAX, SIM3D_A_DIRECTION,
    SIM3D_B_MIN, SIM3D_B_CENTER, SIM3D_B_MAX, SIM3D_B_DIRECTION,
    SIM3D_SERVO_RATE_HZ,
    SIM3D_LAUNCH_G, SIM3D_LAUNCH_CONFIRM_MS, SIM3D_TVC_DELAY_MS,
    SIM3D_CHUTE_ENABLED, SIM3D_CHUTE_BACKUP_MS,
    SIM3D_ARM_HOLD_MS, SIM3D_ARM_TILT_DEG, SIM3D_DISARM_TILT_DEG,
    /* vehicle */
    SIM3D_DRY_MASS_KG, SIM3D_MOTOR_MASS_KG, SIM3D_PROP_MASS_KG, SIM3D_AVG_THRUST_N, SIM3D_BURN_S,
    SIM3D_THRUST_PROFILE,              /* 0 = flat average, 1 = Estes-style shape, 2 = measured curve (SetCurve) */
    SIM3D_DIAMETER_M, SIM3D_BODY_CD, SIM3D_CHUTE_DIAMETER_M, SIM3D_CHUTE_CD,
    /* simulation model */
    /* mass properties at liftoff, from the rocket model (all distances along the body axis) */
    SIM3D_LENGTH_M, SIM3D_GIMBAL_ARM_M,   /* gimbal pivot behind the CG */
    SIM3D_CP_AHEAD_M,                     /* CP ahead of the CG (+ = unstable) */
    SIM3D_INERTIA,                        /* pitch/yaw inertia about the CG, kg*m^2 (0 = estimate) */
    SIM3D_CN_ALPHA,                       /* nose normal-force slope per rad (Barrowman: 2) */
    /* servo linkage: horn angle = (pulse - centre) * deg/us; gimbal = asin(ratio * sin(horn)) */
    SIM3D_LINKAGE_RATIO,                  /* horn radius / gimbal lever radius */
    SIM3D_SERVO_SPEED_DPS,                /* servo horn speed, deg/s (MG90D ~600) */
    SIM3D_WIND_M_S, SIM3D_GUST_M_S, SIM3D_WIND_DIR_DEG, SIM3D_PAD_TILT_DEG, SIM3D_MISALIGN_DEG,
    SIM3D_GYRO_BIAS_DPS, SIM3D_NOISE, SIM3D_SEED, SIM3D_REVERSE_SERVO_A, SIM3D_TOWER_M,
    SIM3D_SERVO_DEG_PER_US,               /* servo horn degrees per microsecond (~0.09) */
    SIM3D_GIMBAL_STOP_DEG,                /* mechanical limit of the gimbal */
    SIM3D_CG_SHIFT_M,                     /* CG moves this far toward the nose by burnout */
    SIM3D_INERTIA_BURNOUT,                /* inertia at burnout (0 = estimate) */
    SIM3D_PLANFORM_AREA_M2,               /* side-view area, for body lift at high angle of attack */
    SIM3D_PLANFORM_AHEAD_M,               /* planform centroid ahead of the CG */
    /* real servo behaviour */
    SIM3D_SERVO_DEADBAND_US,              /* pulse change the servo ignores (MG90D ~4-5 us) */
    SIM3D_SERVO_BACKLASH_DEG,             /* gear + linkage slop at the horn, total, deg */
    SIM3D_SERVO_TAU_S,                    /* servo position-loop time constant (~0.01 s) */
    /* physics fidelity */
    SIM3D_THRUST_SCALE,                   /* motor-to-motor spread, 1 = nominal */
    SIM3D_SITE_ALT_M, SIM3D_TEMP_C,        /* launch site, for the standard atmosphere */
    SIM3D_NOZZLE_ARM_M,                   /* CG -> nozzle exit, for jet damping */
    SIM3D_MOTOR_GIMBAL_INERTIA,           /* gimballed motor about the pivot, kg*m^2 (reaction torque) */
    SIM3D_SERVO_STALL_NM,                 /* servo stall torque at the horn, N*m (MG90D ~0.2) */
    SIM3D_IMU_AHEAD_M,                    /* IMU position ahead of the CG (lever-arm accelerations) */
    SIM3D_TURBULENCE_M_S,                 /* Dryden-like turbulence intensity, m/s RMS */
    SIM3D_GYRO_LPF_HZ, SIM3D_ACCEL_LPF_HZ, /* BMI088 internal filters (47 / ~145 Hz at 400 Hz ODR) */
    SIM3D_IGNITION_S,
    SIM3D_PARAM_COUNT
};

enum {
    SIM3D_COL_T, SIM3D_COL_X, SIM3D_COL_Y, SIM3D_COL_Z, SIM3D_COL_QW, SIM3D_COL_QX, SIM3D_COL_QY,
    SIM3D_COL_QZ, SIM3D_COL_SPEED, SIM3D_COL_VZ, SIM3D_COL_TILT_X, SIM3D_COL_TILT_Y,
    SIM3D_COL_GIMBAL_A, SIM3D_COL_GIMBAL_B, SIM3D_COL_STATE, SIM3D_COL_TVC, SIM3D_COL_CHUTE,
    SIM3D_COL_THRUST, SIM3D_COL_EST_TILT_X, SIM3D_COL_EST_TILT_Y, SIM3D_COL_ARMED,
    SIM3D_COL_PULSE_A, SIM3D_COL_PULSE_B, SIM3D_COL_HORN_A, SIM3D_COL_HORN_B, SIM3D_COL_ALPHA,
    /* moments about the CG in body X/Y, N*m: what actually turns the rocket */
    SIM3D_COL_TQ_TVC_X, SIM3D_COL_TQ_TVC_Y, SIM3D_COL_TQ_AERO_X, SIM3D_COL_TQ_AERO_Y,
    SIM3D_COL_SERVO_FAULT,                /* bit0 servo A jammed, bit1 servo B jammed, bit2 manual TVC */
    SIM3D_COL_WIND,                       /* horizontal wind speed at the rocket, m/s */
    SIM3D_COLUMN_COUNT
};

enum {
    SIM3D_SUM_APOGEE_M, SIM3D_SUM_APOGEE_T, SIM3D_SUM_MAX_SPEED, SIM3D_SUM_MAX_BOOST_TILT,
    SIM3D_SUM_DRIFT_M, SIM3D_SUM_LANDING_SPEED, SIM3D_SUM_CHUTE_T, SIM3D_SUM_CHUTE_SPEED,
    SIM3D_SUM_ARMED_T, SIM3D_SUM_LAUNCH_T, SIM3D_SUM_TVC_FAULT, SIM3D_SUM_SATURATION_PCT,
    SIM3D_SUM_FLIGHT_TIME, SIM3D_SUM_FINAL_STATE, SIM3D_SUM_UNDER_CHUTE, SIM3D_SUM_EST_ERROR_DEG,
    SIM3D_SUM_LIFTOFF, SIM3D_SUM_DONE, SIM3D_SUM_TIME, SIM3D_SUMMARY_COUNT
};

#define SIM3D_ROW_PERIOD_S 0.02f

/* Whole flight in one call. Returns the number of rows written (>= 1), or < 0 for bad input. */
int EasyTVC_Sim3D(const float *params, unsigned param_count, float *rows, unsigned max_rows,
                  float *summary, unsigned summary_count);

/* Live (stepped) simulation, one instance: Begin, then Advance repeatedly as wall time passes.
 * Advance returns the rows produced during that slice (50 Hz); summary[SIM3D_SUM_DONE] becomes 1
 * once the rocket has landed (or the sim timed out). */
int EasyTVC_Sim3DBegin(const float *params, unsigned param_count);
int EasyTVC_Sim3DAdvance(float seconds, float *rows, unsigned max_rows, float *summary, unsigned summary_count);

/* Disturbances injected into a live flight. */
enum {
    SIM3D_INJECT_KICK = 1,         /* a = body-X rate kick, b = body-Y rate kick, deg/s */
    SIM3D_INJECT_GUST = 2,         /* a = extra wind m/s, b = duration s */
    SIM3D_INJECT_JAM_SERVO = 3,    /* a = 0 servo A / 1 servo B: the horn freezes where it is */
    SIM3D_INJECT_FREE_SERVO = 4,   /* a = 0 / 1: release a jammed servo */
    SIM3D_INJECT_MANUAL = 5,       /* manual TVC: a, b = commands -1..1 for servo A / B (overrides the PID) */
    SIM3D_INJECT_AUTO = 6,         /* hand TVC back to the flight software */
};
int EasyTVC_Sim3DInject(int kind, float a, float b);

/* Measured thrust curve (RASP .eng points, seconds / newtons) used by THRUST_PROFILE 2. Applies to
 * every later Begin / batch run. Up to 128 points; returns 0 or < 0 for bad data. */
int EasyTVC_Sim3DSetCurve(const float *times, const float *newtons, unsigned count);
