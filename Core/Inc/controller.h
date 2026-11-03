#pragma once

#include "settings.h"
#include "attitude.h"
#include <stdint.h>

/* Roll compensation. A finless rocket rolls freely and the servos act in the body frame, so:
 * - the tilt error is always taken in the current body frame (from the full quaternion);
 * - the derivative and its filter are carried across the roll between samples, so a rolling body
 *   does not read as tilt rate (that fake rate was cross-coupling the axes);
 * - the command is rotated ahead by roll_rate * EASYTVC_ROLL_LEAD_S, the time from command to the
 *   gimbal actually moving (servo frame + servo response; 15 ms was best in the 3D sim), so the push lands where it was meant.
 * Without these the TVC lost control above ~500 deg/s of roll in the 3D sim; with them it held
 * every flight up to 2000 deg/s. */
#ifndef EASYTVC_ROLL_LEAD_S
#define EASYTVC_ROLL_LEAD_S 0.015f
#endif

/* A missed or stale IMU sample in boost (one failed SPI read, one late tick) holds the last
 * command instead of switching TVC off: losing control for the rest of the burn is far worse for
 * a finless rocket than steering on a 5-10 ms old command. Only a problem that lasts this long
 * latches the fault. Hard faults (bad settings, a broken attitude, the tilt limit, the arm lost)
 * still latch at once. */
#ifndef EASYTVC_CONTROL_HOLD_MS
#define EASYTVC_CONTROL_HOLD_MS 100U
#endif

typedef struct {
    EasyTVCPid axis[2];
    EasyTVCQuaternion previous_q;
    float last_command[2];   /* held through a soft fault */
    float held_s;            /* time spent holding: added to the next PID step's dt */
    uint32_t hold_since_ms;
    bool holding;
    float roll_rate_rad_s;   /* body Z, from successive attitude samples */
    bool roll_rate_valid;
    bool fault_latched;
    uint32_t last_sample_ms;
    bool sample_seen;
} EasyTVCController;

typedef struct {
    float command[2];
    uint16_t pulse_us[2];
    bool enabled;
    bool fault;
} EasyTVCControlOutput;

void EasyTVC_ControllerReset(EasyTVCController *controller);
/* Tilt of the nose away from world-up, as rotations about body X [0] and body Y [1] (rad). */
void EasyTVC_ControllerTilt(const EasyTVCQuaternion *orientation, float tilt_rad[2]);
/* A new sensor sample is required for every call. Reset only while disarmed.
 * Positive controller command assumes positive body-axis torque; prove direction on a bench. */
EasyTVCControlOutput EasyTVC_ControllerStep(EasyTVCController *controller,
    const EasyTVCSettings *settings, const EasyTVCQuaternion *orientation,
    uint32_t now_ms, uint32_t sample_ms, float dt_s, bool healthy,
    bool physical_arm, bool boost_enabled);
