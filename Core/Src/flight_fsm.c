#include "flight_fsm.h"

#include <math.h>

static bool elapsed_at_least(uint32_t now_ms, uint32_t since_ms, uint32_t duration_ms)
{
    return (uint32_t)(now_ms - since_ms) >= duration_ms;
}

static float absolute_value(float value)
{
    return value < 0.0f ? -value : value;
}

static void transition(EasyTVCFlightMachine *machine,
                       EasyTVCFlightState next_state,
                       uint32_t now_ms)
{
    machine->state = next_state;
    machine->state_entered_ms = now_ms;
    machine->landed_candidate_active = false;
}

void EasyTVC_FlightInit(EasyTVCFlightMachine *machine,
                        const EasyTVCFlightConfig *config,
                        uint32_t now_ms)
{
    machine->config = *config;
    machine->state = EASYTVC_FLIGHT_IDLE;
    machine->state_entered_ms = now_ms;
    machine->launch_candidate_ms = now_ms;
    machine->launch_ms = now_ms;
    machine->landed_candidate_ms = now_ms;
    machine->launch_candidate_active = false;
    machine->landed_candidate_active = false;
    machine->drogue_requested = false;
    machine->main_requested = false;
}

EasyTVCFlightOutput EasyTVC_FlightStep(EasyTVCFlightMachine *machine,
                                       const EasyTVCFlightInput *input)
{
    bool drogue_event = false;
    bool main_event = false;
    EasyTVCFlightOutput output = {
        .state = machine->state,
        .log_active = false,
        .tvc_enabled = false,
        .drogue_deploy_requested = false,
        .main_deploy_requested = false,
        .fault = false,
    };

    const bool valid = input->sensors_healthy &&
        isfinite(input->acceleration_magnitude_g) && isfinite(input->axial_acceleration_g) &&
        isfinite(input->altitude_m) && isfinite(input->vertical_speed_m_s);
    if (!valid && machine->state != EASYTVC_FLIGHT_IDLE &&
        machine->state != EASYTVC_FLIGHT_LANDED) {
        transition(machine, EASYTVC_FLIGHT_FAULT, input->now_ms);
    }
    if (machine->state == EASYTVC_FLIGHT_BOOST && !input->physical_arm_qualified) {
        transition(machine, EASYTVC_FLIGHT_FAULT, input->now_ms);
    }

    switch (machine->state) {
    case EASYTVC_FLIGHT_IDLE:
        machine->launch_candidate_active = false;
        if (valid && input->physical_arm_qualified &&
            input->software_arm_request) {
            transition(machine, EASYTVC_FLIGHT_ARMED, input->now_ms);
        }
        break;

    case EASYTVC_FLIGHT_ARMED:
        if (!input->sensors_healthy) {
            transition(machine, EASYTVC_FLIGHT_FAULT, input->now_ms);
        } else if (!input->physical_arm_qualified || !input->software_arm_request) {
            transition(machine, EASYTVC_FLIGHT_IDLE, input->now_ms);
        } else if (input->acceleration_magnitude_g >= machine->config.launch_accel_g &&
                   input->axial_acceleration_g >= machine->config.launch_accel_g * 0.5f) {
            if (!machine->launch_candidate_active) {
                machine->launch_candidate_active = true;
                machine->launch_candidate_ms = input->now_ms;
            } else if (elapsed_at_least(input->now_ms,
                                        machine->launch_candidate_ms,
                                        machine->config.launch_confirm_ms)) {
                machine->launch_ms = input->now_ms;
                transition(machine, EASYTVC_FLIGHT_BOOST, input->now_ms);
            }
        } else {
            machine->launch_candidate_active = false;
        }
        break;

    case EASYTVC_FLIGHT_BOOST:
        if (!input->sensors_healthy) {
            transition(machine, EASYTVC_FLIGHT_FAULT, input->now_ms);
        } else if (elapsed_at_least(input->now_ms,
                                    machine->launch_ms,
                                    machine->config.minimum_boost_ms) &&
                   input->axial_acceleration_g < machine->config.burnout_axial_accel_g) {
            transition(machine, EASYTVC_FLIGHT_COAST, input->now_ms);
        }
        break;

    case EASYTVC_FLIGHT_COAST:
        if (!input->sensors_healthy) {
            transition(machine, EASYTVC_FLIGHT_FAULT, input->now_ms);
        } else if (elapsed_at_least(input->now_ms,
                                    machine->launch_ms,
                                    machine->config.failsafe_apogee_ms) ||
                   (elapsed_at_least(input->now_ms,
                                     machine->launch_ms,
                                     machine->config.minimum_apogee_ms) &&
                    input->vertical_speed_m_s <= 0.0f)) {
            machine->drogue_requested = true;
            drogue_event = true;
            transition(machine, EASYTVC_FLIGHT_APOGEE, input->now_ms);
        }
        break;

    case EASYTVC_FLIGHT_APOGEE:
        transition(machine, EASYTVC_FLIGHT_DESCENT, input->now_ms);
        break;

    case EASYTVC_FLIGHT_DESCENT:
        if (!machine->main_requested &&
            input->vertical_speed_m_s < 0.0f &&
            input->altitude_m <= machine->config.main_deploy_altitude_m) {
            machine->main_requested = true;
            main_event = true;
        }

        if (absolute_value(input->vertical_speed_m_s) <= machine->config.landed_speed_m_s) {
            if (!machine->landed_candidate_active) {
                machine->landed_candidate_active = true;
                machine->landed_candidate_ms = input->now_ms;
            } else if (elapsed_at_least(input->now_ms,
                                        machine->landed_candidate_ms,
                                        machine->config.landed_confirm_ms)) {
                transition(machine, EASYTVC_FLIGHT_LANDED, input->now_ms);
            }
        } else {
            machine->landed_candidate_active = false;
        }
        break;

    case EASYTVC_FLIGHT_LANDED:
        break;

    case EASYTVC_FLIGHT_FAULT:
        break;

    default:
        transition(machine, EASYTVC_FLIGHT_FAULT, input->now_ms);
        break;
    }

    output.state = machine->state;
    output.log_active = machine->state != EASYTVC_FLIGHT_IDLE &&
                        machine->state != EASYTVC_FLIGHT_LANDED;
    output.tvc_enabled = machine->state == EASYTVC_FLIGHT_BOOST &&
                         elapsed_at_least(input->now_ms,
                                          machine->launch_ms,
                                          machine->config.tvc_enable_delay_ms);
    output.drogue_deploy_requested = drogue_event;
    output.main_deploy_requested = main_event;
    output.fault = machine->state == EASYTVC_FLIGHT_FAULT;
    return output;
}
