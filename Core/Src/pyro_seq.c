#include "pyro_seq.h"

static void start_pulse(EasyTVCPyroChannel *channel,
                        EasyTVCPyroEvent event,
                        uint32_t now_ms,
                        uint32_t pulse_ms)
{
    channel->pulse_latched = true;
    channel->event = event;
    channel->pulse_end_ms = now_ms + pulse_ms;
}

static bool channel_active(EasyTVCPyroChannel *channel, uint32_t now_ms)
{
    if (!channel->pulse_latched) {
        return false;
    }
    if ((int32_t)(now_ms - channel->pulse_end_ms) >= 0) {
        channel->pulse_latched = false;
        return false;
    }
    return true;
}

void EasyTVC_PyroSequencerInit(EasyTVCPyroSequencer *sequencer)
{
    sequencer->drogue.pulse_latched = false;
    sequencer->drogue.pulse_end_ms = 0U;
    sequencer->drogue.event = EASYTVC_PYRO_EVENT_DROGUE;
    sequencer->main.pulse_latched = false;
    sequencer->main.pulse_end_ms = 0U;
    sequencer->main.event = EASYTVC_PYRO_EVENT_MAIN;
}

EasyTVCPyroCommand EasyTVC_PyroSequencerStep(EasyTVCPyroSequencer *sequencer,
                                             const EasyTVCPyroConfig *config,
                                             EasyTVCFlightState state,
                                             bool physical_arm_qualified,
                                             bool drogue_event,
                                             bool main_event,
                                             uint32_t now_ms)
{
    EasyTVCPyroCommand command = {
        .request_drogue_output = false,
        .request_main_output = false,
    };

    if (drogue_event &&
        EasyTVC_PyroPolicyAllows(state, EASYTVC_PYRO_EVENT_DROGUE, physical_arm_qualified)) {
        start_pulse(&sequencer->drogue, EASYTVC_PYRO_EVENT_DROGUE, now_ms, config->pulse_ms);
    }
    if (main_event &&
        EasyTVC_PyroPolicyAllows(state, EASYTVC_PYRO_EVENT_MAIN, physical_arm_qualified)) {
        start_pulse(&sequencer->main, EASYTVC_PYRO_EVENT_MAIN, now_ms, config->pulse_ms);
    }

    /*
     * The sequencer may remember a software pulse, but the compiled-in lock
     * keeps every hardware request false. Do not treat these flags as GPIO.
     */
    command.request_drogue_output =
        EasyTVC_PyroPolicyAllows(state, EASYTVC_PYRO_EVENT_DROGUE, physical_arm_qualified) &&
        channel_active(&sequencer->drogue, now_ms);
    command.request_main_output =
        EasyTVC_PyroPolicyAllows(state, EASYTVC_PYRO_EVENT_MAIN, physical_arm_qualified) &&
        channel_active(&sequencer->main, now_ms);
    return command;
}
