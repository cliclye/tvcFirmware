#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "easytvc_safety.h"

typedef struct {
    uint32_t pulse_ms;
} EasyTVCPyroConfig;

typedef struct {
    bool pulse_latched;
    uint32_t pulse_end_ms;
    EasyTVCPyroEvent event;
} EasyTVCPyroChannel;

typedef struct {
    EasyTVCPyroChannel drogue;
    EasyTVCPyroChannel main;
} EasyTVCPyroSequencer;

typedef struct {
    bool request_drogue_output;
    bool request_main_output;
} EasyTVCPyroCommand;

void EasyTVC_PyroSequencerInit(EasyTVCPyroSequencer *sequencer);
EasyTVCPyroCommand EasyTVC_PyroSequencerStep(EasyTVCPyroSequencer *sequencer,
                                             const EasyTVCPyroConfig *config,
                                             EasyTVCFlightState state,
                                             bool physical_arm_qualified,
                                             bool drogue_event,
                                             bool main_event,
                                             uint32_t now_ms);
