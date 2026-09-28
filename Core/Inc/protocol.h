#pragma once

#include "settings.h"
#include "easytvc_safety.h"

#define EASYTVC_PROTOCOL_CAPACITY 256U

typedef bool (*EasyTVCSaveSettings)(void *context, const uint8_t *data, size_t length);

typedef struct {
    EasyTVCSettings settings;
    EasyTVCFlightState state;
    bool hardware_verified;
    bool physical_arm;
    bool settings_persisted;
    EasyTVCSaveSettings save;
    void *context;
    char line[EASYTVC_PROTOCOL_CAPACITY];
    size_t used;
    bool dropping;
} EasyTVCProtocol;

void EasyTVC_ProtocolInit(EasyTVCProtocol *protocol);
/* Feed from a bounded USB RX queue outside the control interrupt. One response
 * per complete line. No motor, pyro, arming or bootloader commands exist. */
size_t EasyTVC_ProtocolFeed(EasyTVCProtocol *protocol, char byte,
                          char response[EASYTVC_PROTOCOL_CAPACITY]);
