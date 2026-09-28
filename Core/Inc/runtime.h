#pragma once

#include "app.h"
#include "protocol.h"

/* Single-owner portable runtime. The board's main loop serializes step/feed;
 * interrupt handlers only queue samples/bytes. No hardware access is performed here. */
typedef struct {
    EasyTVCApp app;
    EasyTVCProtocol protocol;
    EasyTVCSaveSettings persist;
    void *persist_context;
} EasyTVCRuntime;

void EasyTVC_RuntimeInit(EasyTVCRuntime *runtime, uint32_t now_ms, bool hardware_verified,
                        EasyTVCSaveSettings persist, void *persist_context);
bool EasyTVC_RuntimeLoadSettings(EasyTVCRuntime *runtime, const uint8_t *data, size_t size,
                                bool physical_arm);
EasyTVCAppOutput EasyTVC_RuntimeStep(EasyTVCRuntime *runtime, const EasyTVCAppInput *sample);
size_t EasyTVC_RuntimeFeed(EasyTVCRuntime *runtime, char byte, bool physical_arm,
                          char response[EASYTVC_PROTOCOL_CAPACITY]);
