#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bus.h"

#define EASYTVC_BME280_CHIP_ID_REG 0xD0U
#define EASYTVC_BME280_CHIP_ID     0x60U

typedef struct {
    EasyTVCBus *bus;
    bool identity_confirmed;
} EasyTVCBme280;

void EasyTVC_Bme280Init(EasyTVCBme280 *device, EasyTVCBus *bus);
bool EasyTVC_Bme280Probe(EasyTVCBme280 *device);
