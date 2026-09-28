#pragma once

#include <stddef.h>
#include <stdint.h>

/*
 * Abstract register bus. Hardware SPI/I2C adapters must not be added until
 * chip-select, address, and pin nets are confirmed. Host tests inject fakes.
 */
typedef struct EasyTVCBus EasyTVCBus;

struct EasyTVCBus {
    void *context;
    int (*write)(EasyTVCBus *bus, uint8_t reg, const uint8_t *data, size_t length);
    int (*read)(EasyTVCBus *bus, uint8_t reg, uint8_t *data, size_t length);
};

int EasyTVC_BusWrite(EasyTVCBus *bus, uint8_t reg, const uint8_t *data, size_t length);
int EasyTVC_BusRead(EasyTVCBus *bus, uint8_t reg, uint8_t *data, size_t length);
int EasyTVC_BusReadU8(EasyTVCBus *bus, uint8_t reg, uint8_t *value);
