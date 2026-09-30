#pragma once

#include <stddef.h>
#include <stdint.h>

/*
 * Abstract register bus. Pin nets are in board_pins.h (vendor hwdef). SPI/I2C
 * adapters still must not write GPIO from the blank image. Host tests inject fakes.
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
