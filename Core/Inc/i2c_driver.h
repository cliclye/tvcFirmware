#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "bus.h"

void EasyTVC_I2C1_Init(void);
void EasyTVC_I2C2_Init(void);

int EasyTVC_I2C1_Write(uint8_t addr, uint8_t reg, const uint8_t *data, size_t length);
int EasyTVC_I2C1_Read(uint8_t addr, uint8_t reg, uint8_t *data, size_t length);

int EasyTVC_I2C2_Write(uint8_t addr, uint8_t reg, const uint8_t *data, size_t length);
int EasyTVC_I2C2_Read(uint8_t addr, uint8_t reg, uint8_t *data, size_t length);
