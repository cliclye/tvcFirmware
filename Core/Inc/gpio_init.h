#pragma once

#include <stdint.h>
#include <stdbool.h>

void EasyTVC_GPIO_Init(void);
void EasyTVC_GPIO_LedSet(uint8_t led, bool state);
void EasyTVC_GPIO_BuzzerSet(bool state);
