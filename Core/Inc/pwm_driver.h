#pragma once

#include <stdint.h>
#include <stdbool.h>

void EasyTVC_PWM_Init(void);
void EasyTVC_PWM_SetChannel(uint8_t channel, uint16_t pulse_us);
