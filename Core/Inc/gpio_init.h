#pragma once

#include <stdint.h>
#include <stdbool.h>

void EasyTVC_GPIO_Init(void);
void EasyTVC_GPIO_LedSet(uint8_t led, bool state);
/* Called from the 1 kHz SysTick: software PWM that dims the status LED to EASYTVC_LED_BRIGHTNESS_PCT. */
void EasyTVC_GPIO_LedTick(uint32_t now_ms);
void EasyTVC_GPIO_BuzzerSet(bool state);
