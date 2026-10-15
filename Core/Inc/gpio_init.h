#pragma once

#include <stdint.h>
#include <stdbool.h>

void EasyTVC_GPIO_Init(void);
void EasyTVC_GPIO_LedSet(uint8_t led, bool state);
/* Called from the 1 kHz SysTick: software PWM that dims the status LED to EASYTVC_LED_BRIGHTNESS_PCT. */
void EasyTVC_GPIO_LedTick(uint32_t now_ms);
void EasyTVC_GPIO_BuzzerSet(bool state);
/* The PLL did not lock: the whole timing/baud/bus budget is wrong, so never run the vehicle.
 * Blinks the red LED forever with a plain busy-wait (no timers needed). */
void EasyTVC_GPIO_HaltClockFault(void) __attribute__((noreturn));
