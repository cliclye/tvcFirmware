#pragma once

#include <stdint.h>

/* ADC1, single conversions: the battery-sense pin (PA4 / IN4), the internal reference and the MCU
 * temperature sensor. Each read takes ~25 us (480-cycle sampling), so call it a few times a second,
 * not every control tick. */
#define EASYTVC_ADC_CH_TEMP 16U
#define EASYTVC_ADC_CH_VREF 17U

void EasyTVC_ADC_Init(void);
/* 12-bit result, or 0xFFFF if the conversion timed out. */
uint16_t EasyTVC_ADC_Read(uint32_t channel);
/* 3.3 V rail in mV from a VREFINT reading (factory calibration at 3.3 V), 0 if implausible. */
uint16_t EasyTVC_ADC_VddaMv(uint16_t vref_raw);
/* The factory VREFINT calibration word (raw count at 3.3 V, 30 C). */
uint16_t EasyTVC_ADC_VrefintCal(void);
