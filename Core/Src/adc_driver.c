#include "adc_driver.h"
#include "board_pins.h"
#include "stm32f4xx.h"

/* ADC common registers (not in stm32f4xx.h) and the factory VREFINT calibration (3.3 V, 30 C). */
#define ADC_COMMON_CCR      (*(volatile uint32_t *)(STM32_ADC1_BASE + 0x300UL + 0x04UL))   /* 0x40012304 */
#define ADC_CCR_TSVREFE     (1UL << 23)
#define ADC_CCR_ADCPRE_DIV4 (1UL << 16)               /* 84 MHz / 4 = 21 MHz (max 36) */
#define VREFINT_CAL         (*(const volatile uint16_t *)0x1FFF7A2AUL)

void EasyTVC_ADC_Init(void)
{
    ADC_COMMON_CCR = (ADC_COMMON_CCR & ~(3UL << 16)) | ADC_CCR_ADCPRE_DIV4 | ADC_CCR_TSVREFE;
    STM32_ADC1->CR1 = 0;                                   /* 12-bit, single channel */
    STM32_ADC1->CR2 = STM32_ADC_CR2_ADON;
    /* 480-cycle sampling: the temperature sensor needs >= 10 us */
    STM32_ADC1->SMPR1 = (7UL << ((EASYTVC_ADC_CH_TEMP - 10U) * 3U)) | (7UL << ((EASYTVC_ADC_CH_VREF - 10U) * 3U));
    STM32_ADC1->SMPR2 = 7UL << (EASYTVC_ADC_BATT_CHANNEL * 3U);
    STM32_DelayMicroseconds(20U);
}

uint16_t EasyTVC_ADC_Read(uint32_t channel)
{
    STM32_ADC1->SQR1 = 0;                                  /* one conversion */
    STM32_ADC1->SQR3 = channel;
    STM32_ADC1->SR = 0;
    STM32_ADC1->CR2 |= STM32_ADC_CR2_SWSTART;
    const uint32_t start = STM32_Micros();
    while ((STM32_ADC1->SR & STM32_ADC_SR_EOC) == 0U) {
        if ((uint32_t)(STM32_Micros() - start) > 200U) {
            return 0xFFFFU;
        }
    }
    return (uint16_t)(STM32_ADC1->DR & 0xFFFU);
}

uint16_t EasyTVC_ADC_VddaMv(uint16_t vref_raw)
{
    return (vref_raw > 0U && vref_raw < 4096U) ? (uint16_t)(3300UL * VREFINT_CAL / vref_raw) : 0U;
}

uint16_t EasyTVC_ADC_VrefintCal(void)
{
    return VREFINT_CAL;
}
