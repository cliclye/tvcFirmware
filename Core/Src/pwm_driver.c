#include "pwm_driver.h"
#include "stm32f4xx.h"
#include "board_pins.h"

/* Servo PWM configuration: 50 Hz (20ms period) */
#define PWM_FREQ_HZ          50U
#define PWM_PERIOD_US        20000U
#define PWM_MAX_PULSE_US    2000U
#define PWM_MIN_PULSE_US    1000U

/* Timer clock calculation: APB1 = 42 MHz, timer clock = 84 MHz (since timers on APB1 get x2) */
#define TIMER_CLOCK_HZ       84000000UL
#define TIMER_PRESCALER      (TIMER_CLOCK_HZ / 1000000UL) - 1  /* 1 MHz timer clock */
#define TIMER_PERIOD         (PWM_PERIOD_US - 1)  /* 19999 counts for 20ms */

void EasyTVC_PWM_Init(void)
{
    STM32_TIM_TypeDef *tim3 = STM32_TIM3;
    STM32_TIM_TypeDef *tim4 = STM32_TIM4;

    /* Reset TIM3 */
    STM32_RCC->APB1RSTR |= STM32_RCC_APB1ENR_TIM3EN;
    STM32_RCC->APB1RSTR &= ~STM32_RCC_APB1ENR_TIM3EN;

    /* Reset TIM4 */
    STM32_RCC->APB1RSTR |= STM32_RCC_APB1ENR_TIM4EN;
    STM32_RCC->APB1RSTR &= ~STM32_RCC_APB1ENR_TIM4EN;

    /* Configure TIM3 for servo PWM */
    tim3->CR1 = 0;
    tim3->PSC = TIMER_PRESCALER;
    tim3->ARR = TIMER_PERIOD;
    tim3->CR1 |= STM32_TIM_CR1_ARPE;  /* Auto-reload preload enable */

    /* Configure TIM3 channels 3 and 4 for PWM */
    tim3->CCMR2 = (STM32_TIM_CCMR2_OC3M_PWM1 << 4) |  /* OC3 PWM mode 1 */
                  (STM32_TIM_CCMR2_OC4M_PWM1 << 12); /* OC4 PWM mode 1 */
    tim3->CCMR2 |= (1U << 3) | (1U << 11);  /* Output compare preload enable */

    /* Enable TIM3 channels 3 and 4 */
    tim3->CCER = STM32_TIM_CCER_CC3E | STM32_TIM_CCER_CC4E;

    /* Configure TIM4 for servo PWM */
    tim4->CR1 = 0;
    tim4->PSC = TIMER_PRESCALER;
    tim4->ARR = TIMER_PERIOD;
    tim4->CR1 |= STM32_TIM_CR1_ARPE;

    /* Configure TIM4 channels 3 and 4 for PWM */
    tim4->CCMR2 = (STM32_TIM_CCMR2_OC3M_PWM1 << 4) |
                  (STM32_TIM_CCMR2_OC4M_PWM1 << 12);
    tim4->CCMR2 |= (1U << 3) | (1U << 11);

    /* Enable TIM4 channels 3 and 4 */
    tim4->CCER = STM32_TIM_CCER_CC3E | STM32_TIM_CCER_CC4E;

    /* Set initial pulse widths to center (1500 us) */
    tim3->CCR[2] = 1500;  /* TIM3_CH3 */
    tim3->CCR[3] = 1500;  /* TIM3_CH4 */
    tim4->CCR[2] = 1500;  /* TIM4_CH3 */
    tim4->CCR[3] = 1500;  /* TIM4_CH4 */

    /* Enable TIM3 and TIM4 */
    tim3->CR1 |= STM32_TIM_CR1_CEN;
    tim4->CR1 |= STM32_TIM_CR1_CEN;
}

void EasyTVC_PWM_SetChannel(uint8_t channel, uint16_t pulse_us)
{
    STM32_TIM_TypeDef *tim;
    uint8_t ch;

    /* Clamp pulse width to valid range */
    if (pulse_us < PWM_MIN_PULSE_US) {
        pulse_us = PWM_MIN_PULSE_US;
    }
    if (pulse_us > PWM_MAX_PULSE_US) {
        pulse_us = PWM_MAX_PULSE_US;
    }

    switch (channel) {
        case 1:  /* PB0, TIM3_CH3 */
            tim = STM32_TIM3;
            ch = 2;
            break;
        case 2:  /* PB1, TIM3_CH4 */
            tim = STM32_TIM3;
            ch = 3;
            break;
        case 3:  /* PB8, TIM4_CH3 */
            tim = STM32_TIM4;
            ch = 2;
            break;
        case 4:  /* PB9, TIM4_CH4 */
            tim = STM32_TIM4;
            ch = 3;
            break;
        default:
            return;
    }

    tim->CCR[ch] = pulse_us;
}
