#include "pwm_driver.h"
#include "stm32f4xx.h"
#include "board_pins.h"
#include "vehicle_config.h"

/* Servo PWM: EASYTVC_SERVO_RATE_HZ (50 Hz default), 1 us resolution. */
#if EASYTVC_SERVO_RATE_HZ < 50 || EASYTVC_SERVO_RATE_HZ > 333 || (1000000 % EASYTVC_SERVO_RATE_HZ) != 0
#error "EASYTVC_SERVO_RATE_HZ must be 50..333 and divide 1 MHz evenly"
#endif
#define PWM_PERIOD_US       (1000000U / EASYTVC_SERVO_RATE_HZ)
#define PWM_MAX_PULSE_US    2000U
#define PWM_MIN_PULSE_US    1000U
#define PWM_CENTER_US       1500U

/* TIM3/TIM4 sit on APB1, which is divided by 4, so their kernel clock is 2 x 42 = 84 MHz. */
#define TIMER_PRESCALER     ((STM32_TIM_APB1_HZ / 1000000UL) - 1UL)   /* 1 MHz tick */
#define TIMER_PERIOD        (PWM_PERIOD_US - 1U)

#define CCMR2_OC3PE         (1U << 3)
#define CCMR2_OC4PE         (1U << 11)
#define TIM_EGR_UG          (1U << 0)

static void pwm_timer_init(STM32_TIM_TypeDef *tim)
{
    tim->CR1 = 0;
    tim->PSC = TIMER_PRESCALER;
    tim->ARR = TIMER_PERIOD;

    /* The OCxM macros already carry their bit position; do not shift them again. */
    tim->CCMR1 = 0;
    tim->CCMR2 = STM32_TIM_CCMR2_OC3M_PWM1 | STM32_TIM_CCMR2_OC4M_PWM1 | CCMR2_OC3PE | CCMR2_OC4PE;
    tim->CCER = STM32_TIM_CCER_CC3E | STM32_TIM_CCER_CC4E;

    tim->CCR[2] = PWM_CENTER_US;
    tim->CCR[3] = PWM_CENTER_US;

    tim->CR1 = STM32_TIM_CR1_ARPE;
    tim->EGR = TIM_EGR_UG;          /* load the preloaded ARR/CCR before the counter starts */
    tim->CR1 |= STM32_TIM_CR1_CEN;
}

void EasyTVC_PWM_Init(void)
{
    STM32_RCC->APB1RSTR |= STM32_RCC_APB1ENR_TIM3EN | STM32_RCC_APB1ENR_TIM4EN;
    STM32_RCC->APB1RSTR &= ~(STM32_RCC_APB1ENR_TIM3EN | STM32_RCC_APB1ENR_TIM4EN);

    pwm_timer_init(STM32_TIM3);
    pwm_timer_init(STM32_TIM4);
}

void EasyTVC_PWM_SetChannel(uint8_t channel, uint16_t pulse_us)
{
    STM32_TIM_TypeDef *tim;
    uint8_t ch;

    if (pulse_us < PWM_MIN_PULSE_US) {
        pulse_us = PWM_MIN_PULSE_US;
    }
    if (pulse_us > PWM_MAX_PULSE_US) {
        pulse_us = PWM_MAX_PULSE_US;
    }

    switch (channel) {
        case 1:  tim = STM32_TIM3; ch = 2; break;   /* PB0, TIM3_CH3 */
        case 2:  tim = STM32_TIM3; ch = 3; break;   /* PB1, TIM3_CH4 */
        case 3:  tim = STM32_TIM4; ch = 2; break;   /* PB8, TIM4_CH3 */
        case 4:  tim = STM32_TIM4; ch = 3; break;   /* PB9, TIM4_CH4 */
        default: return;
    }

    tim->CCR[ch] = pulse_us;
}
