#include "stm32f4xx.h"
#include "board_pins.h"

/* System clock configuration using HSI (16 MHz) */
#define STM32_TARGET_SYSCLK_HZ     16000000UL

/* Delay using busy loop - more reliable than SysTick for basic operation */
void STM32_DelayMicroseconds(uint32_t us)
{
    /* Approximate delay using busy loop for small delays */
    /* Each loop iteration is approximately 4 cycles at 16 MHz */
    /* Adjust for 16 MHz: cycles = us * 16 / 4 = us * 4 */
    uint32_t cycles = us * 4;
    while (cycles--) {
        __asm volatile ("nop");
    }
}

void STM32_DelayMilliseconds(uint32_t ms)
{
    /* Use microsecond delay for millisecond delay */
    for (uint32_t i = 0; i < ms; i++) {
        STM32_DelayMicroseconds(1000);
    }
}

void STM32_SystemInit(void)
{
    /* Enable FPU */
    __asm volatile (
        "ldr r0, =0xE000ED88\n"
        "ldr r1, [r0]\n"
        "orr r1, r1, #(1 << 20)\n"
        "orr r1, r1, #(1 << 16)\n"
        "str r1, [r0]\n"
        "dsb\n"
        "isb\n"
        ::: "r0", "r1", "memory"
    );

    /* MINIMAL INITIALIZATION - Just enable clocks, no PLL, no clock switching */
    /* The board boots with HSI (16 MHz) by default, we use that directly */
    
    /* Enable GPIO clocks */
    STM32_RCC->AHB1ENR |= STM32_RCC_AHB1ENR_GPIOAEN |
                         STM32_RCC_AHB1ENR_GPIOBEN |
                         STM32_RCC_AHB1ENR_GPIOCEN;

    /* Enable peripheral clocks */
    STM32_RCC->APB1ENR |= STM32_RCC_APB1ENR_TIM3EN |
                         STM32_RCC_APB1ENR_TIM4EN |
                         STM32_RCC_APB1ENR_SPI2EN |
                         STM32_RCC_APB1ENR_SPI3EN |
                         STM32_RCC_APB1ENR_USART2EN |
                         STM32_RCC_APB1ENR_UART4EN |
                         STM32_RCC_APB1ENR_I2C1EN |
                         STM32_RCC_APB1ENR_I2C2EN;

    STM32_RCC->APB2ENR |= STM32_RCC_APB2ENR_SPI1EN |
                         STM32_RCC_APB2ENR_SYSCFGEN |
                         STM32_RCC_APB2ENR_ADC1EN;

    STM32_RCC->AHB2ENR |= STM32_RCC_AHB2ENR_OTGFSEN;
}
