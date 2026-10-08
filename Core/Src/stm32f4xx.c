#include "stm32f4xx.h"
#include "board_pins.h"

/* System clock configuration using HSI (16 MHz) */
#define STM32_TARGET_SYSCLK_HZ     168000000UL

/* Delay using busy loop - more reliable than SysTick for basic operation */
void STM32_DelayMicroseconds(uint32_t us)
{
    /* Approximate delay using busy loop for small delays */
    /* Each loop iteration is approximately 4 cycles at 168 MHz */
    /* Adjust for 168 MHz: cycles = us * 168 / 4 = us * 42 */
    uint32_t cycles = us * 42;
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

    /* ENABLE PLL FOR MAXIMUM PERFORMANCE (168 MHz) */
    /* Use HSI (16 MHz) as PLL source */
    
    /* Enable HSI */
    STM32_RCC->CR |= STM32_RCC_CR_HSION;
    while (!(STM32_RCC->CR & STM32_RCC_CR_HSIRDY));
    
    /* Configure PLL: HSI (16 MHz) / 16 * 336 / 2 = 168 MHz */
    STM32_RCC->PLLCFGR = (16 << 0) |   /* PLLM = 16 (16 MHz / 16 = 1 MHz) */
                         (336 << 6) |  /* PLLN = 336 (1 MHz * 336 = 336 MHz) */
                         (0 << 16) |   /* PLLP = 0 (PLL = 336 MHz / 2 = 168 MHz) */
                         STM32_RCC_PLLCFGR_PLLSRC_HSI;  /* HSI source */
    
    /* Enable PLL */
    STM32_RCC->CR |= STM32_RCC_CR_PLLON;
    while (!(STM32_RCC->CR & STM32_RCC_CR_PLLRDY));
    
    /* Configure flash latency for 168 MHz */
    STM32_FLASH->ACR = (5 << 0) |  /* 5 wait states for 168 MHz */
                       STM32_FLASH_ACR_PRFTEN |  /* Prefetch enable */
                       STM32_FLASH_ACR_ICEN |     /* Instruction cache enable */
                       STM32_FLASH_ACR_DCEN;      /* Data cache enable */
    
    /* Configure AHB, APB1, APB2 dividers */
    STM32_RCC->CFGR = (0 << 4) |   /* AHB = 1 (no division) */
                      (4 << 10) |  /* APB1 = 4 (168 MHz / 4 = 42 MHz) */
                      (2 << 13);   /* APB2 = 2 (168 MHz / 2 = 84 MHz) */
    
    /* Switch to PLL */
    STM32_RCC->CFGR |= (2 << 0);  /* PLL as system clock */
    while ((STM32_RCC->CFGR & (3 << 2)) != (2 << 2));  /* Wait for switch */
    
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
