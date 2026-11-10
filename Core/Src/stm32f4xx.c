#include "stm32f4xx.h"
#include "board_pins.h"
#include "gpio_init.h"

/* Cortex-M system registers */
#define SCB_CPACR   (*(volatile uint32_t *)0xE000ED88UL)
#define SCB_AIRCR   (*(volatile uint32_t *)0xE000ED0CUL)
#define SYST_CSR    (*(volatile uint32_t *)0xE000E010UL)
#define SYST_RVR    (*(volatile uint32_t *)0xE000E014UL)
#define SYST_CVR    (*(volatile uint32_t *)0xE000E018UL)

#define SYST_TICKS_PER_MS  (STM32_SYSCLK_HZ / 1000UL)
#define SYST_TICKS_PER_US  (STM32_SYSCLK_HZ / 1000000UL)

static volatile uint32_t system_millis;

void SysTick_Handler(void)
{
    system_millis++;
    EasyTVC_GPIO_LedTick(system_millis);
}

uint32_t STM32_Millis(void)
{
    return system_millis;
}

uint32_t STM32_Micros(void)
{
    uint32_t ms;
    uint32_t val;
    /* Re-read if the tick interrupt fired between the two reads. */
    do {
        ms = system_millis;
        val = SYST_CVR;
    } while (ms != system_millis);
    return ms * 1000UL + (SYST_TICKS_PER_MS - 1UL - val) / SYST_TICKS_PER_US;
}

void STM32_DelayMicroseconds(uint32_t us)
{
    const uint32_t start = STM32_Micros();
    while ((uint32_t)(STM32_Micros() - start) < us) {
        /* spin */
    }
}

/* Deliberate waits (sensor power-up, SD init, IMU retries) keep the watchdog fed: several of them
 * in a row used to exceed its ~170 ms worst-case timeout and reboot the board. A real hang still
 * trips it, because nothing spins here forever. */
void STM32_DelayMilliseconds(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; ++i) {
        STM32_DelayMicroseconds(1000UL);
        STM32_WatchdogFeed();
    }
}

void STM32_SystemReset(void)
{
    __asm volatile ("dsb" ::: "memory");
    SCB_AIRCR = 0x05FA0004UL;   /* VECTKEY | SYSRESETREQ */
    __asm volatile ("dsb" ::: "memory");
    for (;;) {
    }
}

bool STM32_ResetWasAbnormal(void)
{
    const uint32_t csr = STM32_RCC->CSR;
    const uint32_t por = 1UL << 27;
    const uint32_t abnormal = (1UL << 28) | (1UL << 29) | (1UL << 30) | (1UL << 31); /* SFT IWDG WWDG LPWR */
    STM32_RCC->CSR = csr | (1UL << 24);   /* RMVF: clear for the next boot */
    return (csr & por) == 0U && (csr & abnormal) != 0U;
}

void STM32_WatchdogStart(void)
{
    /* LSI is nominally 32 kHz (17..47 kHz over temperature): /16 -> ~2 kHz,
     * reload 500 -> ~250 ms nominal, >=170 ms worst case. Freeze it under a debugger. */
    STM32_DBGMCU_APB1_FZ |= (1UL << 12);
    STM32_IWDG->KR = 0x5555U;
    STM32_IWDG->PR = 2U;
    STM32_IWDG->RLR = 500U;
    uint32_t guard = 1000000UL;
    while (STM32_IWDG->SR != 0U && guard--) {
    }
    STM32_IWDG->KR = 0xAAAAU;
    STM32_IWDG->KR = 0xCCCCU;
}

void STM32_WatchdogFeed(void)
{
    STM32_IWDG->KR = 0xAAAAU;
}

/*
 * 168 MHz from the 16 MHz HSE the hwdef specifies (HSI as fallback if the crystal
 * does not start; both give 2 MHz PLL input):
 *   VCO = 2 MHz * 168 = 336 MHz, SYSCLK = VCO/2 = 168 MHz, USB = VCO/7 = 48 MHz.
 *   AHB /1, APB1 /4 (42 MHz max), APB2 /2 (84 MHz max).
 */
bool STM32_SystemInit(void)
{
    /* Enable CP10/CP11 (FPU) full access. */
    SCB_CPACR |= (0xFUL << 20);
    __asm volatile ("dsb\n isb" ::: "memory");

    /* Make sure we are running from HSI while the PLL is reconfigured. */
    STM32_RCC->CR |= STM32_RCC_CR_HSION;
    while (!(STM32_RCC->CR & STM32_RCC_CR_HSIRDY)) {
    }

    uint32_t pll_source = STM32_RCC_PLLCFGR_PLLSRC_HSI;
    STM32_RCC->CR |= STM32_RCC_CR_HSEON;
    for (uint32_t guard = 200000UL; guard; --guard) {
        if (STM32_RCC->CR & STM32_RCC_CR_HSERDY) {
            pll_source = STM32_RCC_PLLCFGR_PLLSRC_HSE;
            break;
        }
    }
    if (pll_source != STM32_RCC_PLLCFGR_PLLSRC_HSE) {
        STM32_RCC->CR &= ~STM32_RCC_CR_HSEON;
    }

    /* Flash wait states must be raised BEFORE the clock goes up. 5 WS for 150-168 MHz at 3.3 V. */
    STM32_FLASH->ACR = 5U | STM32_FLASH_ACR_PRFTEN | STM32_FLASH_ACR_ICEN | STM32_FLASH_ACR_DCEN;
    if ((STM32_FLASH->ACR & STM32_FLASH_ACR_LATENCY) != 5U) {
        return false;
    }

    STM32_RCC->CFGR = STM32_RCC_CFGR_HPRE_DIV1 | STM32_RCC_CFGR_PPRE1_DIV4 | STM32_RCC_CFGR_PPRE2_DIV2;

    STM32_RCC->PLLCFGR = (8UL << STM32_RCC_PLLCFGR_PLLM_SHIFT) |
                         (168UL << STM32_RCC_PLLCFGR_PLLN_SHIFT) |
                         (0UL << STM32_RCC_PLLCFGR_PLLP_SHIFT) |   /* /2 */
                         (7UL << STM32_RCC_PLLCFGR_PLLQ_SHIFT) |   /* 336/7 = 48 MHz */
                         pll_source;

    STM32_RCC->CR |= STM32_RCC_CR_PLLON;
    bool pll_ready = false;
    for (uint32_t guard = 500000UL; guard; --guard) {
        if (STM32_RCC->CR & STM32_RCC_CR_PLLRDY) {
            pll_ready = true;
            break;
        }
    }
    if (!pll_ready) {
        STM32_RCC->CR &= ~STM32_RCC_CR_PLLON;
        STM32_RCC->CFGR = 0U;   /* stay on HSI, no dividers */
        return false;
    }

    STM32_RCC->CFGR = (STM32_RCC->CFGR & ~STM32_RCC_CFGR_SW_MASK) | STM32_RCC_CFGR_SW_PLL;
    for (uint32_t guard = 500000UL; guard; --guard) {
        if ((STM32_RCC->CFGR & STM32_RCC_CFGR_SWS_MASK) == STM32_RCC_CFGR_SWS_PLL) {
            break;
        }
        if (guard == 1U) {
            STM32_RCC->CFGR = 0U;
            return false;
        }
    }

    /* Bus clocks. GPIOA/B/C only; nothing is enabled that the image does not drive. */
    STM32_RCC->AHB1ENR |= STM32_RCC_AHB1ENR_GPIOAEN |
                          STM32_RCC_AHB1ENR_GPIOBEN |
                          STM32_RCC_AHB1ENR_GPIOCEN;

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

    /* 1 kHz time base. */
    SYST_RVR = SYST_TICKS_PER_MS - 1UL;
    SYST_CVR = 0U;
    SYST_CSR = 7U;   /* core clock, interrupt, enable */
    return true;
}
