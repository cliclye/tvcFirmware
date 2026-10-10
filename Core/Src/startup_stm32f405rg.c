#include <stdint.h>

extern uint32_t _estack;
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

int main(void);

void Default_Handler(void) __attribute__((noreturn));
void Reset_Handler(void) __attribute__((noreturn, section(".text.Reset_Handler")));
void Fault_Handler(void) __attribute__((noreturn));
void SysTick_Handler(void);
void USART2_IRQHandler(void);

#define SCB_CPACR (*(volatile uint32_t *)0xE000ED88UL)
#define SCB_AIRCR (*(volatile uint32_t *)0xE000ED0CUL)

/* A fault or an unexpected interrupt in flight must not leave the vehicle with frozen
 * servos: reset, and let the boot sequence re-centre the outputs. */
void Fault_Handler(void)
{
    __asm volatile ("dsb" ::: "memory");
    SCB_AIRCR = 0x05FA0004UL;
    for (;;) {
    }
}

void Default_Handler(void)
{
    Fault_Handler();
}

void Reset_Handler(void)
{
    /* FPU on before any C code can touch a float register. */
    SCB_CPACR |= (0xFUL << 20);
    __asm volatile ("dsb\n isb" ::: "memory");

    uint32_t *source = &_sidata;
    uint32_t *destination = &_sdata;
    while (destination < &_edata) {
        *destination++ = *source++;
    }

    destination = &_sbss;
    while (destination < &_ebss) {
        *destination++ = 0U;
    }

    (void)main();
    Default_Handler();
}

/* Core exceptions (16) + 82 STM32F405 interrupts. Everything unhandled resets the MCU. */
#define EASYTVC_VECTOR_COUNT 98

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverride-init"
__attribute__((used, section(".isr_vector")))
void (* const easytvc_vector_table[EASYTVC_VECTOR_COUNT])(void) = {
    [0 ... EASYTVC_VECTOR_COUNT - 1] = Default_Handler,
    [0] = (void (*)(void))&_estack,
    [1] = Reset_Handler,
    [7] = 0, [8] = 0, [9] = 0, [10] = 0, [13] = 0,
    [15] = SysTick_Handler,
    [16 + 38] = USART2_IRQHandler,
};
#pragma GCC diagnostic pop
