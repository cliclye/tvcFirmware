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

void Default_Handler(void)
{
    for (;;) {
        __asm volatile ("wfi");
    }
}

void Reset_Handler(void)
{
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

__attribute__((used, section(".isr_vector")))
void (* const easytvc_vector_table[])(void) = {
    (void (*)(void))&_estack,
    Reset_Handler,
    Default_Handler, /* NMI */
    Default_Handler, /* HardFault */
    Default_Handler, /* MemManage */
    Default_Handler, /* BusFault */
    Default_Handler, /* UsageFault */
    0,
    0,
    0,
    0,
    Default_Handler, /* SVCall */
    Default_Handler, /* Debug monitor */
    0,
    Default_Handler, /* PendSV */
    Default_Handler, /* SysTick */
};
