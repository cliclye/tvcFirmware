/*
 * Safe-blank image.
 *
 * Do not add peripheral setup here. At reset, STM32 GPIO is left in its
 * reset-state configuration. This image intentionally does not enable clocks
 * for GPIO or any peripheral, and it cannot drive an external net.
 */
int main(void)
{
    __asm volatile ("cpsid i" ::: "memory");

    for (;;) {
        __asm volatile ("wfi");
    }
}
