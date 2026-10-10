#include "i2c_driver.h"
#include "stm32f4xx.h"
#include "board_pins.h"

/* 100 kHz standard mode from the 42 MHz APB1 clock. The BME280 pull-ups on this board are the
 * STM32's internal ~40 kOhm resistors, so 400 kHz is not an option. */
#define I2C_PCLK_MHZ    (STM32_APB1_HZ / 1000000UL)
#define I2C_CCR_VALUE   (STM32_APB1_HZ / (2UL * 100000UL))   /* 210 */
#define I2C_TRISE_VALUE (I2C_PCLK_MHZ + 1UL)                 /* 1000 ns max rise time */
#define I2C_WAIT_LIMIT  20000UL                              /* ~1 ms per phase at 168 MHz */

static void i2c_configure(STM32_I2C_TypeDef *i2c)
{
    i2c->CR1 = 0;
    i2c->CR2 = I2C_PCLK_MHZ;
    i2c->CCR = I2C_CCR_VALUE;
    i2c->TRISE = I2C_TRISE_VALUE;
    i2c->CR1 = STM32_I2C_CR1_PE | STM32_I2C_CR1_ACK;
}

static void i2c_reset(STM32_I2C_TypeDef *i2c)
{
    i2c->CR1 = STM32_I2C_CR1_SWRST;
    i2c->CR1 = 0;
    i2c_configure(i2c);
}

void EasyTVC_I2C1_Init(void)
{
    STM32_RCC->APB1RSTR |= (1U << 21);
    STM32_RCC->APB1RSTR &= ~(1U << 21);
    i2c_configure(STM32_I2C1);
}

void EasyTVC_I2C2_Init(void)
{
    STM32_RCC->APB1RSTR |= (1U << 22);
    STM32_RCC->APB1RSTR &= ~(1U << 22);
    i2c_configure(STM32_I2C2);
}

/* Release the bus after an error (NACK, arbitration loss, timeout). Without this a single
 * failed probe leaves SCL held low and every later transaction times out. */
static int i2c_abort(STM32_I2C_TypeDef *i2c)
{
    i2c->SR1 &= ~(STM32_I2C_SR1_AF | STM32_I2C_SR1_ARLO | STM32_I2C_SR1_BERR);
    i2c->CR1 |= STM32_I2C_CR1_STOP;
    uint32_t guard = I2C_WAIT_LIMIT;
    while ((i2c->SR2 & STM32_I2C_SR2_BUSY) && guard--) {
    }
    if (i2c->SR2 & STM32_I2C_SR2_BUSY) {
        i2c_reset(i2c);
    }
    i2c->CR1 = (i2c->CR1 & ~STM32_I2C_CR1_POS) | STM32_I2C_CR1_ACK;
    return -1;
}

static bool i2c_wait(STM32_I2C_TypeDef *i2c, uint32_t flag)
{
    uint32_t guard = I2C_WAIT_LIMIT;
    for (;;) {
        const uint32_t sr1 = i2c->SR1;
        if (sr1 & flag) {
            return true;
        }
        if ((sr1 & (STM32_I2C_SR1_AF | STM32_I2C_SR1_ARLO | STM32_I2C_SR1_BERR)) || guard-- == 0U) {
            return false;
        }
    }
}

/* START + address + register pointer. On success the bus is still ours (no STOP sent). */
static int i2c_begin(STM32_I2C_TypeDef *i2c, uint8_t addr, uint8_t reg)
{
    uint32_t guard = I2C_WAIT_LIMIT;
    while ((i2c->SR2 & STM32_I2C_SR2_BUSY) && guard--) {
    }
    if (i2c->SR2 & STM32_I2C_SR2_BUSY) {
        i2c_reset(i2c);
    }

    i2c->CR1 |= STM32_I2C_CR1_START;
    if (!i2c_wait(i2c, STM32_I2C_SR1_SB)) {
        return -1;
    }
    i2c->DR = (uint8_t)(addr << 1);
    if (!i2c_wait(i2c, STM32_I2C_SR1_ADDR)) {   /* returns early on NACK (wrong address) */
        return -1;
    }
    (void)i2c->SR2;                              /* clears ADDR */
    i2c->DR = reg;
    if (!i2c_wait(i2c, STM32_I2C_SR1_TXE)) {
        return -1;
    }
    return 0;
}

static int i2c_write(STM32_I2C_TypeDef *i2c, uint8_t addr, uint8_t reg, const uint8_t *data, size_t length)
{
    if (i2c_begin(i2c, addr, reg) != 0) {
        return i2c_abort(i2c);
    }
    for (size_t i = 0; i < length; i++) {
        i2c->DR = data[i];
        if (!i2c_wait(i2c, STM32_I2C_SR1_TXE)) {
            return i2c_abort(i2c);
        }
    }
    if (!i2c_wait(i2c, STM32_I2C_SR1_BTF)) {
        return i2c_abort(i2c);
    }
    i2c->CR1 |= STM32_I2C_CR1_STOP;
    return 0;
}

/* Master receiver, following RM0090 "Master receiver" for N = 1, 2 and >2 bytes. The ACK/STOP
 * timing differs for each; getting it wrong makes the slave send an extra byte or hold SDA. */
static int i2c_read(STM32_I2C_TypeDef *i2c, uint8_t addr, uint8_t reg, uint8_t *data, size_t length)
{
    if (length == 0U) {
        return 0;
    }
    if (i2c_begin(i2c, addr, reg) != 0 || !i2c_wait(i2c, STM32_I2C_SR1_BTF)) {
        return i2c_abort(i2c);
    }

    i2c->CR1 |= STM32_I2C_CR1_START;                       /* repeated start */
    if (!i2c_wait(i2c, STM32_I2C_SR1_SB)) {
        return i2c_abort(i2c);
    }
    i2c->DR = (uint8_t)((addr << 1) | 1U);

    if (length == 1U) {
        if (!i2c_wait(i2c, STM32_I2C_SR1_ADDR)) {
            return i2c_abort(i2c);
        }
        __asm volatile ("cpsid i" ::: "memory");
        i2c->CR1 &= ~STM32_I2C_CR1_ACK;                    /* NACK the only byte */
        (void)i2c->SR2;                                    /* clear ADDR */
        i2c->CR1 |= STM32_I2C_CR1_STOP;
        __asm volatile ("cpsie i" ::: "memory");
        if (!i2c_wait(i2c, STM32_I2C_SR1_RXNE)) {
            return i2c_abort(i2c);
        }
        data[0] = (uint8_t)i2c->DR;
    } else if (length == 2U) {
        i2c->CR1 |= STM32_I2C_CR1_POS | STM32_I2C_CR1_ACK;
        if (!i2c_wait(i2c, STM32_I2C_SR1_ADDR)) {
            return i2c_abort(i2c);
        }
        __asm volatile ("cpsid i" ::: "memory");
        (void)i2c->SR2;
        i2c->CR1 &= ~STM32_I2C_CR1_ACK;
        __asm volatile ("cpsie i" ::: "memory");
        if (!i2c_wait(i2c, STM32_I2C_SR1_BTF)) {
            return i2c_abort(i2c);
        }
        __asm volatile ("cpsid i" ::: "memory");
        i2c->CR1 |= STM32_I2C_CR1_STOP;
        data[0] = (uint8_t)i2c->DR;
        __asm volatile ("cpsie i" ::: "memory");
        data[1] = (uint8_t)i2c->DR;
    } else {
        i2c->CR1 |= STM32_I2C_CR1_ACK;
        if (!i2c_wait(i2c, STM32_I2C_SR1_ADDR)) {
            return i2c_abort(i2c);
        }
        (void)i2c->SR2;
        size_t i = 0;
        for (; length - i > 3U; ++i) {
            if (!i2c_wait(i2c, STM32_I2C_SR1_RXNE)) {
                return i2c_abort(i2c);
            }
            data[i] = (uint8_t)i2c->DR;
        }
        /* Last three bytes: N-2 is in DR, N-1 in the shift register. */
        if (!i2c_wait(i2c, STM32_I2C_SR1_BTF)) {
            return i2c_abort(i2c);
        }
        i2c->CR1 &= ~STM32_I2C_CR1_ACK;
        __asm volatile ("cpsid i" ::: "memory");
        data[i++] = (uint8_t)i2c->DR;
        i2c->CR1 |= STM32_I2C_CR1_STOP;
        data[i++] = (uint8_t)i2c->DR;
        __asm volatile ("cpsie i" ::: "memory");
        if (!i2c_wait(i2c, STM32_I2C_SR1_RXNE)) {
            return i2c_abort(i2c);
        }
        data[i] = (uint8_t)i2c->DR;
    }

    /* Wait for the STOP to leave the bus, then restore ACK/POS for the next transfer. */
    uint32_t guard = I2C_WAIT_LIMIT;
    while ((i2c->CR1 & STM32_I2C_CR1_STOP) && guard--) {
    }
    i2c->CR1 = (i2c->CR1 & ~STM32_I2C_CR1_POS) | STM32_I2C_CR1_ACK;
    return 0;
}

int EasyTVC_I2C1_Write(uint8_t addr, uint8_t reg, const uint8_t *data, size_t length)
{
    return i2c_write(STM32_I2C1, addr, reg, data, length);
}

int EasyTVC_I2C1_Read(uint8_t addr, uint8_t reg, uint8_t *data, size_t length)
{
    return i2c_read(STM32_I2C1, addr, reg, data, length);
}

int EasyTVC_I2C2_Write(uint8_t addr, uint8_t reg, const uint8_t *data, size_t length)
{
    return i2c_write(STM32_I2C2, addr, reg, data, length);
}

int EasyTVC_I2C2_Read(uint8_t addr, uint8_t reg, uint8_t *data, size_t length)
{
    return i2c_read(STM32_I2C2, addr, reg, data, length);
}
