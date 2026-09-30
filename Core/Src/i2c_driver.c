#include "i2c_driver.h"
#include "stm32f4xx.h"
#include "board_pins.h"

/* I2C timing for 100 kHz standard mode with 42 MHz APB1 clock */
#define I2C_CCR_FS   0
#define I2C_CCR_DUTY 0
#define I2C_CCR_VALUE  210  /* 42 MHz / (210 * 2) = 100 kHz */

void EasyTVC_I2C1_Init(void)
{
    STM32_I2C_TypeDef *i2c = STM32_I2C1;

    /* Reset I2C1 */
    STM32_RCC->APB1RSTR |= (1U << 21);
    STM32_RCC->APB1RSTR &= ~(1U << 21);

    /* Disable I2C */
    i2c->CR1 = 0;

    /* Configure I2C1: 100 kHz standard mode */
    i2c->CR2 = 42;  /* APB1 clock = 42 MHz */
    i2c->CCR = I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_VALUE;
    i2c->TRISE = 43;  /* Maximum rise time for 100 kHz */

    /* Enable I2C */
    i2c->CR1 = STM32_I2C_CR1_PE;
}

void EasyTVC_I2C2_Init(void)
{
    STM32_I2C_TypeDef *i2c = STM32_I2C2;

    /* Reset I2C2 */
    STM32_RCC->APB1RSTR |= (1U << 22);
    STM32_RCC->APB1RSTR &= ~(1U << 22);

    /* Disable I2C */
    i2c->CR1 = 0;

    /* Configure I2C2: 100 kHz standard mode */
    i2c->CR2 = 42;
    i2c->CCR = I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_VALUE;
    i2c->TRISE = 43;

    /* Enable I2C */
    i2c->CR1 = STM32_I2C_CR1_PE;
}

static int i2c_wait_for_flag(STM32_I2C_TypeDef *i2c, uint32_t flag, bool set)
{
    uint32_t timeout = 100000;
    while (timeout--) {
        bool flag_set = (i2c->SR1 & flag) != 0;
        if (flag_set == set) {
            return 0;
        }
    }
    return -1;  /* Timeout */
}

static int i2c_write(STM32_I2C_TypeDef *i2c, uint8_t addr, uint8_t reg, const uint8_t *data, size_t length)
{
    /* Send START */
    i2c->CR1 |= STM32_I2C_CR1_START;
    if (i2c_wait_for_flag(i2c, STM32_I2C_SR1_SB, true) != 0) {
        return -1;
    }

    /* Send address + write bit */
    i2c->DR = (addr << 1) | 0;
    if (i2c_wait_for_flag(i2c, STM32_I2C_SR1_ADDR, true) != 0) {
        return -1;
    }

    /* Clear ADDR flag */
    (void)i2c->SR2;

    /* Send register address */
    i2c->DR = reg;
    if (i2c_wait_for_flag(i2c, STM32_I2C_SR1_TXE, true) != 0) {
        return -1;
    }

    /* Send data */
    for (size_t i = 0; i < length; i++) {
        i2c->DR = data[i];
        if (i2c_wait_for_flag(i2c, STM32_I2C_SR1_TXE, true) != 0) {
            return -1;
        }
    }

    /* Wait for byte transfer complete */
    if (i2c_wait_for_flag(i2c, STM32_I2C_SR1_BTF, true) != 0) {
        return -1;
    }

    /* Send STOP */
    i2c->CR1 |= STM32_I2C_CR1_STOP;

    return 0;
}

static int i2c_read(STM32_I2C_TypeDef *i2c, uint8_t addr, uint8_t reg, uint8_t *data, size_t length)
{
    /* Send START for write */
    i2c->CR1 |= STM32_I2C_CR1_START;
    if (i2c_wait_for_flag(i2c, STM32_I2C_SR1_SB, true) != 0) {
        return -1;
    }

    /* Send address + write bit */
    i2c->DR = (addr << 1) | 0;
    if (i2c_wait_for_flag(i2c, STM32_I2C_SR1_ADDR, true) != 0) {
        return -1;
    }

    /* Clear ADDR flag */
    (void)i2c->SR2;

    /* Send register address */
    i2c->DR = reg;
    if (i2c_wait_for_flag(i2c, STM32_I2C_SR1_TXE, true) != 0) {
        return -1;
    }

    /* Wait for byte transfer complete */
    if (i2c_wait_for_flag(i2c, STM32_I2C_SR1_BTF, true) != 0) {
        return -1;
    }

    /* Send START for read */
    i2c->CR1 |= STM32_I2C_CR1_START;
    if (i2c_wait_for_flag(i2c, STM32_I2C_SR1_SB, true) != 0) {
        return -1;
    }

    /* Send address + read bit */
    i2c->DR = (addr << 1) | 1;
    if (i2c_wait_for_flag(i2c, STM32_I2C_SR1_ADDR, true) != 0) {
        return -1;
    }

    /* Clear ADDR flag */
    (void)i2c->SR2;

    /* Configure ACK/NACK */
    if (length == 1) {
        i2c->CR1 &= ~STM32_I2C_CR1_ACK;  /* NACK after single byte */
    } else {
        i2c->CR1 |= STM32_I2C_CR1_ACK;   /* ACK for multiple bytes */
    }

    /* Read data */
    for (size_t i = 0; i < length; i++) {
        if (i == length - 1) {
            /* Last byte: send STOP before reading */
            i2c->CR1 |= STM32_I2C_CR1_STOP;
        }

        if (i2c_wait_for_flag(i2c, STM32_I2C_SR1_RXNE, true) != 0) {
            return -1;
        }

        data[i] = (uint8_t)i2c->DR;
    }

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
