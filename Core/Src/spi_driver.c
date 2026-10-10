#include "spi_driver.h"
#include "stm32f4xx.h"
#include "board_pins.h"

void EasyTVC_SPI1_Init(void)
{
    STM32_SPI_TypeDef *spi = STM32_SPI1;

    /* Reset SPI1 */
    STM32_RCC->APB2RSTR |= (1U << 12);
    STM32_RCC->APB2RSTR &= ~(1U << 12);

    /* Configure SPI1: Master, Mode 3 (CPOL=1, CPHA=1).
     * APB2 = 84 MHz. /8 would be 10.5 MHz, just over the BMI088's 10 MHz limit; /16 = 5.25 MHz
     * is plenty for 200 Hz reads of a dozen bytes. */
    spi->CR1 = STM32_SPI_CR1_MSTR |
               STM32_SPI_CR1_CPOL_HIGH |
               STM32_SPI_CR1_CPHA_2EDGE |
               STM32_SPI_CR1_BR_DIV16 |
               STM32_SPI_CR1_SSM |
               STM32_SPI_CR1_SSI;

    /* 8-bit data frame, MSB first */
    spi->CR2 = 0;

    /* Enable SPI1 */
    spi->CR1 |= STM32_SPI_CR1_SPE;
}

void EasyTVC_SPI2_Init(void)
{
    STM32_SPI_TypeDef *spi = STM32_SPI2;

    /* Reset SPI2 */
    STM32_RCC->APB1RSTR |= (1U << 14);
    STM32_RCC->APB1RSTR &= ~(1U << 14);

    /* Configure SPI2: Master, Mode 3. APB1 = 42 MHz, so /2 = 21 MHz is the fastest this bus can
     * go (the hwdef's 32 MHz is not reachable from APB1). The NOR flash is not used yet. */
    spi->CR1 = STM32_SPI_CR1_MSTR |
               STM32_SPI_CR1_CPOL_HIGH |
               STM32_SPI_CR1_CPHA_2EDGE |
               STM32_SPI_CR1_BR_DIV2 |
               STM32_SPI_CR1_SSM |
               STM32_SPI_CR1_SSI;

    spi->CR2 = 0;
    spi->CR1 |= STM32_SPI_CR1_SPE;
}

void EasyTVC_SPI3_Init(void)
{
    STM32_SPI_TypeDef *spi = STM32_SPI3;

    /* Reset SPI3 */
    STM32_RCC->APB1RSTR |= (1U << 15);
    STM32_RCC->APB1RSTR &= ~(1U << 15);

    /* Configure SPI3 for the microSD card: Master, Mode 0, start SLOW.
     * SD cards must be initialised at 100-400 kHz. 42 MHz / 128 = 328 kHz.
     * Call EasyTVC_SPI3_SetFast(true) once SD_Raw_Init has finished. */
    spi->CR1 = STM32_SPI_CR1_MSTR |
               STM32_SPI_CR1_BR_DIV128 |
               STM32_SPI_CR1_SSM |
               STM32_SPI_CR1_SSI;

    spi->CR2 = 0;
    spi->CR1 |= STM32_SPI_CR1_SPE;
}

void EasyTVC_SPI3_SetFast(bool fast)
{
    STM32_SPI_TypeDef *spi = STM32_SPI3;
    uint32_t guard = 100000U;
    while ((spi->SR & STM32_SPI_SR_BSY) && guard--) {
    }
    /* BR[2:0] may only be changed while the peripheral is disabled. */
    spi->CR1 &= ~STM32_SPI_CR1_SPE;
    spi->CR1 = (spi->CR1 & ~(7U << 3)) | (fast ? STM32_SPI_CR1_BR_DIV4 : STM32_SPI_CR1_BR_DIV128);
    spi->CR1 |= STM32_SPI_CR1_SPE;   /* 42/4 = 10.5 MHz when fast */
}

#define SPI_WAIT_LIMIT 200000U

static int spi_wait(STM32_SPI_TypeDef *spi, uint32_t mask, bool set)
{
    uint32_t guard = SPI_WAIT_LIMIT;
    while (((spi->SR & mask) != 0U) != set) {
        if (guard-- == 0U) {
            return -1;
        }
    }
    return 0;
}

static int spi_transfer(STM32_SPI_TypeDef *spi, uint8_t *tx_data, uint8_t *rx_data, size_t length)
{
    for (size_t i = 0; i < length; i++) {
        if (spi_wait(spi, STM32_SPI_SR_TXE, true) != 0) {
            return -1;
        }
        spi->DR = (tx_data != NULL) ? tx_data[i] : 0xFFU;

        if (spi_wait(spi, STM32_SPI_SR_RXNE, true) != 0) {
            return -1;
        }
        const uint8_t byte = (uint8_t)spi->DR;   /* also clears RXNE */
        if (rx_data != NULL) {
            rx_data[i] = byte;
        }
    }

    return spi_wait(spi, STM32_SPI_SR_BSY, false);
}

int EasyTVC_SPI1_Transfer(uint8_t *tx_data, uint8_t *rx_data, size_t length)
{
    return spi_transfer(STM32_SPI1, tx_data, rx_data, length);
}

int EasyTVC_SPI2_Transfer(uint8_t *tx_data, uint8_t *rx_data, size_t length)
{
    return spi_transfer(STM32_SPI2, tx_data, rx_data, length);
}

int EasyTVC_SPI3_Transfer(uint8_t *tx_data, uint8_t *rx_data, size_t length)
{
    return spi_transfer(STM32_SPI3, tx_data, rx_data, length);
}

void EasyTVC_SPI1_CS_Accel_Set(bool state)
{
    STM32_GPIO_TypeDef *gpio = STM32_GPIOC;
    if (state) {
        gpio->BSRR = (1U << 8);  /* PC8 high (inactive) */
    } else {
        gpio->BSRR = (1U << 8) << 16;  /* PC8 low (active) */
    }
}

void EasyTVC_SPI1_CS_Gyro_Set(bool state)
{
    STM32_GPIO_TypeDef *gpio = STM32_GPIOC;
    if (state) {
        gpio->BSRR = (1U << 7);  /* PC7 high (inactive) */
    } else {
        gpio->BSRR = (1U << 7) << 16;  /* PC7 low (active) */
    }
}

void EasyTVC_SPI2_CS_Flash_Set(bool state)
{
    STM32_GPIO_TypeDef *gpio = STM32_GPIOC;
    if (state) {
        gpio->BSRR = (1U << 5);  /* PC5 high (inactive) */
    } else {
        gpio->BSRR = (1U << 5) << 16;  /* PC5 low (active) */
    }
}

void EasyTVC_SPI3_CS_SD_Set(bool state)
{
    STM32_GPIO_TypeDef *gpio = STM32_GPIOC;
    if (state) {
        gpio->BSRR = (1U << 6);  /* PC6 high (inactive) */
    } else {
        gpio->BSRR = (1U << 6) << 16;  /* PC6 low (active) */
    }
}

static uint8_t spi_transfer_byte(STM32_SPI_TypeDef *spi, uint8_t data)
{
    uint8_t rx = 0xFFU;   /* a dead bus reads as idle-high */
    if (spi_wait(spi, STM32_SPI_SR_TXE, true) != 0) {
        return rx;
    }
    spi->DR = data;
    if (spi_wait(spi, STM32_SPI_SR_RXNE, true) != 0) {
        return rx;
    }
    rx = (uint8_t)spi->DR;
    (void)spi_wait(spi, STM32_SPI_SR_BSY, false);
    return rx;
}

uint8_t EasyTVC_SPI3_TransferByte(uint8_t data)
{
    return spi_transfer_byte(STM32_SPI3, data);
}

uint8_t EasyTVC_SPI2_TransferByte(uint8_t data)
{
    return spi_transfer_byte(STM32_SPI2, data);
}
