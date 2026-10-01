#include "spi_driver.h"
#include "stm32f4xx.h"
#include "board_pins.h"

void EasyTVC_SPI1_Init(void)
{
    STM32_SPI_TypeDef *spi = STM32_SPI1;

    /* Reset SPI1 */
    STM32_RCC->APB2RSTR |= (1U << 12);
    STM32_RCC->APB2RSTR &= ~(1U << 12);

    /* Configure SPI1: Master, Mode 3 (CPOL=1, CPHA=1), 10 MHz */
    /* APB2 clock = 84 MHz, need 10 MHz => prescaler = 8 */
    spi->CR1 = STM32_SPI_CR1_MSTR |
               STM32_SPI_CR1_CPOL_HIGH |
               STM32_SPI_CR1_CPHA_2EDGE |
               STM32_SPI_CR1_BR_DIV8 |
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

    /* Configure SPI2: Master, Mode 3, 32 MHz */
    /* APB1 clock = 42 MHz, need 32 MHz => prescaler = 2 */
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

    /* Configure SPI3: Master, Mode 3, for SD card */
    /* APB1 clock = 42 MHz, use 21 MHz => prescaler = 4 */
    spi->CR1 = STM32_SPI_CR1_MSTR |
               STM32_SPI_CR1_CPOL_HIGH |
               STM32_SPI_CR1_CPHA_2EDGE |
               STM32_SPI_CR1_BR_DIV4 |
               STM32_SPI_CR1_SSM |
               STM32_SPI_CR1_SSI;

    spi->CR2 = 0;
    spi->CR1 |= STM32_SPI_CR1_SPE;
}

static int spi_transfer(STM32_SPI_TypeDef *spi, uint8_t *tx_data, uint8_t *rx_data, size_t length)
{
    for (size_t i = 0; i < length; i++) {
        /* Wait for TX buffer empty */
        while (!(spi->SR & STM32_SPI_SR_TXE)) {
            /* Wait */
        }

        /* Write data */
        spi->DR = (tx_data != NULL) ? tx_data[i] : 0xFFU;

        /* Wait for RX buffer not empty */
        while (!(spi->SR & STM32_SPI_SR_RXNE)) {
            /* Wait */
        }

        /* Read data */
        if (rx_data != NULL) {
            rx_data[i] = (uint8_t)(spi->DR);
        } else {
            (void)spi->DR;  /* Clear RXNE */
        }
    }

    /* Wait for SPI not busy */
    while (spi->SR & STM32_SPI_SR_BSY) {
        /* Wait */
    }

    return 0;
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

/* Single-byte transfer for SD card */
uint8_t EasyTVC_SPI3_TransferByte(uint8_t data)
{
    STM32_SPI_TypeDef *spi = STM32_SPI3;
    uint8_t rx_data;
    
    /* Wait for TX buffer empty */
    while (!(spi->SR & STM32_SPI_SR_TXE)) {
        /* Wait */
    }
    
    /* Write data */
    spi->DR = data;
    
    /* Wait for RX buffer not empty */
    while (!(spi->SR & STM32_SPI_SR_RXNE)) {
        /* Wait */
    }
    
    /* Read data */
    rx_data = (uint8_t)(spi->DR);
    
    /* Wait for SPI not busy */
    while (spi->SR & STM32_SPI_SR_BSY) {
        /* Wait */
    }
    
    return rx_data;
}

/* Single-byte transfer for flash card */
uint8_t EasyTVC_SPI2_TransferByte(uint8_t data)
{
    STM32_SPI_TypeDef *spi = STM32_SPI2;
    uint8_t rx_data;
    
    /* Wait for TX buffer empty */
    while (!(spi->SR & STM32_SPI_SR_TXE)) {
        /* Wait */
    }
    
    /* Write data */
    spi->DR = data;
    
    /* Wait for RX buffer not empty */
    while (!(spi->SR & STM32_SPI_SR_RXNE)) {
        /* Wait */
    }
    
    /* Read data */
    rx_data = (uint8_t)(spi->DR);
    
    /* Wait for SPI not busy */
    while (spi->SR & STM32_SPI_SR_BSY) {
        /* Wait */
    }
    
    return rx_data;
}
