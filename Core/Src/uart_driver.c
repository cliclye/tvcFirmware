#include "uart_driver.h"
#include "stm32f4xx.h"
#include "board_pins.h"

void EasyTVC_USART2_Init(void)
{
    STM32_USART_TypeDef *usart = STM32_USART2;

    /* Reset USART2 */
    STM32_RCC->APB1RSTR |= STM32_RCC_APB1ENR_USART2EN;
    STM32_RCC->APB1RSTR &= ~STM32_RCC_APB1ENR_USART2EN;

    /* Configure USART2: 115200 baud, 8N1 */
    /* APB1 clock = 42 MHz */
    /* USARTDIV = 42e6 / 115200 = 364.58 */
    /* BRR = (364 << 4) | 9 = 5833 */
    usart->BRR = 5833;
    usart->CR1 = STM32_USART_CR1_UE | STM32_USART_CR1_TE | STM32_USART_CR1_RE;
    usart->CR2 = 0;
    usart->CR3 = 0;
}

void EasyTVC_UART4_Init(void)
{
    STM32_USART_TypeDef *uart = STM32_UART4;

    /* Reset UART4 */
    STM32_RCC->APB1RSTR |= STM32_RCC_APB1ENR_UART4EN;
    STM32_RCC->APB1RSTR &= ~STM32_RCC_APB1ENR_UART4EN;

    /* Configure UART4: 115200 baud, 8N1 */
    uart->BRR = 5833;
    uart->CR1 = STM32_USART_CR1_UE | STM32_USART_CR1_TE | STM32_USART_CR1_RE;
    uart->CR2 = 0;
    uart->CR3 = 0;
}

int EasyTVC_USART2_Send(const uint8_t *data, size_t length)
{
    STM32_USART_TypeDef *usart = STM32_USART2;

    for (size_t i = 0; i < length; i++) {
        /* Wait for TX buffer empty */
        while (!(usart->SR & STM32_USART_SR_TXE)) {
            /* Wait */
        }
        usart->DR = data[i];
    }

    /* Wait for transmission complete */
    while (!(usart->SR & (1U << 6))) {  /* TC bit */
        /* Wait */
    }

    return 0;
}

int EasyTVC_USART2_Receive(uint8_t *data, size_t max_length)
{
    STM32_USART_TypeDef *usart = STM32_USART2;
    size_t received = 0;

    while (received < max_length) {
        if (usart->SR & STM32_USART_SR_RXNE) {
            data[received++] = (uint8_t)usart->DR;
        } else {
            break;  /* No more data available */
        }
    }

    return received;
}

bool EasyTVC_USART2_TransmitComplete(void)
{
    STM32_USART_TypeDef *usart = STM32_USART2;
    return (usart->SR & (1U << 6)) != 0;  /* TC bit */
}
