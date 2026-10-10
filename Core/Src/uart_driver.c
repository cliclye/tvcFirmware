#include "uart_driver.h"
#include "stm32f4xx.h"
#include "board_pins.h"

#define USART_SR_TC     (1U << 6)
#define USART_CR1_TXEIE (1U << 7)
#define NVIC_ISER1      (*(volatile uint32_t *)0xE000E104UL)
#define USART2_IRQ      38U

/* 16x oversampling: BRR = fPCLK / baud (mantissa.fraction), 42 MHz / 115200 = 364.58 -> 365. */
#define USART_BRR(baud) ((uint32_t)((STM32_APB1_HZ + (baud) / 2U) / (baud)))

#define TX_RING_SIZE 512U   /* power of two */

static volatile uint8_t tx_ring[TX_RING_SIZE];
static volatile uint16_t tx_head;   /* written by the main loop */
static volatile uint16_t tx_tail;   /* written by the interrupt */

void EasyTVC_USART2_Init(void)
{
    STM32_USART_TypeDef *usart = STM32_USART2;

    STM32_RCC->APB1RSTR |= STM32_RCC_APB1ENR_USART2EN;
    STM32_RCC->APB1RSTR &= ~STM32_RCC_APB1ENR_USART2EN;

    usart->BRR = USART_BRR(EASYTVC_DEFAULT_UART_BAUD);
    usart->CR2 = 0;
    usart->CR3 = 0;
    usart->CR1 = STM32_USART_CR1_UE | STM32_USART_CR1_TE | STM32_USART_CR1_RE;

    tx_head = 0;
    tx_tail = 0;
    NVIC_ISER1 = 1UL << (USART2_IRQ - 32U);
}

void EasyTVC_UART4_Init(void)
{
    STM32_USART_TypeDef *uart = STM32_UART4;

    STM32_RCC->APB1RSTR |= STM32_RCC_APB1ENR_UART4EN;
    STM32_RCC->APB1RSTR &= ~STM32_RCC_APB1ENR_UART4EN;

    uart->BRR = USART_BRR(EASYTVC_DEFAULT_UART_BAUD);
    uart->CR2 = 0;
    uart->CR3 = 0;
    uart->CR1 = STM32_USART_CR1_UE | STM32_USART_CR1_TE | STM32_USART_CR1_RE;
}

void USART2_IRQHandler(void)
{
    STM32_USART_TypeDef *usart = STM32_USART2;
    if ((usart->CR1 & USART_CR1_TXEIE) && (usart->SR & STM32_USART_SR_TXE)) {
        if (tx_tail != tx_head) {
            usart->DR = tx_ring[tx_tail];
            tx_tail = (uint16_t)((tx_tail + 1U) & (TX_RING_SIZE - 1U));
        } else {
            usart->CR1 &= ~USART_CR1_TXEIE;
        }
    }
    /* Receive is polled; an overrun flag would otherwise re-enter forever, so drain it. */
    if (usart->SR & (1U << 3)) {
        (void)usart->DR;
    }
}

size_t EasyTVC_USART2_QueueFree(void)
{
    return (size_t)((tx_tail - tx_head - 1U) & (TX_RING_SIZE - 1U));
}

size_t EasyTVC_USART2_Queue(const uint8_t *data, size_t length)
{
    size_t room = EasyTVC_USART2_QueueFree();
    if (length > room) {
        length = room;
    }
    uint16_t head = tx_head;
    for (size_t i = 0; i < length; i++) {
        tx_ring[head] = data[i];
        head = (uint16_t)((head + 1U) & (TX_RING_SIZE - 1U));
    }
    tx_head = head;
    if (length != 0U) {
        STM32_USART2->CR1 |= USART_CR1_TXEIE;
    }
    return length;
}

int EasyTVC_USART2_Send(const uint8_t *data, size_t length)
{
    STM32_USART_TypeDef *usart = STM32_USART2;

    for (size_t i = 0; i < length; i++) {
        uint32_t guard = 2000000U;
        while (!(usart->SR & STM32_USART_SR_TXE)) {
            if (guard-- == 0U) {
                return -1;
            }
        }
        usart->DR = data[i];
    }
    return 0;
}

int EasyTVC_USART2_Receive(uint8_t *data, size_t max_length)
{
    STM32_USART_TypeDef *usart = STM32_USART2;
    size_t received = 0;

    while (received < max_length && (usart->SR & STM32_USART_SR_RXNE)) {
        data[received++] = (uint8_t)usart->DR;
    }
    return (int)received;
}

bool EasyTVC_USART2_TransmitComplete(void)
{
    return (STM32_USART2->SR & USART_SR_TC) != 0U;
}
