#include "uart_driver.h"
#include "stm32f4xx.h"
#include "board_pins.h"

#include <string.h>

#define USART_SR_TC     (1U << 6)
#define USART_CR1_TXEIE (1U << 7)
#define USART_CR1_RXNEIE (1U << 5)
#define NVIC_ISER1      (*(volatile uint32_t *)0xE000E104UL)
#define USART2_IRQ      38U

/* 16x oversampling: BRR = fPCLK / baud (mantissa.fraction), 42 MHz / 115200 = 364.58 -> 365. */
#define USART_BRR(baud) ((uint32_t)((STM32_APB1_HZ + (baud) / 2U) / (baud)))

#define TX_RING_SIZE 512U   /* power of two */

static volatile uint8_t tx_ring[TX_RING_SIZE];
static volatile uint16_t tx_head;   /* written by the main loop */
static volatile uint16_t tx_tail;   /* written by the interrupt */

#define RX_RING_SIZE 256U   /* power of two: a few ground requests; at 115200 a byte comes every 87 us */
static volatile uint8_t rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head;   /* written by the interrupt */
static volatile uint16_t rx_tail;   /* written by the main loop */
static volatile uint32_t rx_overflow;

void EasyTVC_USART2_Init(void)
{
    STM32_USART_TypeDef *usart = STM32_USART2;

    STM32_RCC->APB1RSTR |= STM32_RCC_APB1ENR_USART2EN;
    STM32_RCC->APB1RSTR &= ~STM32_RCC_APB1ENR_USART2EN;

    usart->BRR = USART_BRR(EASYTVC_DEFAULT_UART_BAUD);
    usart->CR2 = 0;
    usart->CR3 = 0;
    tx_head = 0;
    tx_tail = 0;
    rx_head = 0;
    rx_tail = 0;
    rx_overflow = 0;
    usart->CR1 = STM32_USART_CR1_UE | STM32_USART_CR1_TE | STM32_USART_CR1_RE | USART_CR1_RXNEIE;
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
    /* Receive into a ring; reading DR also clears an overrun, which would otherwise re-enter forever. */
    const uint32_t sr = usart->SR;
    if (sr & (STM32_USART_SR_RXNE | (1U << 3))) {
        const uint8_t byte = (uint8_t)usart->DR;
        const uint16_t next = (uint16_t)((rx_head + 1U) & (RX_RING_SIZE - 1U));
        if (next != rx_tail) {
            rx_ring[rx_head] = byte;
            rx_head = next;
        } else {
            ++rx_overflow;   /* the main loop fell behind: the request is lost and the ground retries */
        }
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
    size_t received = 0;
    while (received < max_length && rx_tail != rx_head) {
        data[received++] = rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) & (RX_RING_SIZE - 1U));
    }
    return (int)received;
}

bool EasyTVC_USART2_QueueAll(const uint8_t *data, size_t length)
{
    if (length > EasyTVC_USART2_QueueFree()) {
        return false;
    }
    return EasyTVC_USART2_Queue(data, length) == length;
}

bool EasyTVC_USART2_TransmitComplete(void)
{
    return (STM32_USART2->SR & USART_SR_TC) != 0U;
}

void EasyTVC_USART2_QueueText(const char *text)
{
    (void)EasyTVC_USART2_Queue((const uint8_t *)text, strlen(text));
}
