#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

void EasyTVC_USART2_Init(void);
void EasyTVC_UART4_Init(void);

/* Blocking send (bring-up only: it stalls the caller for the whole message). */
int EasyTVC_USART2_Send(const uint8_t *data, size_t length);

/* Non-blocking send: copies into a ring drained by the USART2 interrupt. Returns the number of
 * bytes accepted; the rest are dropped so telemetry can never stall the control loop. */
size_t EasyTVC_USART2_Queue(const uint8_t *data, size_t length);
size_t EasyTVC_USART2_QueueFree(void);

int EasyTVC_USART2_Receive(uint8_t *data, size_t max_length);
bool EasyTVC_USART2_TransmitComplete(void);
