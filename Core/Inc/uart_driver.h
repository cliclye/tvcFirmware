#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

void EasyTVC_USART2_Init(void);
void EasyTVC_UART4_Init(void);

int EasyTVC_USART2_Send(const uint8_t *data, size_t length);
int EasyTVC_USART2_Receive(uint8_t *data, size_t max_length);
bool EasyTVC_USART2_TransmitComplete(void);
