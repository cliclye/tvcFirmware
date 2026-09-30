#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

void EasyTVC_USB_Init(void);
void EasyTVC_USB_Task(void);

int EasyTVC_USB_Send(const uint8_t *data, size_t length);
int EasyTVC_USB_Receive(uint8_t *data, size_t max_length);
bool EasyTVC_USB_IsConnected(void);
