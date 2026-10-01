#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "bus.h"

void EasyTVC_SPI1_Init(void);
void EasyTVC_SPI2_Init(void);
void EasyTVC_SPI3_Init(void);

int EasyTVC_SPI1_Transfer(uint8_t *tx_data, uint8_t *rx_data, size_t length);
int EasyTVC_SPI2_Transfer(uint8_t *tx_data, uint8_t *rx_data, size_t length);
int EasyTVC_SPI3_Transfer(uint8_t *tx_data, uint8_t *rx_data, size_t length);

/* CS control functions */
void EasyTVC_SPI1_CS_Accel_Set(bool state);
void EasyTVC_SPI1_CS_Gyro_Set(bool state);
void EasyTVC_SPI2_CS_Flash_Set(bool state);
void EasyTVC_SPI3_CS_SD_Set(bool state);

/* Simple single-byte transfer for flash/SD card */
uint8_t EasyTVC_SPI2_TransferByte(uint8_t data);
uint8_t EasyTVC_SPI3_TransferByte(uint8_t data);
