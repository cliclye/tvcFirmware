#pragma once

#include "bus.h"
#include "bmi088.h"
#include "bme280.h"

void EasyTVC_HardwareBus_Init(void);

/* Create bus instances for sensors */
EasyTVCBus* EasyTVC_GetBMI088_AccelBus(void);
EasyTVCBus* EasyTVC_GetBMI088_GyroBus(void);
EasyTVCBus* EasyTVC_GetBME280Bus(void);
