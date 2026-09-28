#include "bme280.h"

void EasyTVC_Bme280Init(EasyTVCBme280 *device, EasyTVCBus *bus)
{
    device->bus = bus;
    device->identity_confirmed = false;
}

bool EasyTVC_Bme280Probe(EasyTVCBme280 *device)
{
    uint8_t chip_id = 0U;
    if (EasyTVC_BusReadU8(device->bus, EASYTVC_BME280_CHIP_ID_REG, &chip_id) != 0) {
        device->identity_confirmed = false;
        return false;
    }
    device->identity_confirmed = (chip_id == EASYTVC_BME280_CHIP_ID);
    return device->identity_confirmed;
}
