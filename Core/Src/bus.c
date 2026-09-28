#include "bus.h"

int EasyTVC_BusWrite(EasyTVCBus *bus, uint8_t reg, const uint8_t *data, size_t length)
{
    if (bus == 0 || bus->write == 0 || (length > 0U && data == 0)) {
        return -1;
    }
    return bus->write(bus, reg, data, length);
}

int EasyTVC_BusRead(EasyTVCBus *bus, uint8_t reg, uint8_t *data, size_t length)
{
    if (bus == 0 || bus->read == 0 || (length > 0U && data == 0)) {
        return -1;
    }
    return bus->read(bus, reg, data, length);
}

int EasyTVC_BusReadU8(EasyTVCBus *bus, uint8_t reg, uint8_t *value)
{
    return EasyTVC_BusRead(bus, reg, value, 1U);
}
