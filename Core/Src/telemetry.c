#include "telemetry.h"

#include <math.h>
#include <limits.h>

static size_t append_char(char *buffer, size_t capacity, size_t used, char value)
{
    if (used + 1U >= capacity) {
        return used;
    }
    buffer[used] = value;
    return used + 1U;
}

static size_t append_uint(char *buffer, size_t capacity, size_t used, uint32_t value)
{
    char digits[10];
    size_t count = 0U;
    if (value == 0U) {
        return append_char(buffer, capacity, used, '0');
    }
    while (value > 0U && count < sizeof(digits)) {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    }
    while (count > 0U) {
        used = append_char(buffer, capacity, used, digits[--count]);
    }
    return used;
}

static size_t append_int(char *buffer, size_t capacity, size_t used, int32_t value)
{
    if (value < 0) {
        used = append_char(buffer, capacity, used, '-');
        return append_uint(buffer, capacity, used, 0U - (uint32_t)value);
    }
    return append_uint(buffer, capacity, used, (uint32_t)value);
}

static size_t append_milli(char *buffer, size_t capacity, size_t used, float value)
{
    const double scaled = (double)value * 1000.0;
    const int32_t milli = !isfinite(value) ? 0 : scaled >= INT32_MAX ? INT32_MAX :
        scaled <= INT32_MIN ? INT32_MIN : (int32_t)scaled;
    return append_int(buffer, capacity, used, milli);
}

size_t EasyTVC_TelemetryFormat(const EasyTVCTelemetrySample *sample,
                               char *buffer,
                               size_t capacity)
{
    size_t used = 0U;
    if (sample == 0 || buffer == 0 || capacity == 0U) {
        return 0U;
    }

    used = append_uint(buffer, capacity, used, sample->now_ms);
    used = append_char(buffer, capacity, used, ',');
    used = append_uint(buffer, capacity, used, (uint32_t)sample->state);
    used = append_char(buffer, capacity, used, ',');
    used = append_milli(buffer, capacity, used, sample->roll_rad);
    used = append_char(buffer, capacity, used, ',');
    used = append_milli(buffer, capacity, used, sample->pitch_rad);
    used = append_char(buffer, capacity, used, ',');
    used = append_milli(buffer, capacity, used, sample->yaw_rad);
    used = append_char(buffer, capacity, used, ',');
    used = append_milli(buffer, capacity, used, sample->altitude_m);
    used = append_char(buffer, capacity, used, ',');
    used = append_milli(buffer, capacity, used, sample->vertical_speed_m_s);
    used = append_char(buffer, capacity, used, ',');
    used = append_milli(buffer, capacity, used, sample->pitch_command);
    used = append_char(buffer, capacity, used, ',');
    used = append_milli(buffer, capacity, used, sample->yaw_command);
    used = append_char(buffer, capacity, used, ',');
    used = append_char(buffer, capacity, used, sample->tvc_enabled ? '1' : '0');
    used = append_char(buffer, capacity, used, ',');
    used = append_char(buffer, capacity, used, sample->physical_arm_qualified ? '1' : '0');
    used = append_char(buffer, capacity, used, ',');
    used = append_char(buffer, capacity, used, sample->pyro_lock_active ? '1' : '0');
    used = append_char(buffer, capacity, used, '\n');
    if (used < capacity) {
        buffer[used] = '\0';
    } else {
        buffer[capacity - 1U] = '\0';
        used = capacity - 1U;
    }
    return used;
}
