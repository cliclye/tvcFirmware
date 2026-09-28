#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t timestamp_ms;
    float acceleration_m_s2[3];
    float gyro_rad_s[3];
    float altitude_m;
    float vertical_speed_m_s;
    uint8_t state;
    uint8_t flags;
} EasyTVCLogRecord;

typedef struct {
    EasyTVCLogRecord *records;
    size_t capacity;
    size_t oldest_index;
    size_t count;
    bool overflowed;
} EasyTVCLogRing;

void EasyTVC_LogRingInit(EasyTVCLogRing *ring,
                         EasyTVCLogRecord *storage,
                         size_t capacity);
void EasyTVC_LogRingAppend(EasyTVCLogRing *ring, const EasyTVCLogRecord *record);
bool EasyTVC_LogRingGet(const EasyTVCLogRing *ring,
                        size_t chronological_index,
                        EasyTVCLogRecord *record);
