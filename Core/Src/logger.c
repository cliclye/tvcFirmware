#include "logger.h"

void EasyTVC_LogRingInit(EasyTVCLogRing *ring,
                         EasyTVCLogRecord *storage,
                         size_t capacity)
{
    ring->records = storage;
    ring->capacity = capacity;
    ring->oldest_index = 0U;
    ring->count = 0U;
    ring->overflowed = false;
}

void EasyTVC_LogRingAppend(EasyTVCLogRing *ring, const EasyTVCLogRecord *record)
{
    if (ring->capacity == 0U || ring->records == 0) {
        return;
    }

    const size_t write_index = (ring->oldest_index + ring->count) % ring->capacity;
    ring->records[write_index] = *record;
    if (ring->count == ring->capacity) {
        ring->oldest_index = (ring->oldest_index + 1U) % ring->capacity;
        ring->overflowed = true;
    } else {
        ++ring->count;
    }
}

bool EasyTVC_LogRingGet(const EasyTVCLogRing *ring,
                        size_t chronological_index,
                        EasyTVCLogRecord *record)
{
    if (record == 0 || chronological_index >= ring->count || ring->capacity == 0U) {
        return false;
    }
    *record = ring->records[(ring->oldest_index + chronological_index) % ring->capacity];
    return true;
}
