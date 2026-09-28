#pragma once

#include <stddef.h>
#include <stdint.h>
#include "pid.h"

#define EASYTVC_SETTINGS_SIZE 88U
#define EASYTVC_SETTINGS_VERSION 1U

typedef struct {
    uint16_t minimum_us;
    uint16_t center_us;
    uint16_t maximum_us;
    uint8_t channel; /* Physical output 1..4; mapping still requires board evidence. */
    int8_t direction;
} EasyTVCServoSettings;

typedef struct {
    EasyTVCPidConfig axis[2]; /* Body X and Y rotations. Body Z is noseward. */
    EasyTVCServoSettings servo[2];
    float maximum_tilt_deg;
} EasyTVCSettings;

void EasyTVC_SettingsDefault(EasyTVCSettings *settings);
bool EasyTVC_SettingsValid(const EasyTVCSettings *settings);
uint32_t EasyTVC_Crc32(const void *data, size_t length);
bool EasyTVC_SettingsEncode(const EasyTVCSettings *settings, uint8_t data[EASYTVC_SETTINGS_SIZE]);
bool EasyTVC_SettingsDecode(EasyTVCSettings *settings, const uint8_t *data, size_t length);
uint16_t EasyTVC_ServoPulse(const EasyTVCServoSettings *servo, float command);
