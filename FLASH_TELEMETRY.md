# EasyTVC Flash Telemetry System

This system allows you to log sensor data to the onboard W25Q128JVS flash memory and read it back for analysis.

## Overview

- **Firmware**: Logs sensor data to external flash at 10 Hz
- **Flash Chip**: W25Q128JVS (16 MB) on SPI2
- **Data Format**: Binary telemetry records
- **Viewer**: Python script to visualize data

## Telemetry Data Format

Each record is 23 bytes and contains:

```c
typedef struct __attribute__((packed)) {
    uint32_t timestamp;       /* Milliseconds since boot */
    int16_t accel_x;          /* Accelerometer X (mg) */
    int16_t accel_y;          /* Accelerometer Y (mg) */
    int16_t accel_z;          /* Accelerometer Z (mg) */
    int16_t gyro_x;           /* Gyroscope X (deg/s * 100) */
    int16_t gyro_y;           /* Gyroscope Y (deg/s * 100) */
    int16_t gyro_z;           /* Gyroscope Z (deg/s * 100) */
    int16_t temperature;      /* Temperature (0.01°C) */
    uint32_t pressure;        /* Pressure (Pa) */
    uint8_t flags;           /* Status flags */
} TelemetryRecord;
```

## Usage

### 1. Flash Firmware with Logging

The firmware automatically logs sensor data to flash when it detects sensors.

```bash
cd /Users/changhyunglee/Coding/EasyTVC-Firmware
./scripts/build-firmware.sh
dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D build-arm/easytvc_safe_blank.bin
```

### 2. Run Board to Collect Data

Power on the board and let it run. The firmware will:
- Log sensor data to flash at 10 Hz
- Show record count in serial telemetry
- Red LED = flash logging status

**Serial output shows:**
```
EasyTVC: Accel=0.12,0.01,9.87 Gyro=0.02,-0.01,0.00 Temp=24.50C Press=1013hPa Logs=1234
```

### 3. Read Flash Data

Put board in DFU mode and read flash data:

```bash
cd /Users/changhyunglee/Coding/EasyTVC-Firmware
./scripts/read-flash-data.sh
```

This creates `telemetry_data.bin` with the logged data.

### 4. View Telemetry

Install dependencies (if needed):

```bash
pip install matplotlib numpy
```

View the data:

```bash
python scripts/telemetry_viewer.py telemetry_data.bin
```

This will display graphs for:
- Accelerometer (X, Y, Z)
- Gyroscope (X, Y, Z)
- Temperature
- Pressure

## LED Indicators

- **Blue LED**: Heartbeat (toggles every second)
- **Green LED**: BME280 barometer detected
- **Red LED**: Flash logging status

## Flash Memory Capacity

- **Total flash**: 16 MB
- **Logging start**: 1 MB offset (0x100000)
- **Available for logging**: 15 MB
- **Max records**: ~650,000 records
- **Max duration**: ~18 hours at 10 Hz

## Troubleshooting

### Flash not detected

- Check Red LED: If OFF, flash chip not detected
- Verify W25Q128JVS chip is present on board
- Check SPI2 connections (PB13, PB14, PB15, PC5)

### No data in file

- Ensure board ran long enough to collect data
- Check serial output for "Logs=" count
- Verify board is in DFU mode before reading

### Parse errors

- Ensure firmware and viewer use same data format
- Check that binary file is from correct address
- Verify no corruption during DFU read

## Technical Details

### Flash Chip

- **Model**: W25Q128JVS
- **Capacity**: 16 MB (128 Mbit)
- **Interface**: SPI2
- **Pins**: 
  - SCK: PB13
  - MISO: PB14
  - MOSI: PB15
  - CS: PC5

### Logging Behavior

- **Frequency**: 10 Hz (every 100ms)
- **Format**: Binary packed structs
- **Sector management**: 4KB sectors with auto-erase
- **Error handling**: Continues logging on individual record failures

### Data Retrieval

- **Method**: DFU (Device Firmware Upgrade)
- **Command**: dfu-util
- **Address**: 0x100000 (1 MB offset)
- **Size**: Configurable (default 1 MB)
