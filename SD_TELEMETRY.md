# EasyTVC SD Card Telemetry System

This system allows you to log sensor data to a microSD card and read it back for analysis.

## Overview

- **Firmware**: Logs sensor data to microSD card at 10 Hz
- **SD Card**: microSD on SPI3
- **Data Format**: Binary telemetry records (raw sectors, no file system)
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

### 1. Flash Firmware with SD Card Logging

```bash
cd /Users/changhyunglee/Coding/EasyTVC-Firmware
./scripts/build-firmware.sh
dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D build-arm/easytvc_safe_blank.bin
```

### 2. Insert SD Card

Insert a microSD card into the EasyTVC board's SD card slot.

### 3. Run Board to Collect Data

Power on the board and let it run. The firmware will:
- Log sensor data to SD card at 10 Hz
- Show record count in serial telemetry
- Red LED = SD card status

**Serial output shows:**
```
EasyTVC: Accel=0.12,0.01,9.87 Gyro=0.02,-0.01,0.00 Temp=24.50C Press=1013hPa Logs=1234
```

### 4. Remove SD Card

After collecting data:
1. Power off the board
2. Remove the microSD card
3. Insert it into your Mac

### 5. Read SD Card Data

Find your SD card device:

```bash
diskutil list
```

Look for something like `/dev/disk2` (the number may vary).

Read the telemetry data:

```bash
cd /Users/changhyunglee/Coding/EasyTVC-Firmware
./scripts/read-sd-data.sh /dev/disk2
```

This creates `telemetry_data.bin` with the logged data.

### 6. View Telemetry

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
- **Red LED**: SD card logging status

## SD Card Details

### SD Card Interface

- **Interface**: SPI3
- **Pins**: 
  - SCK: PC10
  - MISO: PC11
  - MOSI: PC12
  - CS: PC6

### Logging Behavior

- **Frequency**: 10 Hz (every 100ms)
- **Format**: Binary packed structs (raw sectors)
- **Sector size**: 512 bytes
- **Start sector**: 1024 (512KB offset)
- **Sector management**: Auto-flush when sector full
- **No file system**: Raw binary data (no FAT32 needed)

### SD Card Capacity

- **Data per record**: 23 bytes
- **Records per sector**: ~22 records (512 bytes)
- **Records per MB**: ~45,000 records
- **At 10 Hz**: ~75 minutes per GB
- **32 GB card**: ~38 hours of logging

## Troubleshooting

### SD Card not detected

- Check Red LED: If OFF, SD card not detected
- Ensure SD card is properly inserted
- Try a different SD card
- Verify SD card is formatted (FAT32 recommended)

### No data in file

- Ensure board ran long enough to collect data
- Check serial output for "Logs=" count
- Verify SD card device path is correct
- Make sure to unmount SD card before reading

### Parse errors

- Ensure firmware and viewer use same data format
- Check that binary file is from correct sector offset
- Verify no corruption during SD card read

## Technical Details

### Data Storage

- **Raw sector writes**: No file system overhead
- **Sector buffering**: 512-byte buffer with auto-flush
- **Error handling**: Continues logging on individual sector failures
- **Binary format**: Packed structs for compact storage

### Reading on Mac

- **Raw device access**: Using `dd` command
- **Sector skipping**: Starts at sector 1024 (512KB offset)
- **Unrequired unmount**: Script automatically unmounts before reading
- **File system independent**: Reads raw sectors, not files

### SD Card Initialization

The firmware performs basic SD card initialization:
- CMD0: Go idle state
- CMD8: Check voltage compatibility
- CMD55 + ACMD41: Initialize card
- CMD24: Write single block

## Advantages of SD Card Logging

- ✅ **Removable**: Easy to remove and read on any computer
- ✅ **Large capacity**: Hours of logging on a single card
- ✅ **Fast access**: Direct file reading, no DFU needed
- ✅ **Reliable**: Proven technology, widely supported
- ✅ **No file system**: Simpler firmware, faster writes
