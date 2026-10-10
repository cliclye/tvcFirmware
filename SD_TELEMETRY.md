# EasyTVC SD Card Telemetry System

This system allows you to log sensor data to a microSD card and read it back for analysis.

## Overview

- **Firmware**: Logs sensor data to microSD card at 10 Hz (downsampled from the 200 Hz control loop)
- **SD Card**: microSD on SPI3: ~328 kHz during card init, then ~10.5 MHz
- **System Clock**: 168 MHz (16 MHz HSE -> PLL, HSI fallback)

> **Use a dedicated card.** Logging is raw, starting at sector 1024. If the card holds a FAT
> filesystem (any card straight from the shop does) its FAT/data at that offset is overwritten
> and the card will need re-formatting afterwards.
- **Data Format**: Binary telemetry records (raw sectors, no file system)
- **Viewer**: Python script to visualize data

## Performance

### System Clock
- **CPU**: 168 MHz (PLL from the 16 MHz HSE; falls back to HSI if the crystal does not start)
- **AHB**: 168 MHz
- **APB1**: 42 MHz (SPI2/3, I2C, USART2, UART4); APB1 timers (TIM3/4) run at 84 MHz
- **APB2**: 84 MHz (SPI1)
- If the PLL fails to lock the firmware refuses to run and blinks the red LED.

### Control Loop
- **Loop rate**: 200 Hz, timed from SysTick (not a delay loop), watchdog-supervised (~250 ms)
- **IMU**: BMI088 +-24 g / +-2000 dps, read every loop (accel 400 Hz ODR, gyro 400 Hz / 47 Hz filter)
- **Barometer**: BME280 at 50 Hz
- **Image behaviour**: *monitor only*. The estimator and flight state machine run on real data,
  but arming is hard-wired off, so the servos stay centred and pyro outputs are never driven.

### SD Card Logging
- **Logging Rate**: 10 Hz (every 20th loop)
- **Writes never block the loop**: a sector is handed to the card in ~0.5 ms and polled until done.
  If the card is still busy when the next sector fills, the record is dropped and counted.
- **Flush**: the partly filled sector is written every second and keeps filling afterwards.

## Telemetry Data Format

Each record is 23 bytes and contains:

```c
typedef struct __attribute__((packed)) {
    uint32_t timestamp;       /* Milliseconds since boot */
    int16_t accel_x;          /* Accelerometer X (mg) */
    int16_t accel_y;          /* Accelerometer Y (mg) */
    int16_t accel_z;          /* Accelerometer Z (mg) */
    int16_t gyro_x;           /* Gyroscope X (0.1 deg/s, bias-corrected) */
    int16_t gyro_y;           /* Gyroscope Y (0.1 deg/s) */
    int16_t gyro_z;           /* Gyroscope Z (0.1 deg/s) */
    int16_t temperature;      /* Temperature (0.01°C) */
    uint32_t pressure;        /* Pressure (Pa) */
    uint8_t flags;           /* bits 7:5 = 0b101 marker; bit0 IMU ok, bit1 baro ok, bit2 gyro cal ok,
                                bit3 accel saturated, bit4 gyro saturated */
} TelemetryRecord;
```

Records are packed **per sector**: 22 records (506 bytes) then 6 bytes of 0xFF padding. The flag
marker lets the viewer tell real records from stale card contents. Accel/gyro are in the sensor
frame; the `ROLL_180_YAW_90` mounting rotation is not applied yet.

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

**Serial output (USART2, 115200 8N1, non-blocking) shows two kinds of line:**
```
now_ms,state,roll_mrad,pitch_mrad,yaw_mrad,alt_mm,vspeed_mm_s,pitch_cmd_milli,yaw_cmd_milli,tvc,arm,pyro_lock   (10 Hz)
S,imu_ok,baro_ok,sd_ok,gyro_cal_ok,records,dropped,write_errors                                              (1 Hz)
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

- **Blue LED**: Heartbeat (toggles every second, only while the IMU is working)
- **Green LED**: BME280 barometer data fresh
- **Red LED**: ON = SD logging working with no write errors (OFF = no card / init failed / write errors). Fast blinking = PLL failed.

## SD Card Details

### SD Card Interface

- **Interface**: SPI3
- **Pins**: 
  - SCK: PC10
  - MISO: PC11
  - MOSI: PC12
  - CS: PC6

### Logging Behavior

- **Sensor reading**: 200 Hz (every 5 ms)
- **SD card storage**: 10 Hz (every 20th sample, downsampling)
- **Format**: Binary packed structs (raw sectors)
- **Sector size**: 512 bytes
- **Start sector**: 1024 (512KB offset)
- **Sector management**: sector is handed to the card when full
- **Periodic flush**: partial sector rewritten every second (same sector, no data lost)
- **No file system**: Raw binary data (no FAT32 needed)

### SD Card Capacity

- **Data per record**: 23 bytes
- **Records per sector**: ~22 records (512 bytes)
- **Records per MB**: ~45,000 records
- **At 10 Hz logging**: ~75 minutes per MB
- **32 GB card**: ~38 hours of logging
- **Sensors read at 100 Hz**: Full-resolution data for TVC control
- **SD card stores at 10 Hz**: Downsampled for storage efficiency

## Troubleshooting

### SD Card not detected

- Check Red LED: If OFF, SD card not detected
- Ensure SD card is properly inserted
- Try a different SD card
- The card needs no formatting; any existing filesystem will be overwritten from sector 1024

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
- >=74 clocks with CS high at ~328 kHz, CMD0 (go idle), CMD8 (v2 check)
- CMD55 + ACMD41 (HCS for v2 cards) until ready, 1 s timeout
- CMD58 reads the OCR: SDHC/SDXC use sector addressing, SDSC cards use byte addressing (+CMD16)
- Then SPI3 is switched to ~10.5 MHz. Every wait has a timeout; a missing/failed card disables
  logging instead of hanging the flight loop.

## Advantages of SD Card Logging

- ✅ **Removable**: Easy to remove and read on any computer
- ✅ **Large capacity**: Hours of logging on a single card
- ✅ **Fast access**: Direct file reading, no DFU needed
- ✅ **Reliable**: Proven technology, widely supported
- ✅ **No file system**: Simpler firmware, faster writes
