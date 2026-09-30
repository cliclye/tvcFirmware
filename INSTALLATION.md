# EasyTVC Firmware Installation Instructions

## Firmware Status

The firmware has been successfully built with the following hardware support:
- **System**: STM32F405RG @ 168 MHz, FPU enabled
- **GPIO**: All pins configured per vendor hwdef
- **SPI1**: BMI088 IMU (accel + gyro) with chip select control and data reading
- **I2C1**: BME280 barometer with internal pull-ups and data reading
- **PWM**: 4 servo outputs (TIM3/TIM4) at 50 Hz
- **USART2**: Telemetry output at 115200 baud with real sensor data
- **UART4**: RC input (configured but not used in current version)
- **LEDs**: Blue (heartbeat), Green (Barometer status), Red (Error)
- **Buzzer**: Configured but not used in current version
- **Sensor Reading**: Real-time BMI088 and BME280 data with compensation

## Built Files

- `build-arm/easytvc_safe_blank.bin` - Operational firmware binary
- `build-arm/easytvc_safe_blank.hex` - Intel HEX format
- `build-arm/libeasytvc_core.a` - Core library archive

## Prerequisites

1. **Hardware**:
   - EasyTVC flight controller board
   - USB data cable (not just charging cable)
   - BMI088 IMU connected via SPI1
   - BME280 barometer connected via I2C1
   - Servos connected to PWM outputs (optional for testing)

2. **Software**:
   - `dfu-util` for firmware flashing
   - Serial terminal (screen, minicom, or Arduino Serial Monitor)
   - Mac/Linux system (dfu-util included)

## Installation Steps

### 1. Put Board in DFU Mode

The EasyTVC board needs to be in DFU (Device Firmware Upgrade) mode to accept new firmware:

1. **Power off the board** (disconnect USB)
2. **Short the DFU pin to ground**:
   - The DFU pin is PA14 (marked on board)
   - Use a jumper wire or momentary switch to connect PA14 to GND
3. **Connect USB cable** while holding DFU pin grounded
4. **Release DFU pin** after USB is connected

The board should now be in DFU mode and ready for firmware upload.

### 2. Install dfu-util (if not already installed)

**On macOS:**
```bash
brew install dfu-util
```

**On Linux:**
```bash
sudo apt-get install dfu-util
```

### 3. Verify DFU Connection

Check if the board is detected in DFU mode:
```bash
dfu-util -l
```

You should see output like:
```
Found DFU: [0483:df11]...
```

### 4. Flash the Firmware

Navigate to the project directory and flash the firmware:
```bash
cd /Users/changhyunglee/Coding/EasyTVC-Firmware
dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D build-arm/easytvc_safe_blank.bin
```

Expected output:
```
Copyright 2005-2009 Weston Schmidt, Harald Welte and OpenMoko Inc.
Copyright 2010-2014 Tormod Volden and Stefan Schmidt
This program is Free Software and has ABSOLUTELY NO WARRANTY
...
Download    [=========================] 100%        7604 bytes
Download done.
File downloaded successfully
```

### 5. Power Cycle the Board

1. **Disconnect USB cable**
2. **Wait 2 seconds**
3. **Reconnect USB cable** (without shorting DFU pin)

The board should now boot into the new firmware.

## Testing and Telemetry

### 1. Connect Serial Terminal

Connect to the USART2 telemetry port:
- **TX**: PA2 (connect to your USB-serial adapter RX)
- **RX**: PA3 (connect to your USB-serial adapter TX)
- **GND**: Ground
- **Baud rate**: 115200
- **Data bits**: 8
- **Stop bits**: 1
- **Parity**: None

**Using screen on macOS/Linux:**
```bash
screen /dev/tty.usbserial-XXXX 115200
```

**Using minicom:**
```bash
minicom -D /dev/tty.usbserial-XXXX -b 115200
```

### 2. Expected Telemetry Output

If the firmware is working correctly, you should see actual sensor readings like:
```
EasyTVC: Accel=0.12,0.01,9.87 Gyro=0.02,-0.01,0.00 Temp=24.50C Press=1013hPa
EasyTVC: Accel=0.11,0.02,9.88 Gyro=0.01,-0.02,0.01 Temp=24.51C Press=1013hPa
EasyTVC: Accel=0.13,0.00,9.86 Gyro=0.03,-0.01,0.00 Temp=24.50C Press=1013hPa
...
```

- **Accel**: Accelerometer data in m/s² (X, Y, Z)
- **Gyro**: Gyroscope data in degrees/second (X, Y, Z)
- **Temp**: Temperature in Celsius (from BME280)
- **Press**: Atmospheric pressure in hPa (from BME280)

If sensors are not detected, you'll see:
```
EasyTVC: IMU=FAIL Baro=FAIL
```

### 3. LED Status Indicators

- **Blue LED (PB5)**: Firmware heartbeat
  - Toggles every second to show firmware is running
  - Always toggles regardless of sensor status

- **Green LED (PB4)**: Barometer status
  - ON when BME280 is detected and working
  - OFF if BME280 not detected

- **Red LED (PB3)**: Error status
  - Currently OFF (no error conditions implemented)

### 4. Servo Outputs

The firmware sets all servo outputs to center position (1500 µs pulse):
- **PWM1 (PB0)**: TIM3_CH3
- **PWM2 (PB1)**: TIM3_CH4
- **PWM3 (PB8)**: TIM4_CH3
- **PWM4 (PB9)**: TIM4_CH4

Connect servos to verify they move to center position.

## Troubleshooting

### Board not detected in DFU mode
- Ensure DFU pin (PA14) is properly grounded when connecting USB
- Try a different USB cable (some cables are power-only)
- Check USB port on your computer

### "Device not found" error
- Run `dfu-util -l` to check if device is detected
- Ensure board is in DFU mode (LEDs may flash differently)
- Try unplugging and replugging USB

### No telemetry output
- Check serial terminal settings (115200 baud, 8N1)
- Verify TX/RX connections are correct (cross-connect TX to RX)
- Check that USB-serial adapter is powered
- Ensure ground is connected between board and adapter

### BMI088 not detected
- Check SPI1 connections:
  - PA5: SCK
  - PA6: MISO
  - PA7: MOSI
  - PC7: Gyro CS
  - PC8: Accel CS
- Verify 3.3V power to sensor
- Check sensor is properly soldered

### BME280 not detected
- Check I2C1 connections:
  - PB6: SCL
  - PB7: SDA
- Verify internal pull-ups are enabled (should be in firmware)
- Check 3.3V power to sensor
- Try both I2C addresses (0x76 and 0x77)

### Servos not moving
- Check PWM output connections (PB0, PB1, PB8, PB9)
- Verify servos have external power (don't power from board)
- Check servo signal ground is connected to board ground
- Measure PWM output with oscilloscope if available

## Safety Notes

- **Pyro channels are disabled** in this firmware for safety
- **Do not use for actual rocket flights** without extensive testing
- **This is development firmware** - not flight-qualified
- **Always bench test** before any actual use
- **Verify all sensor readings** are reasonable before trusting the system

## Reverting to Original Firmware

If you need to revert to the original firmware, you can:
1. Keep a backup of the original firmware if available
2. Use the same DFU process to flash the original firmware
3. Contact the manufacturer for original firmware files

## Support

For issues with this firmware:
1. Check the hardware connections against the pin definitions
2. Verify the board is receiving proper power
3. Check that sensors are compatible with the implemented drivers
4. Review the board_pins.h file for exact pin mappings

## Next Steps

To develop this firmware further:
1. Implement full USB CDC for direct USB telemetry
2. Add complete sensor reading and calibration
3. Implement the full TVC control algorithm
4. Add settings storage and retrieval
5. Implement proper watchdog and safety systems
6. Add comprehensive error handling and reporting
7. Perform extensive bench and field testing

## File Locations

- **Firmware binary**: `build-arm/easytvc_safe_blank.bin`
- **Hardware definitions**: `board/easytvc.hwdef`
- **Pin mappings**: `Core/Inc/board_pins.h`
- **Main application**: `Core/Src/main.c`
- **Hardware drivers**: `Core/Src/stm32f4xx.c`, `Core/Src/gpio_init.c`, etc.
