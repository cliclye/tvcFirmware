#!/usr/bin/env python3
"""
EasyTVC Telemetry Viewer
Reads a raw SD card sector dump (see read-sd-data.sh) and displays graphs
"""

import struct
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

# Telemetry record structure (matches Core/Inc/data_logger.h, 23 bytes, packed, little-endian):
# timestamp u32, accel xyz i16 (mg), gyro xyz i16 (0.1 deg/s), temperature i16 (0.01 degC),
# pressure u32 (Pa), flags u8
TELEMETRY_RECORD_FORMAT = '<I6hhIB'
TELEMETRY_RECORD_SIZE = struct.calcsize(TELEMETRY_RECORD_FORMAT)
SECTOR_SIZE = 512
RECORDS_PER_SECTOR = SECTOR_SIZE // TELEMETRY_RECORD_SIZE  # 22; the last 6 bytes are 0xFF padding
FLAG_MARKER_MASK = 0xE0
FLAG_MARKER = 0xA0

class TelemetryRecord:
    def __init__(self, data):
        (self.timestamp,
         self.accel_x, self.accel_y, self.accel_z,
         self.gyro_x, self.gyro_y, self.gyro_z,
         self.temperature, self.pressure, self.flags) = struct.unpack(TELEMETRY_RECORD_FORMAT, data)

        # Convert to physical units
        self.accel_x_mg = self.accel_x  # Already in mg
        self.accel_y_mg = self.accel_y
        self.accel_z_mg = self.accel_z
        self.gyro_x_deg = self.gyro_x / 10.0  # 0.1 deg/s units
        self.gyro_y_deg = self.gyro_y / 10.0
        self.gyro_z_deg = self.gyro_z / 10.0
        self.temp_c = self.temperature / 100.0
        self.pressure_pa = self.pressure
        self.pressure_hpa = self.pressure / 100.0

    @property
    def valid(self):
        return (self.flags & FLAG_MARKER_MASK) == FLAG_MARKER

def read_telemetry_file(filename):
    """Read a raw sector dump and return the records in time order.

    Records are packed per 512-byte sector (22 each, then padding), not back to back. Sectors
    past the end of the log hold whatever was on the card before, so parsing stops at the first
    sector that has no valid record or whose timestamps go backwards.
    """
    records = []

    try:
        with open(filename, 'rb') as f:
            data = f.read()
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found")
        return None
    except OSError as e:
        print(f"Error reading file: {e}")
        return None

    last_timestamp = -1
    for sector_start in range(0, len(data) - SECTOR_SIZE + 1, SECTOR_SIZE):
        sector_records = []
        for slot in range(RECORDS_PER_SECTOR):
            offset = sector_start + slot * TELEMETRY_RECORD_SIZE
            record = TelemetryRecord(data[offset:offset + TELEMETRY_RECORD_SIZE])
            if not record.valid or record.timestamp < last_timestamp:
                break
            last_timestamp = record.timestamp
            sector_records.append(record)
        records.extend(sector_records)
        if len(sector_records) < RECORDS_PER_SECTOR:
            # A partial sector is either the final (flushed) sector or the end of the log.
            if not sector_records:
                break

    print(f"Read {len(records)} telemetry records")
    return records

def plot_telemetry(records):
    """Create graphs for all sensor data"""
    if not records:
        print("No records to plot")
        return
    
    # Extract data arrays
    timestamps = [r.timestamp for r in records]
    accel_x = [r.accel_x_mg for r in records]
    accel_y = [r.accel_y_mg for r in records]
    accel_z = [r.accel_z_mg for r in records]
    gyro_x = [r.gyro_x_deg for r in records]
    gyro_y = [r.gyro_y_deg for r in records]
    gyro_z = [r.gyro_z_deg for r in records]
    temperature = [r.temp_c for r in records]
    pressure = [r.pressure_hpa for r in records]
    
    # Convert timestamps to seconds
    time_s = [t / 1000.0 for t in timestamps]
    
    # Create figure with subplots
    fig, axes = plt.subplots(4, 1, figsize=(12, 10))
    fig.suptitle('EasyTVC Telemetry Data', fontsize=16)
    
    # Accelerometer
    axes[0].plot(time_s, accel_x, label='X', color='red')
    axes[0].plot(time_s, accel_y, label='Y', color='green')
    axes[0].plot(time_s, accel_z, label='Z', color='blue')
    axes[0].set_ylabel('Acceleration (mg)')
    axes[0].set_title('Accelerometer')
    axes[0].legend()
    axes[0].grid(True)
    
    # Gyroscope
    axes[1].plot(time_s, gyro_x, label='X', color='red')
    axes[1].plot(time_s, gyro_y, label='Y', color='green')
    axes[1].plot(time_s, gyro_z, label='Z', color='blue')
    axes[1].set_ylabel('Angular Rate (deg/s, bias-corrected)')
    axes[1].set_title('Gyroscope')
    axes[1].legend()
    axes[1].grid(True)
    
    # Temperature
    axes[2].plot(time_s, temperature, color='orange')
    axes[2].set_ylabel('Temperature (°C)')
    axes[2].set_title('Temperature')
    axes[2].grid(True)
    
    # Pressure
    axes[3].plot(time_s, pressure, color='purple')
    axes[3].set_ylabel('Pressure (hPa)')
    axes[3].set_xlabel('Time (s)')
    axes[3].set_title('Pressure')
    axes[3].grid(True)
    
    plt.tight_layout()
    plt.show()
    
    # Print statistics
    print("\n=== Telemetry Statistics ===")
    print(f"Number of records: {len(records)}")
    print(f"Duration: {time_s[-1]:.2f} seconds")
    print(f"Sample rate: {len(records) / time_s[-1]:.2f} Hz")
    print(f"\nAccelerometer (mg):")
    print(f"  X: {np.mean(accel_x):.2f} ± {np.std(accel_x):.2f}")
    print(f"  Y: {np.mean(accel_y):.2f} ± {np.std(accel_y):.2f}")
    print(f"  Z: {np.mean(accel_z):.2f} ± {np.std(accel_z):.2f}")
    print(f"\nGyroscope (deg/s):")
    print(f"  X: {np.mean(gyro_x):.2f} ± {np.std(gyro_x):.2f}")
    print(f"  Y: {np.mean(gyro_y):.2f} ± {np.std(gyro_y):.2f}")
    print(f"  Z: {np.mean(gyro_z):.2f} ± {np.std(gyro_z):.2f}")
    print(f"\nTemperature: {np.mean(temperature):.2f} ± {np.std(temperature):.2f} °C")
    print(f"Pressure: {np.mean(pressure):.2f} ± {np.std(pressure):.2f} hPa")

def main():
    if len(sys.argv) < 2:
        print("EasyTVC Telemetry Viewer")
        print("========================")
        print("Usage: python telemetry_viewer.py <telemetry_file.bin>")
        print("")
        print("Example:")
        print("  python telemetry_viewer.py telemetry_data.bin")
        sys.exit(1)
    
    filename = sys.argv[1]
    
    if not os.path.exists(filename):
        print(f"Error: File '{filename}' not found")
        sys.exit(1)
    
    print(f"Reading telemetry from: {filename}")
    records = read_telemetry_file(filename)
    
    if records:
        plot_telemetry(records)
    else:
        print("Failed to read telemetry data")
        sys.exit(1)

if __name__ == "__main__":
    main()
