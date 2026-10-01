#!/bin/bash

# Script to read telemetry data from SD card
# This reads the raw binary data from the SD card

echo "EasyTVC SD Card Telemetry Reader"
echo "================================"
echo ""

if [ $# -lt 1 ]; then
    echo "Usage: $0 <sd_card_device>"
    echo ""
    echo "Example:"
    echo "  $0 /dev/disk2"
    echo ""
    echo "To find your SD card device:"
    echo "  diskutil list"
    echo ""
    exit 1
fi

SD_DEVICE=$1
OUTPUT_FILE="telemetry_data.bin"
SECTOR_START=1024  # Start at sector 1024 (512KB offset)
SECTOR_COUNT=2048  # Read 1MB (2048 sectors * 512 bytes)

echo "Reading from: $SD_DEVICE"
echo "Start sector: $SECTOR_START"
echo "Sector count: $SECTOR_COUNT"
echo "Output file: $OUTPUT_FILE"
echo ""

# Check if device exists
if [ ! -b "$SD_DEVICE" ]; then
    echo "Error: Device $SD_DEVICE not found"
    echo "Run 'diskutil list' to find your SD card"
    exit 1
fi

# Unmount the device (if mounted)
diskutil unmountDisk "$SD_DEVICE" 2>/dev/null

# Read raw sectors
dd if="$SD_DEVICE" of="$OUTPUT_FILE" bs=512 skip=$SECTOR_START count=$SECTOR_COUNT

if [ $? -eq 0 ]; then
    echo ""
    echo "Success! Telemetry data saved to: $OUTPUT_FILE"
    echo "File size: $(wc -c < $OUTPUT_FILE) bytes"
    echo ""
    echo "View the data with:"
    echo "  python scripts/telemetry_viewer.py $OUTPUT_FILE"
else
    echo ""
    echo "Error: Failed to read SD card"
    exit 1
fi
