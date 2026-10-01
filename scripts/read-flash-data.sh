#!/bin/bash

# Script to read flash data from EasyTVC board using dfu-util
# This reads the W25Q128JVS external flash chip via SPI2

echo "EasyTVC Flash Data Reader"
echo "=========================="
echo ""

# Check if dfu-util is installed
if ! command -v dfu-util &> /dev/null; then
    echo "Error: dfu-util is not installed"
    echo "Install with: brew install dfu-util"
    exit 1
fi

# Configuration
FLASH_START=0x100000  # Start after 1MB (skip bootloader area)
FLASH_SIZE=0x100000   # Read 1MB of flash data
OUTPUT_FILE="telemetry_data.bin"

echo "Checking DFU device status..."
dfu-util -l

echo ""
echo "Reading flash data from 0x${FLASH_START}..."
echo "Size: $((FLASH_SIZE / 1024)) KB"
echo ""

# Read flash data
dfu-util -d 0483:df11 -a 0 -s ${FLASH_START}:${FLASH_SIZE} -U ${OUTPUT_FILE}

if [ $? -eq 0 ]; then
    echo ""
    echo "Success! Flash data saved to: ${OUTPUT_FILE}"
    echo "File size: $(wc -c < ${OUTPUT_FILE}) bytes"
    echo ""
    echo "You can now open this file in the EasyTVC Telemetry Viewer app"
else
    echo ""
    echo "Error: Failed to read flash data"
    echo "Make sure the board is in DFU mode:"
    echo "1. Disconnect USB"
    echo "2. Hold boot button"
    echo "3. Connect USB"
    echo "4. Release button after 3 seconds"
    exit 1
fi
