#!/bin/bash
# Copy one logging session off the EasyTVC microSD card.
#
#   ./scripts/read-sd-data.sh /dev/diskN            latest session (the last power-up)
#   ./scripts/read-sd-data.sh /dev/diskN 7          session 7
#
# Layout (Core/Inc/data_logger.h): directory at sector 1024; session n at
# 2048 + (n % 32) * 16384, 16384 sectors (8 MB) each.
set -euo pipefail

DIR_SECTOR=1024
FIRST_SESSION=2048
SESSION_SECTORS=16384
MAX_SESSIONS=32

if [ $# -lt 1 ]; then
    echo "Usage: $0 <sd_card_device> [session]"
    echo "Find the device with: diskutil list   (e.g. /dev/disk4)"
    exit 1
fi

SD_DEVICE=$1
if [ ! -b "$SD_DEVICE" ] && [ ! -c "$SD_DEVICE" ]; then
    echo "Error: $SD_DEVICE is not a disk device. Run 'diskutil list'."
    exit 1
fi
# The raw node (/dev/rdiskN) is much faster on macOS.
RAW_DEVICE="${SD_DEVICE/\/dev\/disk//dev/rdisk}"
[ -c "$RAW_DEVICE" ] || RAW_DEVICE="$SD_DEVICE"

diskutil unmountDisk "$SD_DEVICE" >/dev/null 2>&1 || true
# Raw card access normally needs admin rights on macOS (read-only: nothing is written).
SUDO=""
[ -r "$RAW_DEVICE" ] || SUDO="sudo"

if [ $# -ge 2 ]; then
    SESSION=$2
else
    DIR=$($SUDO dd if="$RAW_DEVICE" bs=512 skip=$DIR_SECTOR count=1 2>/dev/null | xxd -p -l 16 | tr -d '\n')
    if [ "${DIR:0:16}" != "4554564344495231" ]; then   # "ETVCDIR1"
        echo "Error: no EasyTVC log directory on this card."
        exit 1
    fi
    # next session number, little-endian u32 at byte 8
    NEXT=$(( 16#${DIR:22:2}${DIR:20:2}${DIR:18:2}${DIR:16:2} ))
    if [ "$NEXT" -eq 0 ]; then
        echo "Error: the card has no sessions yet."
        exit 1
    fi
    SESSION=$((NEXT - 1))
fi

START=$((FIRST_SESSION + (SESSION % MAX_SESSIONS) * SESSION_SECTORS))
OUTPUT_FILE="session_${SESSION}.bin"
echo "Session $SESSION -> $OUTPUT_FILE (sectors $START..$((START + SESSION_SECTORS - 1)))"
$SUDO dd if="$RAW_DEVICE" of="$OUTPUT_FILE" bs=512 skip=$START count=$SESSION_SECTORS 2>/dev/null
[ -z "$SUDO" ] || sudo chown "$(id -u)" "$OUTPUT_FILE"
echo "Done. View it with:"
echo "  python3 scripts/telemetry_viewer.py $OUTPUT_FILE            (plots, needs matplotlib)"
echo "  python3 scripts/telemetry_viewer.py $OUTPUT_FILE --csv out.csv"
