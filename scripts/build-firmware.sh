#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOCAL_ARM="$PROJECT_ROOT/.tools/arm-gnu-toolchain-14.3.rel1-darwin-arm64-arm-none-eabi"
if [[ -z "${ARM_TOOLCHAIN_DIR:-}" && -x "$LOCAL_ARM/bin/arm-none-eabi-gcc" ]]; then
    export ARM_TOOLCHAIN_DIR="$LOCAL_ARM"
fi
if [[ -n "${ARM_TOOLCHAIN_DIR:-}" ]]; then
    export PATH="$ARM_TOOLCHAIN_DIR/bin:$PATH"
fi
cmake -S "$PROJECT_ROOT" -B "$PROJECT_ROOT/build-arm" -DCMAKE_TOOLCHAIN_FILE="$PROJECT_ROOT/cmake/arm-none-eabi-gcc.cmake"
cmake --build "$PROJECT_ROOT/build-arm" --parallel 4
for image in easytvc_bench easytvc_flight easytvc_sensortest; do
    arm-none-eabi-size "$PROJECT_ROOT/build-arm/$image"
    shasum -a 256 "$PROJECT_ROOT/build-arm/$image.bin"
done
echo ""
echo "build-arm/easytvc_bench.bin   gimbal follows the board by hand; never arms. Flash this FIRST."
echo "build-arm/easytvc_flight.bin  arms on the pad, TVC during boost. Only after the bench checks."
echo "build-arm/easytvc_sensortest.bin  checks every sensor and logs all raw readings to SD. Never arms."
echo "Your rocket's settings: Core/Inc/vehicle_config.h. Procedure: docs/flight-guide.md."
