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
arm-none-eabi-size "$PROJECT_ROOT/build-arm/easytvc_safe_blank"
shasum -a 256 "$PROJECT_ROOT/build-arm/easytvc_safe_blank.bin"
echo "Built the ARM core archive and operational firmware image."
echo "The firmware includes: GPIO, SPI1 (BMI088), I2C1 (BME280), PWM servos, USART2 telemetry."
