# Safe blank baseline

`easytvc_safe_blank` is a toolchain proof, not a board test.

It does only three things after reset:

1. Initializes ordinary C `.data` and `.bss` memory.
2. Disables interrupts.
3. Sleeps forever with `WFI`.

It deliberately does **not** enable any peripheral clock or write any GPIO,
timer, USART, I2C, SPI, USB, flash, SD, or pyro register. GPIO therefore stays
in STM32 reset state. The vendor pin map is recorded in `board_pins.h` but is
not applied by this image.

## Offline build

```sh
./scripts/build-firmware.sh
```

The artifacts created in `build-arm/` are deliberately excluded from version
control. Do not flash them. Any later hardware test requires a reviewed source
diff, artifact hash, exact target address, a read-back verification procedure,
and explicit user approval.

The same build also compiles `libeasytvc_core.a` for the MCU. The archive is not
linked into the blank image. It still needs a verified board adapter and a full
startup/peripheral layer to become operational firmware.
