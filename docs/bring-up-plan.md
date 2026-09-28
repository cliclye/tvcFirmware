# Bring-up gates

Each gate must pass on USB power only before the next begins. No LiPo, e-match,
charge, or flight hardware is permitted during these gates.

1. Obtain continuity measurements and complete the pin-map evidence register.
2. Build and inspect the command-line `easytvc_safe_blank` baseline without
   connecting the board.
3. Add an early safe-output routine for
   confirmed pyro control nets only. Keep every unknown GPIO at reset state.
4. Build a status-only image. Review its size, address, SHA-256, and DFU write
   command; obtain explicit approval before flashing with verification.
5. Verify a benign RGB status indication over USB power.
6. Verify USB serial telemetry, then sensor identities, one device at a time.
7. Verify one servo with an unloaded bench servo.
8. Verify pyro logic only with a dummy LED-and-resistor load, a confirmed
   physical arming arrangement, and independent timing observation.

The flight state machine, TVC control loop, and flight logging are integration
work after the individual hardware gates pass.

The offline core may emit one-step deployment *requests*. Requests are policy
inputs only: they have no connection to a GPIO or driver, and the current build
lock makes every pyro authorization return false.

Host verification of that core:

```sh
./scripts/host-test.sh
```

MCU blank image (do not flash):

```sh
./scripts/build-firmware.sh
```
