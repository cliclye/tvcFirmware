# EasyTVC Studio protocol v1

Implemented in `protocol.c`/`runtime.c` and the Mac client; no board USB CDC adapter
exists yet. Transport: nominal 115200 CDC serial, ASCII, LF endings, maximum 255
bytes including newline. No CR characters.

```
E1 SEQUENCE COMMAND PAYLOAD *CRC32\n
```

CRC32 is eight hex digits: reflected CRC-32/ISO-HDLC, polynomial `0xEDB88320`,
initial/final XOR `0xffffffff`. It covers bytes through PAYLOAD, excluding the
space before `*`. SEQUENCE is decimal uint32. Responses echo the sequence with
`OK` or `ERR` in the command position. The client ignores wrong sequences/bad
CRCs, times out at two seconds, and never silently retries writes.

| Command | Payload | OK response |
| --- | --- | --- |
| HELLO | `-` | `EASYTVC,1,UNVERIFIED` or `EASYTVC,1,VERIFIED` |
| STATUS | `-` | `state,physical_arm,hardware_verified` |
| GET | `-` | 176 hex digits: the 88-byte settings record |
| SET | 176 hex digits | `SAVED` or `UNCHANGED` |

States: 0 idle, 1 armed, 2 boost, 3 coast, 4 apogee, 5 descent, 6 landed, 7 fault.
SET needs verified hardware, idle state, and physical arm off. Every field is
validated before persistence. `UNCHANGED` requires a matching already-persisted
configuration. Errors include `COMMAND`, `CONFIG`, `HEX`, `LENGTH`,
`HARDWARE_UNVERIFIED`, `ARMED_OR_ACTIVE`, and `STORAGE`.

Bad framing/CRC and overflows are dropped; parsing resumes after LF. There are
**no arm, servo movement, firing, or bootloader commands**.

Settings record, little-endian:

| Offset | Bytes | Meaning |
| --- | --- | --- |
| 0 | 4 | ASCII `ETC1` |
| 4 | 4 | uint32 version 1 |
| 8 | 28 | Axis X: seven IEEE-754 binary32 values |
| 36 | 28 | Axis Y: same fields |
| 64 | 8 | X servo: uint16 min/center/max µs, uint8 channel, int8 direction |
| 72 | 8 | Y servo: same fields |
| 80 | 4 | binary32 maximum tilt degrees |
| 84 | 4 | uint32 CRC32 over bytes 0–83 |

PID order: `kp, ki, kd, integrator_limit, output_limit, output_rate_limit,
derivative_cutoff_hz`. Channels are 1–4 without duplicates; direction +1/-1.
Endpoints are 900–2100 µs with at least 25 µs on each side of center. Tilt limit
is 5–45°. Serialization does not depend on C struct padding.

Diagnostic telemetry can be interleaved as twelve CSV integers, no header:

```
ms,state,roll_mrad,pitch_mrad,yaw_mrad,altitude_mm,vertical_speed_mm_s,command_x_milli,command_y_milli,tvc_enabled,physical_arm,pyro_locked
```

Angles are Euler; the controller uses the quaternion. This legacy CSV has no CRC
and must not drive a safety interlock. Studio marks it stale after one second and
keeps at most 2,000 samples in memory.
