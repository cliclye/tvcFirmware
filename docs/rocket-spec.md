# Rocket spec sheet: TVCRocketV3

The rocket that the firmware config, Studio's rocket model and the 3D sim are set up for. The source is the
Fusion design **TVCRocketV3** (a copy is in `~/Rocket/CAD/`). Studio's numbers live in `studio/launch.py`
(`DEFAULT_PARTS`, `BODY_DEFAULTS`, `GIMBAL_DEFAULTS`, `DEFAULT_MOTOR`). **Weigh the real parts, enter them in
Studio → Rocket setup, and re-run the 3D sim**: the gains depend on CG, inertia and gimbal arm.

Positions are **mm from the nose tip** along the rocket axis (Studio's convention). In the CAD, z is measured
up from the motor-tube bottom, so tip distance = 1060 − z. Firmware body axes: **+Z to the nose**.

## 1. Overall

| | Value |
|---|---|
| Overall length (tip → nozzle exit) | **1062 mm** |
| Body | Blue Tube 2.0 3.0": **OD 79.35 / ID 76.2 mm**, **870 mm** long |
| Nose | tangent ogive **180 mm**, + 50 mm shoulder (OD 76.0) |
| Liftoff mass (F15-0) | **978 g** (CAD volumes × density + published weights) |
| Burnout mass | 918 g |
| CG | **606 mm** at liftoff, 580 mm at burnout |
| CP (Barrowman, finless) | 84 mm → **−6.6 cal** (unstable on purpose: the TVC holds it) |
| Gimbal pivot | **1003 mm** (CAD z 57.4) → gimbal arm **397 mm** |
| Pitch/yaw inertia | 0.089 kg·m² at liftoff |
| Motor | **Estes F15-0**, 29 × 114 mm, 102 g (60 g propellant), 49.6 N·s, 25–27 N peak, 3.4 s |

**Mass warning:** Estes rates the F15-0 for a **539 g** maximum liftoff mass. At 978 g the average thrust is only
about 1.5 × the rocket's weight. The rocket leaves the pad slowly (about 5.5 m/s after 1 m), and that is why
`EASYTVC_LAUNCH_ACCEL_G` is 1.8. The airframe (~350 g) is the biggest lever for saving weight.

## 2. Station map

```
   0 ── nose tip
 154    nose cone centre (printed shell + base plate, 86 g)
 180 ── nose / tube joint (tube top). Shoulder 180–230 inside the tube
 230    nose base plate: latch keeper + shock-cord eye
 233–265  chute-release servo (MG90D, channel 3) on a wall bracket; Ø2 latch pin up into the keeper
 244–290  Kevlar Z-fold bundle + 2× 3/16" quick links
 298–414  packed 30" chute, rolled, in a 13" Nomex wrap (Ø 41 so it clears the latch servo)
 416–420  ejection piston
 420–460  SP9734 spring (compressed)
 460–466 ── recovery bulkhead + M4 U-bolt (shock-cord anchor, top of the avionics)
 468–550  battery sled; 2S 450 mAh LiPo centre 510 (front window over it)
 560–678  FC sled (two rings + spine); EasyTVC board 586–666, IMU chip ≈ 640, USB-C port 611–626 (+X side)
 678–932  empty tube (servo wiring runs down here)
 932–936 ── solid blast baffle + Nomex disc (F15-0 forward vent exits through the servo cut-outs)
 976 / 1027  TVC servo B (outer gimbal) / servo A (inner gimbal)
1003 ── GIMBAL PIVOT (both axes)
1050 ── tube aft end
1062 ── nozzle exit
```

## 3. Flight computer mounting and the sensor frame

The EasyTVC (45 × 80 × 1.6 mm) stands **vertically** on the FC sled, in the plane through the rocket axis.

- **Pyro/switch screw terminals toward the nose; XT30 toward the tail.**
- **Component side faces the front window** (CAD +Y).
- **Held only by the two bottom-corner holes** (Ø ≈ 3.9 mm, centres 3.9 mm from the edges): M3 × 8 into heat-set inserts.
- **Nothing touches the board's top or front.** Two pads sit behind the holes under the logo.

| | Evidence |
|---|---|
| Sensor **+Z** = out of the component side | **Measured**: the 2026-10-15 sensor test, board flat with parts up, read +0.98 g on +Z |
| Sensor **+Y** = toward the terminals | **Inferred**: vendor hwdef `ROTATION_ROLL_180_YAW_90` puts board-forward on sensor +Y, and the silkscreen rocket points that way |
| → `EASYTVC_NOSE_AXIS` | **`POS_Y`** |

Firmware body frame with `POS_Y` (`imu_to_body` in `main.c`, `_imu_to_body` in Studio):
- **body Z = sensor +Y** = nose
- **body X = sensor +Z** = CAD +Y
- **body Y = sensor +X** = CAD −X

The IMU sits on the rocket axis, so roll adds no lever-arm acceleration. The sim places it **34 mm behind
the CG**.

**Servo mapping.** Firmware servo A turns the motor about body X (= CAD Y), so it is the CAD's **"TVC Servo B"**
(outer gimbal) on **header 2**. Firmware servo B is the CAD's **"TVC Servo A"** (inner gimbal) on **header 1**.
The tilt test on the bench sets `SERVO_A/B_DIRECTION`.

**Bench check before the first flight (2 minutes):**
1. Flash the sensor-test image.
2. Stand the assembled rocket **nose-up**.
3. Studio's sensor test names the axis reading +1 g. It must say **POS_Y**.
4. If it says NEG_Y (the hwdef sign was the other way), set `EASYTVC_NOSE_AXIS EASYTVC_AXIS_NEG_Y`, rebuild and reflash.
5. Then do the flight image's arm check: a solid green LED when the rocket is vertical.

A wrong axis fails safe, because the board never arms unless it sees itself vertical.

## 4. Motor mount and TVC gimbal

| | Value |
|---|---|
| Gimbal | K-9 V8: outer ring bolted to the airframe (2× M3×8), inner gimbal on M3×12 pivots, motor tube on M3×6 pivots |
| Motor sleeve | printed, **OD 31.8 / bore 30.0 mm** (prints ≈ 29.8, a hand-slide fit on the 29.0 mm F15) |
| Linkage | MG90D horn hole **9 mm**, pushrod lever ≈ **42 mm** → ratio 0.214 (1 mm music wire, Z-bends) |
| Travel | ±4.7° at 1250 / 1750 µs (horn ±22.5°); the airframe cut-outs clear **±12°** (the sim's mechanical stop) |
| Pivot | 59.8 mm above the nozzle exit |

## 5. Recovery

| | Value |
|---|---|
| Parachute | Estes 30" (76 cm) nylon, Cd 0.8 in the model. The sim lands at ~6.6 m/s: a 36" chute would bring that under 5 m/s |
| Shock cord | 1/8" Kevlar, anchored to the U-bolt; quick links to the chute swivel and the nose eye |
| Release | servo on **channel 3** pulls the latch pin down out of the nose keeper, and the spring pushes the nose off. No wires cross the joint |
| Servo pulses | closed **1500 µs**, open **2000 µs** |
| When | at detected apogee, backup **5.7 s** after launch |

## 6. Mass budget (Studio parts list)

| Part | g | Centre (mm) |
|---|---|---|
| Nose cone + base plate (printed) | 86 | 154 |
| Release servo + bracket + latch pin | 17 | 249 |
| Parachute 30" + Nomex + Kevlar + quick links | 103 | 317 |
| Ejection spring + piston | 43 | 433 |
| Recovery bulkhead + U-bolt | 31 | 460 |
| Battery sled (printed) | 17 | 509 |
| 2S LiPo 450 mAh + strap | 31 | 510 |
| FC sled (printed) + screws | 22 | 620 |
| EasyTVC board (entered at the IMU position) | 25 | 640 |
| Body tube (Blue Tube 3", 870 mm) | 350 | 610 |
| Glue, epoxy fillets, misc | 15 | 610 |
| Wiring + servo extensions | 32 | 840 |
| Blast baffle + Nomex disc | 16 | 934 |
| TVC servos ×2 + pushrods | 27 | 1001 |
| K-9 gimbal set (printed) + M3 hardware | 50 | 1003 |
| Motor sleeve (printed) | 11 | 1002 |
| Estes F15-0 | 102 | 1005 |
| **Total** | **978** | CG 606 |

## 7. Simulated flight (Studio 3D sim, firmware in the loop, 2026-10-31)

Default conditions: wind 2 m/s, gusts 4 m/s, 0.5° misalignment, 1° pad tilt, 0.5 m/s turbulence.

| | Result |
|---|---|
| Nominal | apogee **53 m** at **5.2 s**, top speed 18.8 m/s, worst boost tilt **1.1°**, launch detected at 0.38 s, chute at 5.1 s, lands at 6.6 m/s |
| 40 random flights | **40/40** OK; boost tilt mean 1.8°, worst **3.2°**; apogee 33–60 m; no gimbal saturation or faults |

Real flights usually come in 10–20 % lower than this.

## 8. Before you print and fly

- [ ] Measure the tube ID, motor OD, servo tabs and board holes. Weigh every part and update Studio.
- [ ] Gimbal reaches ±12° with no binding. Servo endpoints set on the bench.
- [ ] Board vertical, terminals to the nose, parts facing the window. Sensor-test nose-up says **POS_Y**.
- [ ] Servo tilt test sets both DIRECTIONs (CAD Servo B = header 2 = firmware A).
- [ ] Release servo pulls the pin clear at 2000 µs, and the nose pops with the spring.
- [ ] Re-run the 3D sim and the 40 random flights with your measured numbers.
