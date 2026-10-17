# Shopping list: EasyTVC finless TVC rocket (Estes E12-0)

Everything needed to build and fly the rocket that `vehicle_config.h`, the 3D sim and the flight
firmware are set up for, **except the EasyTVC v0.1 flight computer (already owned)** and parts
that are 3D printed (listed at the end so nothing printable gets bought by mistake).

## Instructions for the buying agent

- Find a **purchase link for every REQUIRED and RECOMMENDED item**. OPTIONAL items: find a link,
  mark them optional.
- Match the **Must have** specs exactly. If the preferred product is unavailable, pick one that
  meets every Must-have line, and say what you substituted and why.
- Check **Reject if** before choosing. Weight limits matter: the whole rocket must stay under
  **400 g at liftoff** (design is 382 g), so prefer the lighter of two equal options.
- Ship-to region: ask the owner if unknown. Prefer one or two shops for most items (e.g. a hobby
  rocketry shop for rocketry parts, an RC/FPV shop for servos and batteries).
- Motors: rocket motors may have shipping restrictions (hazmat / ground only, adult signature).
  Note any restriction you find. Do not buy anything but the exact motor listed.
- Output a table: item ID, product name, link, price, quantity, total, notes/substitutions.

Prices are rough USD for sanity checking only.

---

## A. Motors and ignition

### A1. Rocket motor: Estes E12-0 (REQUIRED)
- **Must have:** Estes **E12-0** (24 mm × 95 mm, 0-second delay = **no ejection charge**,
  "booster" motor). ~27 N·s total impulse, ~60 g each.
- **Quantity:** 2 packs of 3 motors (6 motors).
- **Preferred:** Estes E12-0 rocket engines, 3-pack (Estes item number believed to be **1665**;
  verify on the listing. E12-4 is 1666 and E12-6 is 1667: **wrong**).
- **Reject if:** E12-4, E12-6, E12-8, or any other letter/number. Not E9, not E16, not F.
- **Search:** "Estes E12-0 engines 3 pack", "Estes 1665".
- **Why -0:** the flight computer releases the parachute with a servo at apogee; an ejection charge
  would fire into the gimbal and electronics, and its 4 s delay would fire ~2 s after apogee.
- ~$25–30 per pack.

### A2. Igniters (REQUIRED, check A1 first)
- Estes E12 packs normally include **Estes Starters** and starter plugs. If the listing does
  **not** say starters are included, buy **Estes Starters** (igniters for Estes motors), 1 pack
  of 6 or more.
- **Reject if:** high-power igniters, electric match "e-matches" sold for pyro, or Quest Q2G2
  (those are fine technically but keep it simple: Estes Starters).
- ~$8 if needed.

### A3. Launch controller (REQUIRED)
- **Must have:** model-rocket launch controller with a **safety key**, **≥ 4.5 m (15 ft)**
  lead, micro-clips, rated for Estes starters on **E** motors.
- **Preferred:** Estes **Electron Beam** launch controller (believed to be Estes item 2230; verify).
- **Reject if:** a combined pad + controller "launch set" with a rod pad only, unless it is much
  cheaper than the controller alone (the rod pad cannot be used: this rocket has no fins and
  launches from a tower, item G1).
- ~$20–30. Uses AA batteries (buy them if not owned).

---

## B. Servos

### B1. TVC servos: TowerPro MG90D (REQUIRED)
- **Must have:** **TowerPro MG90D**: *digital*, **metal gear**, ~13.4 g, ~2.1 kg·cm at 4.8 V,
  ~0.10 s/60°, standard 3-wire 2.54 mm (JR/Futaba-style) plug, horns included.
- **Quantity:** **4** (2 for the gimbal + 2 spares).
- **Acceptable alternative:** **Emax ES08MD II** (digital, metal gear, ~12 g).
- **Reject if:** **MG90S** (analog, different centre behaviour), **SG90** (plastic gears, sloppy),
  anything heavier than 15 g, "360° continuous rotation" versions, unbranded "MG90D-style" clones
  with no specs.
- **Search:** "TowerPro MG90D digital metal gear servo".
- ~$6–9 each.

### B2. Parachute-release servo (REQUIRED)
- **Must have:** 9 g micro servo, 3-wire 2.54 mm plug. Only pulls a latch pin.
- **Quantity:** 2 (1 + 1 spare).
- **Preferred:** **TowerPro SG90** (cheap, light) or a 5th MG90D.
- **Reject if:** heavier than 14 g, continuous-rotation.
- ~$3–5 each.

### B3. Servo extension leads (REQUIRED)
- **Must have:** JR/Futaba-compatible servo extension, **150 mm (6 in)**, 26 AWG,
  male-to-female.
- **Quantity:** 1 pack of at least 4.
- ~$5.

---

## C. Power

### C1. Flight battery: 2S LiPo with XT30 (REQUIRED)
- **Must have:** **2S (7.4 V nominal) LiPo**, **300–450 mAh**, **XT30** connector (the standard
  FPV-battery XT30 that plugs into a flight-controller XT30), **≥ 30C** discharge, JST-XH 3-pin
  balance lead, **weight ≤ 28 g**, **largest dimension ≤ 60 mm** (fits a sled inside a 64 mm
  tube).
- **Quantity:** **2**.
- **Preferred (any one):** GNB / Gaoneng 2S 450 mAh XT30; Tattu 2S 450 mAh XT30; CNHL 2S 450 mAh
  XT30; or a 2S 300 mAh XT30 from the same brands (lighter, ~20 g).
- **Reject if:** **3S or higher** (the board's linear 5 V regulator overheats), 1S, XT60 /
  JST-PH / Deans plugs, over 28 g, "LiHV" is acceptable only if it is still 2S.
- **Search:** "2S 450mAh LiPo XT30", "2S 300mAh LiPo XT30".
- ~$10–15 each.

### C2. LiPo balance charger (REQUIRED unless already owned)
- **Must have:** balance charger supporting **2S LiPo**, **AC (wall) powered** or sold with its
  power supply, charge current settable down to **0.5 A**.
- **Preferred:** ISDT 608AC, or SkyRC / ToolkitRC charger **with** included AC adapter.
- **Must also buy:** a **charge lead/adapter from the charger's output (usually XT60) to XT30**
  that mates with the C1 battery.
- **Reject if:** DC-only charger without a power supply (unless one is bought too), 1S-only
  "whoop" chargers.
- ~$25–45 + $5 adapter.

### C3. LiPo safe bag (RECOMMENDED)
- Fire-resistant LiPo charging/storage bag, small (fits 2 small packs).
- ~$8.

### C4. 5 V servo UBEC (RECOMMENDED, backup power for the servos)
- **Must have:** switching UBEC, input **2S–6S**, output **5 V–6 V** (5.25 V or selectable),
  **≥ 3 A continuous**, **≤ 12 g**.
- **Preferred:** **Hobbywing 3A UBEC**.
- **Why:** if the bench power test shows the board's 5 V sagging when the servos move, the servos
  get their own supply.
- **Reject if:** linear BEC, < 3 A, input range that excludes 2S.
- ~$8–12.

---

## D. Data

### D1. microSD card (REQUIRED)
- **Must have:** **microSDHC**, **8–32 GB**, Class 10 / UHS-I, from a major brand (SanDisk,
  Samsung, Kingston). Used only for this board: logging overwrites it.
- **Quantity:** 2.
- **Preferred:** SanDisk Ultra 32 GB microSDHC, or Samsung EVO Select 32 GB.
- **Reject if:** **64 GB or larger (microSDXC)**: not tested with the firmware; unbranded cards.
- ~$6–10 each.

### D2. USB-to-serial adapter (OPTIONAL, makes bench testing much easier)
- **Must have:** USB to TTL serial, **3.3 V logic** (jumper set to 3.3 V), CP2102 or FT232RL
  chip, macOS-compatible.
- **Preferred:** FT232RL module with 3.3 V / 5 V jumper; or a CP2102 module.
- **Connector note:** the board's 7-pin telemetry socket is probably **JST-PH (2.0 mm pitch)**
  or **JST-XH (2.5 mm)**. **Do not buy the mating cable until the owner measures the pitch.**
  Then buy a matching 7-pin pre-crimped cable (or a 2.0/2.5 mm crimp kit).
- ~$8 + ~$6 cable.

---

## E. Airframe and recovery

### E1. Body tube (REQUIRED)
- **Must have:** cardboard model-rocket body tube, **OD 65.5–67 mm (2.6 in, "BT-80" size)**,
  length **≥ 450 mm (18 in)**, weight **≤ 60 g for 450 mm**.
- **Quantity:** 2 (one spare, or one to cut parts from).
- **Preferred:** **Estes BT-80** body tube, 18 in (457 mm).
- **Acceptable:** LOC Precision 2.6" (66 mm) tube cut to 450 mm (heavier: weigh it).
- **Reject if:** OD outside 65.5–67 mm, fibreglass/phenolic (too heavy), shorter than 450 mm.
- ~$10–20.

### E2. Motor sleeve tube (RECOMMENDED)
- **Must have:** **24 mm motor mount tube** ("BT-50" size, ID fits a 24 mm motor), any length
  ≥ 100 mm. Cut to 95 mm; it lines the printed gimbal so the hot motor case does not touch plastic.
- **Preferred:** Estes BT-50 tube or a 24 mm motor mount tube.
- ~$5.

### E3. Parachute (REQUIRED)
- **Must have:** ripstop nylon round or hex parachute, **24 in (61 cm)**, shroud lines + swivel
  or attachment loop, **≤ 20 g**.
- **Preferred:** Estes 24" parachute, or Fruity Chutes / Top Flight 24" nylon.
- **Reject if:** plastic (polyethylene) chutes (melt and tangle), smaller than 20 in.
- After buying, enter **61 cm** as the parachute diameter in Studio's Rocket setup.
- ~$10–20.

### E4. Shock cord (REQUIRED)
- **Must have:** **Kevlar** (aramid) shock cord, **1/8 in (3 mm)** or ~100 lb rating, **1.5 m
  (5 ft)** or longer.
- **Reject if:** elastic-only cord (burns), nylon cord without a Kevlar section.
- ~$8–12.

### E5. Swivel (REQUIRED)
- **Must have:** ball-bearing swivel, stainless, ≥ 50 lb, with snap. Fishing or rocketry.
- **Quantity:** 2.
- ~$5.

### E6. Parachute protector (REQUIRED)
- **Must have:** **Nomex** parachute protector / chute blanket, **6–9 in (15–23 cm)** square.
- **Quantity:** **2** (one wraps the chute; one is cut up as the heat shield above the E12-0, which
  vents forward at burnout).
- ~$8–12 each.

### E7. Nose push-off spring (REQUIRED)
- With no ejection charge, a spring pushes the nose cone off when the servo latch lets go.
- **Must have:** stainless steel **compression spring**, OD **20–30 mm**, free length **50–70 mm**,
  wire **0.8–1.2 mm**, light rate (roughly **0.3–0.8 N/mm**).
- **Quantity:** 1 assortment or 2 springs.
- ~$5–10.

### E8. Nose ballast (REQUIRED)
- **Must have:** **80 g** of dense filler to set in epoxy in the nose: steel BBs, lead fishing
  sinkers, or tungsten putty.
- **Preferred:** 4.5 mm steel BBs (about 240 BBs = 80 g) or lead split-shot.
- ~$5–10.

---

## F. Hardware and adhesives

### F1. Heat-set inserts (REQUIRED)
- **Must have:** brass threaded heat-set inserts for 3D prints, **M2** (e.g. M2 × 3 mm long) and
  **M2.5** (M2.5 × 4 mm), assortment OK.
- ~$10.

### F2. Screws (REQUIRED)
- **Must have:** stainless socket-head cap screws assortment: **M2 × 4/6/8/10 mm** and
  **M2.5 × 6/8/10 mm**, with nuts.
- ~$10.

### F3. Nylon standoffs (REQUIRED)
- **Must have:** nylon standoff/spacer assortment kit covering **M2, M2.5 and M3** (the flight
  computer's mounting hole size has not been measured), with screws and nuts.
- ~$10.

### F4. Pushrod wire (REQUIRED)
- **Must have:** **1.0 mm (0.039 in) music wire** (spring steel rod), ≥ 300 mm. Fits MG90D horn
  holes with Z-bends.
- **Acceptable alternative:** RC **M2 ball links** + M2 threaded pushrods (4 sets).
- ~$5.

### F5. Epoxy (REQUIRED)
- **Must have:** **5-minute epoxy** and a **slow-cure (≥ 30 min) epoxy**. For the motor-mount
  joints, a high-temperature epoxy (e.g. J-B Weld Original).
- ~$15–20.

### F6. CA glue (REQUIRED)
- Thin and medium cyanoacrylate, with activator optional.
- ~$10.

### F7. Aluminium foil tape (OPTIONAL)
- Heat reflector inside the printed gimbal around the motor sleeve.
- ~$6.

---

## G. Launch tower (build)

### G1. Tower rails (REQUIRED)
- A finless TVC rocket must be held upright and released cleanly: it launches from a short tower,
  not a launch rod. Base and guides are 3D printed.
- **Must have:** **3** straight **aluminium round rods, 6 mm (1/4 in) diameter, 500 mm (20 in)**
  long (cut to ~300–400 mm).
- ~$10–15.

### G2. Tower stakes / weight (OPTIONAL)
- 3 tent stakes or a 2–3 kg weight to hold the tower base down.

---

## H. 3D printing

### H1. Filament (REQUIRED unless already owned)
- **Must have:** **PETG**, 1.75 mm, 1 kg (or ASA). PETG/ASA for everything near the motor; PLA is
  only OK for the nose cone.
- **Preferred:** Polymaker PolyLite PETG, Prusament PETG, or Bambu PETG.
- ~$20–25.

---

## I. Tools (only if not already owned)

| ID | Tool | Must have | ~Price |
|---|---|---|---|
| I1 | Digital scale | **0.01 g resolution, ≥ 500 g capacity** (weighs parts and the whole rocket) | $15 |
| I2 | Digital caliper | 150 mm, 0.01 mm | $15 |
| I3 | Soldering iron | temperature-controlled, with a **heat-set insert tip** | $30 + $8 |
| I4 | Hobby knife + blades, sandpaper 220/400, masking tape | | $15 |
| I5 | Fine-tooth saw or tube cutter | to cut tubes square | $10 |

---

## Not to buy: printed instead

Nose cone (150 mm ogive, 66 mm base, ballast cavity) · gimbal rings and motor holder ·
servo mounts · flight-computer and battery sled · centering rings and bulkheads · parachute
release latch · motor heat-shield cap · launch tower base and rail guides.

## Not to buy: already owned

EasyTVC v0.1 flight computer.

## Weight budget the purchases must fit

| Part | Budget |
|---|---|
| Body tube (450 mm) | 55 g |
| Nose cone (printed) + ballast | 30 g + 80 g |
| Parachute + shock cord + swivel | 25 g |
| Parachute servo | 9 g |
| Flight computer | 20 g |
| 2S LiPo | 22 g |
| Wiring and hardware | 15 g |
| TVC servos (2) | 26 g |
| Gimbal and motor mount (printed + sleeve) | 40 g |
| E12-0 motor | 60 g |
| **Liftoff total** | **382 g (limit 400 g)** |
