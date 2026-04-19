# Highland Cow Robot — Full Build Plan

> Living document tracking all components, costs, power requirements, and build phases.
> Last updated: 2026-04-19

---

## 1. Design Overview

A bovine quadruped robot modeled after a Highland Cow, attached to a real horse-drawn carriage via a 3 DOF ball-joint hitch (yaw + roll at the cow, pitch at the carriage end). The robot walks using QDD (Quasi-Direct Drive) motors with anatomically accurate bovine leg mechanics including a reciprocal apparatus for the rear legs. The carriage carries batteries, computing, and payload — reducing the need for the robot body to support all that weight.

**Steering:** Toggle A/D (discrete left/right, not ramp), turn rate 0.6 rad/s.

**Gait stride lengths:**
| Gait | Stride | Max single step |
|---|---|---|
| Walk | 0.65 m | 0.45 m |
| Trot | 0.85 m | 0.45 m |

**Carriage dimensions (real measurements):**
| Parameter | Value |
|---|---|
| Track (side-to-side) | 145 cm |
| Wheelbase (front-to-back) | 205 cm |
| Front wheel diameter | 85 cm |
| Rear wheel diameter | 105 cm |
| Shaft length | 235 cm (U-shaped fork, bars at ±50cm = 1.0m apart at cow end, cross piece 85cm) |
| Hitch position | 50 cm fwd + 37 cm above front wheel center |
| Hitch type | 3 DOF ball joint: yaw(Z) + roll(X) at cow, pitch(Y) at carriage end |
| Body (L×W×H) | 250 × 110 × 110 cm, ~60 cm off ground |
| Style | Vis-à-vis (two facing bench seats) |

**Key dimensions (150cm scale):**
| Parameter | Value |
|---|---|
| Body (L×W×H) | 1.80 × 0.88 × 0.75 m |
| Withers height | 1.50 m |
| Body center height | 1.12 m |
| Body floor clearance | ~0.60 m off ground |
| Front hip height | 0.93 m (low on body — scapula) |
| Rear hip height | 1.20 m (high on body — pelvis) |
| Front hip mounts | x=+0.65m, y=±0.44m, z=-0.15m from body center |
| Rear hip mounts | x=-0.65m, y=±0.40m, z=+0.12m from body center |
| Front leg segments | Thigh 0.38m, Shank 0.34m, Cannon 0.18m |
| Rear leg segments | Thigh 0.50m, Shank 0.45m, Cannon 0.22m |
| Hoof radius | Front 0.05m, Rear 0.06m |
| Shaft bars | ±0.50m (1.0m apart), width at cow end 1.0m, cross piece 0.85m |
| Weight distribution | 55% front / 45% rear |
| Body mass (sim) | 107 kg |

---

## 2. Motor Inventory

### 2.1 Leg Motors (×4 legs = 16 motors total)

Each leg has 4 actuated joints. The rear legs use reciprocal apparatus (stifle-hock tendon coupling), so the cannon motor mainly maintains the coupling rather than producing large torques.

| Joint | Motor | Peak τ | Cont. τ | Speed | Mass | Price (est.) |
|---|---|---|---|---|---|---|
| Hip yaw | RobStride 06 | 36 Nm | 12 Nm | 33.5 rad/s | 0.621 kg | €300 |
| Hip pitch (thigh) | RobStride 04 | 120 Nm | 40 Nm | 20.9 rad/s | 1.420 kg | €500 |
| Knee (stifle) | RobStride 04 | 120 Nm | 40 Nm | 20.9 rad/s | 1.420 kg | €500 |
| Cannon (hock/fetlock) | RobStride 02 | 17 Nm | 6 Nm | 45.2 rad/s | 0.405 kg | €200 |

**Per leg:** 3.866 kg motors, €1,500
**All 4 legs:** 15.464 kg motors, €6,000

### 2.2 Carriage Wheel Motors (×2, rear wheels only)

With motorized carriage wheels, the robot legs only need to support the body weight and produce walking motion — they don't need to generate pulling force. This significantly reduces the required leg motor torque during forward locomotion.

| Component | Motor | Peak τ | Cont. τ | Mass | Price (est.) |
|---|---|---|---|---|---|
| Rear wheel L | Hub motor 36V 350W | ~40 Nm | ~15 Nm | 3.5 kg | €120 |
| Rear wheel R | Hub motor 36V 350W | ~40 Nm | ~15 Nm | 3.5 kg | €120 |

**Carriage drive total:** 7.0 kg, €240

> **💡 Budget impact:** With motorized wheels, the legs only support body weight (no traction force needed). This means we could potentially downgrade the hip pitch and knee motors from RobStride 04 to RobStride 03 for a **savings of ~€1,600** (4×2 motors × ~€200 difference). However, we should validate this in simulation first — the RobStride 04 provides safety margin for uneven terrain and dynamic loading.

### 2.3 Head/Neck Servos (7 DOF total)

Small hobby/industrial servos for expressive animation. These don't carry structural load, just move lightweight head parts.

| Joint | Servo | Torque | Mass | Price (est.) |
|---|---|---|---|---|
| Neck yaw | DS3235 (35 kg·cm) | 3.4 Nm | 0.080 kg | €25 |
| Neck pitch | DS3235 (35 kg·cm) | 3.4 Nm | 0.080 kg | €25 |
| Head nod | DS3225 (25 kg·cm) | 2.5 Nm | 0.060 kg | €18 |
| Jaw open/close | SG90 (1.8 kg·cm) | 0.18 Nm | 0.009 kg | €3 |
| Left eye pan | SG90 | 0.18 Nm | 0.009 kg | €3 |
| Right eye pan | SG90 | 0.18 Nm | 0.009 kg | €3 |
| Left eyelid | SG90 | 0.18 Nm | 0.009 kg | €3 |
| Right eyelid | SG90 | 0.18 Nm | 0.009 kg | €3 |

**Head total:** ~0.265 kg servos, €83

> **Note on force calculations:** The head assembly (~6.5 kg total including structure) is at the front of the robot and already included in the 55/45 weight distribution. The small servos consume negligible power (~5W total) and don't affect the force analysis meaningfully. Safe to continue with current calculations.

---

## 3. Mass Budget

| Component | Mass (kg) | Notes |
|---|---|---|
| **Robot body** | | |
| Body frame (aluminum/steel) | 32.0 | Larger frame at 150cm scale |
| QDD leg motors (×16) | 15.5 | See §2.1 |
| Leg structure (×4 — front/rear differ) | 26.0 | Front: 5.7kg, Rear: 7.3kg per leg |
| Head/neck assembly | 6.5 | Neck 3.0 + skull 2.0 + jaw 0.3 + servos 0.3 + eyes/lids 0.15 + wiring 0.75 |
| Body decoration (fur/shell) | 8.0 | Faux fur or lightweight panels (larger body) |
| Wiring & misc | 4.0 | Motor cables, connectors, cable management |
| IMU + sensors | 0.5 | IMU, force sensors in feet |
| **Robot subtotal** | **~92.5 kg** | |
| | | |
| **Carriage** | | |
| Carriage body (wooden frame, panels, seats) | 55.0 | Vis-à-vis body 250×110×110 cm |
| Shaft (U-shaped fork) | 15.0 | 235 cm, wooden |
| Tongue/hitch hardware | 2.0 | Iron pin + bracket |
| Axles (×2) | 12.0 | Iron, front steers |
| Front wheels (×2, ⌀85cm) | 8.0 | Wooden spoke |
| Rear wheels (×2, ⌀105cm) + hub motors | 17.0 | 5kg wheel + 3.5kg motor each |
| Carriage lanterns (×2) | 2.0 | Iron + glass |
| Folding hood | 3.0 | Wooden frame |
| Misc (springs, hardware) | 5.0 | Leaf springs, brackets |
| **Carriage subtotal** | **~119.0 kg** | |
| | | |
| **Driver** | | |
| Driver (seated) | 85.0 | On front bench |
| **Driver subtotal** | **~85.0 kg** | |
| | | |
| **Electronics (on carriage)** | | |
| Main compute (Jetson Orin / NUC) | 1.5 | |
| Motor drivers (leg QDD) | 4.0 | 4 × CAN-bus driver boards |
| Wheel motor controllers | 1.0 | 2 × ESC for hub motors |
| Battery pack | 12.0 | 48V 20Ah LiFePO4 |
| Battery charger + BMS | 2.0 | |
| Servo controller (head) | 0.2 | PCA9685 or similar |
| Networking (WiFi/4G) | 0.3 | |
| **Electronics subtotal** | **~21.0 kg** | |
| | | |
| **GRAND TOTAL** | **~317.5 kg** | Robot + carriage + driver + electronics |

---

## 4. Power & Battery Requirements

### 4.1 Power Consumption Estimates

| System | Idle (W) | Walk (W) | Trot (W) | Peak (W) |
|---|---|---|---|---|
| Leg motors (×16 QDD) | 30 | 250 | 500 | 1200 |
| Carriage hub motors (×2) | 0 | 80 | 200 | 700 |
| Head servos (7) | 2 | 3 | 3 | 5 |
| Main computer | 15 | 15 | 15 | 30 |
| Motor drivers + ESCs | 5 | 10 | 15 | 20 |
| Sensors + networking | 5 | 5 | 5 | 5 |
| **Total** | **~57 W** | **~363 W** | **~738 W** | **~1960 W** |

### 4.2 Battery Sizing

| Scenario | Duration | Energy | Battery |
|---|---|---|---|
| Walking demo (1 hour) | 60 min | 363 Wh | 48V 8Ah |
| Extended walking (2 hours) | 120 min | 726 Wh | 48V 16Ah |
| Mixed walk/trot (1 hour) | 60 min | ~550 Wh | 48V 12Ah |
| **Recommended** | **~90 min** | **~750 Wh** | **48V 16Ah LiFePO4** |

**Recommended battery:** 48V 20Ah LiFePO4 pack
- Capacity: 960 Wh (provides ~30% safety margin)
- Weight: ~12 kg
- Size: approx. 350 × 200 × 150 mm
- Cost: ~€300-400
- Charging: 48V 5A charger, ~4h full charge

### 4.3 Current Requirements

- 48V system → peak current: ~41A (1960W / 48V)
- Continuous walk: ~7.6A
- Continuous trot: ~15.4A
- Wire gauge: 10 AWG minimum for main bus, 14 AWG for branches

---

## 5. Computing & Drivers

### 5.1 Main Computer

| Option | CPU | GPU | RAM | Price | Power |
|---|---|---|---|---|---|
| **Jetson Orin Nano (recommended)** | 6-core ARM | 1024 CUDA | 8 GB | €250 | 15W |
| Jetson Orin NX | 8-core ARM | 2048 CUDA | 16 GB | €500 | 25W |
| Intel NUC 13 | i5-1340P | Iris Xe | 16 GB | €400 | 28W |

**Recommendation:** Jetson Orin Nano — sufficient for ROS2, low power, GPU for future vision.

### 5.2 Motor Driver Stack

| Component | Qty | Protocol | Price (ea.) | Notes |
|---|---|---|---|---|
| CAN-bus HAT / adapter | 2 | CAN 2.0B | €30 | 2 buses: front legs + rear legs |
| RobStride driver boards | 16 | CAN-bus | incl. w/ motors | Integrated in QDD motors |
| PCA9685 servo controller | 1 | I²C | €8 | For 7 head servos |
| Hub motor ESC (36V) | 2 | PWM/UART | €40 | For carriage wheels |

### 5.3 Sensors

| Sensor | Qty | Price | Purpose |
|---|---|---|---|
| IMU (BNO085) | 1 | €25 | Body orientation for balance |
| Force-sensitive resistor (foot) | 4 | €5 ea. | Ground contact detection |
| Encoders (in QDD motors) | 16 | incl. | Joint position feedback |
| USB camera (head) | 1 | €30 | Future: vision, interaction |
| Lidar (optional) | 1 | €100 | Future: obstacle avoidance |

---

## 6. Motorized Carriage — Impact on Leg Motor Requirements

### With motorized carriage wheels:
The robot legs **only need to:**
1. Support the robot body weight (~107 kg on 4 legs)
2. Produce walking/trotting gait motion
3. Handle dynamic forces during gait transitions

The legs **do NOT need to:**
1. Generate forward traction force to pull the carriage
2. Accelerate the combined robot+carriage mass
3. Overcome rolling resistance of the carriage

### Force analysis comparison:

| Scenario | Per-leg torque (knee, static) | Motor needed |
|---|---|---|
| Front leg (no carriage) | ~14 Nm | RobStride 04 ✓ (40 Nm cont.) |
| Rear leg (no carriage) | ~43 Nm | RobStride 04 (marginal, OK w/ duty cycling) |
| Robot pulling carriage (unmotorized) | ~55 Nm rear peak | RobStride 04 (within 120 Nm peak) |
| Robot with motorized carriage | ~43 Nm rear | RobStride 04 recommended |

### Recommendation:
Keep RobStride 04 for the prototype — the safety margin is valuable for:
- Uneven terrain
- Starting/stopping transients
- Future payload increases
- Robustness to modeling errors

Consider downgrading to RobStride 03 only after successful full-prototype testing.

---

## 7. Prototype Build Phases

### Phase 0: Simulation (current)
- [x] Highland cow leg kinematics with reciprocal apparatus
- [x] QDD motor model and force analysis
- [x] Bovine walk + trot gaits
- [x] Gazebo URDF with articulated head
- [x] Carriage URDF with wheel physics (145cm track, 205cm wheelbase)
- [x] Scale to 150cm withers height with anatomical body shape
- [x] Front/rear leg differentiation (different lengths/masses)
- [x] Ball-joint hitch (3 DOF: yaw + roll + pitch)
- [x] Dynamic stride-length gait (walk 0.65m, trot 0.85m, max step 0.45m)
- [x] Toggle A/D steering in teleop (turn rate 0.6 rad/s)
- [x] Cart body at correct height (~60cm off ground)
- [ ] Run simulation, record joint torques
- [ ] Validate motor selection against sim data
- **Cost: €0** (software only)

### Phase 1: Single Leg Prototype
Build one complete leg to validate mechanics, motor control, and joint range.

| Component | Qty | Unit Price | Total |
|---|---|---|---|
| RobStride 06 (hip yaw) | 1 | €300 | €300 |
| RobStride 04 (hip pitch) | 1 | €500 | €500 |
| RobStride 04 (knee) | 1 | €500 | €500 |
| RobStride 02 (cannon) | 1 | €200 | €200 |
| CAN-bus adapter | 1 | €30 | €30 |
| Aluminum leg structure | 1 set | €200 | €200 |
| 3D-printed brackets | 1 set | €75 | €75 |
| Test stand (frame) | 1 | €120 | €120 |
| Power supply (bench, 48V) | 1 | €80 | €80 |
| Jetson Orin Nano | 1 | €250 | €250 |
| Misc (wiring, connectors) | — | — | €50 |
| **Phase 1 Total** | | | **€2,355** |

**Goals:** Validate joint ranges, verify torque margins, test reciprocal apparatus mechanism, tune PID controllers, measure actual power consumption. At 150cm scale the rear leg is ~1.17m total — test stand must be tall enough.

### Phase 2: Two-Leg (Half-Body) Prototype
Build two legs (one front + one rear) on a partial body frame to test walking mechanics.

| Component | Qty/Notes | Cost |
|---|---|---|
| Additional leg motors | 4 (1 more leg set) | €1,500 |
| Body frame (half) | aluminum extrusion 150cm scale | €450 |
| 48V battery (small, 10Ah) | 1 | €200 |
| IMU (BNO085) | 1 | €25 |
| CAN-bus expansion | 1 | €30 |
| Foot force sensors | 2 | €10 |
| Misc | — | €100 |
| **Phase 2 Total** (incremental) | | **€2,315** |
| **Phase 2 Running Total** | | **€4,670** |

**Goals:** Test 2-leg walking on a rail/guide, validate front-rear weight transfer, test bovine gait timing, measure ground reaction forces.

### Phase 3: Full Quadruped
Complete the robot with all 4 legs, full body frame, head, and carriage.

| Component | Qty/Notes | Cost |
|---|---|---|
| Remaining 2 leg sets | 8 motors | €3,000 |
| Full body frame | aluminum/steel (1.80×0.88m) | €600 |
| Head/neck assembly | servos + structure | €150 |
| Decoration (faux fur, shell) | 150cm scale | €350 |
| Carriage hub motors (×2) | 350W each | €240 |
| 48V 20Ah battery | 1 | €350 |
| Charger + BMS | 1 | €80 |
| Servo controller (head) | 1 | €10 |
| Hub motor ESCs | 2 | €80 |
| Remaining sensors | — | €60 |
| Wiring + cable management | — | €100 |
| USB camera | 1 | €30 |
| **Phase 3 Total** (incremental) | | **€5,650** |
| **Phase 3 Running Total** | | **€10,320** |

---

## 8. Cost Summary

| Phase | Description | Incremental | Running Total |
|---|---|---|---|
| Phase 0 | Simulation | €0 | €0 |
| Phase 1 | Single leg + test stand | €2,355 | €2,355 |
| Phase 2 | Two legs + half body | €2,315 | €4,670 |
| Phase 3 | Full quadruped + carriage | €5,650 | €10,320 |
| | | | |
| **Optional extras** | | | |
| Lidar sensor | obstacle avoidance | €100 | |
| Speaker + amplifier | sound effects / moo | €40 | |
| LED strips (under body) | event lighting | €30 | |
| Spare motors | 2× RobStride 04 | €1,000 | |

**Estimated total project cost: €10,320 — €11,490** (150cm scale, ~€1,200 more than original 46cm design)

---

## 9. Frame Construction Tutorial

A detailed guide on building the aluminum extrusion frame and assembling all mechanical components.

### 9.1 Materials Needed

- **40×40mm or 40×80mm aluminum T-slot extrusion** (e.g., 8020 / Misumi / Bosch Rexroth / Dold Mechatronik)
- **Corner brackets, T-nuts, M8 bolts** — standard T-slot fastening hardware
- **6061 aluminum plate (3–5mm)** for motor mounting plates
- **Steel reinforcement plates (3mm)** for hip joint mounts (high stress area)
- **Aluminum rectangular tube** (40×20mm, 30×20mm, 25×15mm) for leg links
- **6205 deep-groove ball bearings** (25×52×15mm) for joint rotation
- **Drag chains** (15×20mm) for cable protection on each leg
- **4mm stainless steel wire rope** for rear-leg reciprocal apparatus

### 9.2 Body Frame Layout

The frame is a rectangular extrusion skeleton. The barrel-shaped Highland Cow body is a cosmetic shell mounted over it — it does not need to be structural.

1. **Main spine:** 2× 1800mm horizontal rails (40×80mm) — the backbone, running front-to-rear
2. **Cross members:** 4× 880mm rails connecting left-right at front, mid-front, mid-rear, and rear
3. **Vertical posts:** 4× at each hip position to support motor mounts
4. **Front hip mounts** at x=+0.65m, y=±0.44m, z=-0.15m from body center
5. **Rear hip mounts** at x=-0.65m, y=±0.40m, z=+0.12m from body center

> **Tip:** Pre-drill and tap the extrusion ends for M8 bolts. Use steel gusset plates at the hip mount positions — these see the highest forces during walking.

### 9.3 Motor Mounting

| Joint | Motor | Mount Position | Shaft Orientation |
|---|---|---|---|
| Hip yaw | RobStride 06 | Vertically on frame cross-member | Output shaft pointing down |
| Hip pitch | RobStride 04 | Bracket hanging from hip yaw output | Shaft horizontal (Y-axis) |
| Knee (stifle) | RobStride 04 | End of thigh link | Shaft horizontal (Y-axis) |
| Cannon (hock) | RobStride 02 | End of shank link | Shaft horizontal (Y-axis) |

### 9.4 Leg Link Construction

| Link | Section | Front Length | Rear Length | Notes |
|---|---|---|---|---|
| Thigh | 40×20mm rect. tube | 380mm | 500mm | Motor flanges at both ends |
| Shank | 30×20mm rect. tube | 340mm | 450mm | Motor flanges at both ends |
| Cannon | 25×15mm tube | 180mm | 220mm | Motor flange + hoof mount |

- Each link needs **motor mounting flanges** at both ends — CNC milled from 6061 aluminum or 3D printed in nylon PA12
- Press-fit **6205 bearings** into bearing housings at each joint for smooth, low-friction rotation
- The rear leg total length is ~1.17m — the test stand must be at least 1.5m tall

### 9.5 Reciprocal Apparatus (Rear Legs Only)

The reciprocal apparatus couples stifle (knee) and hock (cannon) motion via a tension cable, mimicking real bovine anatomy:

- **Cable:** 4mm stainless steel wire rope from stifle to hock
- **Routing:** Outside of the leg, through PTFE-lined cable guides
- **Spring tensioner** to maintain cable tension across the range of motion
- **Coupling ratio:** `hock_angle = 0.3792 - 0.85 × knee_angle`

### 9.6 Hoof / Foot Design

- **Rubber hemisphere** (Shore 60A durometer) bonded to an aluminum disc
- Front hooves: 50mm radius, Rear hooves: 60mm radius
- **Force-sensitive resistor (FSR)** — Interlink 406 — placed under the rubber pad for ground contact detection
- **Mounting:** M8 threaded stud screws into the cannon link end cap

### 9.7 Head / Neck Construction

| Part | Actuator | Notes |
|---|---|---|
| Neck yaw | DS3235 servo | Pan-tilt bracket, lower axis |
| Neck pitch | DS3235 servo | Pan-tilt bracket, upper axis |
| Head nod | DS3225 servo | Tilts head relative to neck |
| Jaw | SG90 micro servo | Simple hinge for mouth open/close |
| Eyes (×2) | SG90 micro servos | 3D-printed eye mechanism |
| Eyelids (×2) | SG90 micro servos | Blink mechanism |

- **Head shell:** Lightweight foam or fiberglass over a 3D-printed internal skeleton
- **Attachment point:** Front of body frame at x=+0.90m, z=+0.42m above body center
- **Total head/neck mass:** ~6.5 kg (already included in 55/45 weight distribution)

### 9.8 Shaft Connection (Harness / Hitch)

The shaft connects the cow to the carriage via a 3 DOF ball joint:

1. **Ball-joint adapter:** Universal joint (U-joint) rated for 500+ kg
2. **Mount position:** Below body center at z=-0.30m from body center
3. **Degrees of freedom:** Yaw (Z) + Roll (X) at the cow end; Pitch (Y) at the carriage end
4. **Alternative:** Two stacked pillow-block bearings oriented 90° apart
5. **Shaft bars** run alongside the body at ±0.50m (1.0m total spacing), clearing the 0.88m body width
6. **Shaft width** at cow end: 1.0m; cross piece: 0.85m

### 9.9 Wiring Routing

1. **Main 48V bus:** From carriage battery → through shaft cable tray → body frame cable tray → motor drivers
2. **CAN bus:** Shielded twisted pair, daisy-chained through all 4 leg motor sets (IDs 1–4 front-left, 5–8 front-right, 9–12 rear-left, 13–16 rear-right)
3. **Servo signals:** Ribbon cable from PCA9685 (I²C) to all 7 head servos
4. **Drag chains:** One per leg (15×20mm), protecting motor power + CAN cables during walking motion
5. **Weatherproof connectors:** IP67 circular connectors at the shaft/body junction (power, CAN, servo signals)

---

## 10. Suggested Parts List with Sources

A shopping list organized by build phase with suggested suppliers.

### Phase 1 — Single Leg

| Part | Spec | Qty | Source | Est. Price |
|---|---|---|---|---|
| RobStride 06 | Hip yaw QDD | 1 | LimX Dynamics / AliExpress | €300 |
| RobStride 04 | Hip/knee QDD | 2 | LimX Dynamics / AliExpress | €1,000 |
| RobStride 02 | Cannon QDD | 1 | LimX Dynamics / AliExpress | €200 |
| 40×80 T-slot extrusion | 1800mm | 2 | Misumi / Dold Mechatronik | €40 |
| 40×40 T-slot extrusion | 880mm | 4 | Misumi / Dold Mechatronik | €25 |
| Corner brackets + hardware | M8 set | 1 | Amazon / RS Components | €30 |
| 6061 aluminum plate | 300×200×5mm | 4 | Metaal24 / Aluminium Online | €40 |
| Aluminum rectangular tube | 40×20×2mm, 2m | 2 | Local metal supplier | €15 |
| 6205 bearings | 25×52×15mm | 4 | SKF / Amazon | €20 |
| CAN-bus USB adapter | PEAK PCAN-USB | 1 | PEAK / Mouser | €30 |
| Jetson Orin Nano (8GB) | Dev kit | 1 | NVIDIA / RS Components | €250 |
| 48V bench power supply | 48V 10A | 1 | Amazon / AliExpress | €80 |
| 3D printed brackets | Nylon PA12 | 1 set | JLC3DP / own printer | €75 |
| Test stand frame | T-slot, 1.5m tall | 1 | — | €120 |
| Rubber foot pad | Shore 60A, 50mm | 1 | Amazon | €10 |
| Misc (wiring, connectors, M8 bolts) | — | — | — | €50 |

### Phase 2 — Add Second Leg + Body Frame

| Part | Spec | Qty | Source | Est. Price |
|---|---|---|---|---|
| Additional leg motor set | 06+04+04+02 | 1 | LimX Dynamics | €1,500 |
| Body frame extrusion | 40×80, various lengths | 6m total | Misumi | €120 |
| Cross members + brackets | — | set | — | €80 |
| Motor mounting plates | CNC 6061 aluminum | 4 | JLC CNC / local | €100 |
| BNO085 IMU | 9-DOF | 1 | Adafruit / Mouser | €25 |
| FSR foot sensors | Interlink 406 | 2 | Mouser / Digi-Key | €10 |
| 48V 10Ah LiFePO4 | — | 1 | AliExpress / Fogstar | €200 |
| CAN bus cable | Shielded twisted pair, 5m | 1 | RS Components | €15 |
| Drag chains | 15×20mm, 1m | 2 | Amazon / AliExpress | €15 |
| Cable + connectors | — | — | — | €50 |

### Phase 3 — Full Quadruped + Head + Carriage

| Part | Spec | Qty | Source | Est. Price |
|---|---|---|---|---|
| Remaining 2 leg motor sets | — | 2 | — | €3,000 |
| Full body frame completion | — | — | — | €350 |
| Head/neck servos | DS3235×2, DS3225×1, SG90×5 | 1 set | Amazon | €83 |
| Head structure | Foam + fiberglass + 3D print | 1 | — | €70 |
| Faux fur / decoration | — | — | — | €350 |
| Hub motors (carriage) | 36V 350W | 2 | AliExpress / eBay | €240 |
| Hub motor ESCs | 36V 25A | 2 | AliExpress | €80 |
| 48V 20Ah LiFePO4 (upgrade) | — | 1 | Fogstar / AliExpress | €350 |
| BMS + charger | 48V 5A | 1 | — | €80 |
| USB camera | Arducam / Logitech | 1 | Amazon | €30 |
| PCA9685 servo controller | I²C | 1 | Adafruit | €10 |
| U-joint (hitch) | HD, rated 500kg+ | 1 | Industrial supplier | €50 |
| Weatherproof connectors | IP67 circular | 4 | RS / Mouser | €40 |

---

## 11. Assembly Order (Step-by-Step)

1. **Build test stand** from T-slot extrusion (1.5m tall, needs to support a leg hanging freely)
2. **Assemble one leg:** Mount motors into 3D-printed/CNC brackets, connect links, test full range of motion by hand
3. **Wire CAN bus:** Jetson Orin → PCAN-USB adapter → daisy chain through 4 motors (CAN IDs 1–4)
4. **Flash motor firmware,** verify encoder feedback, set CAN IDs
5. **Run single-leg calibration script** — find joint limits, verify direction conventions match URDF
6. **Standing test** — hold leg at neutral stance, measure current draw vs. simulation prediction
7. **Walking test** — single-leg walking motion on test stand, verify reciprocal apparatus cable coupling
8. **Measure actual torques** and compare to simulation predictions — validate motor selection
9. **Build half-body frame** (one front + one rear leg mount, partial spine)
10. **Mount IMU** (BNO085) on body frame, wire second leg (CAN IDs 5–8)
11. **Rail-guided 2-leg walking test** — front+rear leg on a linear guide rail
12. **Validate front-rear weight transfer** — check 55/45 distribution matches sim
13. **Expand to full 4-leg body,** mount head, connect carriage via U-joint hitch
14. **Full system integration** — outdoor testing on flat surface, then gravel/grass

---

## 12. Open Questions / TBD

- [ ] Exact carriage weight (user to measure)
- [ ] Carriage hitch height and angle tolerance
- [ ] Terrain requirements (flat paved only? gravel? grass?)
- [ ] Weather protection for electronics
- [ ] Emergency stop mechanism
- [ ] Transportation/storage of robot+carriage
- [ ] Insurance/safety requirements for public use
- [ ] Sound system for mooing and other bovine expressions?
- [ ] Power consumption validation from simulation
- [ ] Motor downgrade feasibility after Phase 1 testing
