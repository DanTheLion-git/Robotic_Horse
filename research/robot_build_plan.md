# Highland Cow Robot — Full Build Plan

> Living document tracking all components, costs, power requirements, and build phases.
> Last updated: 2026-04-19

---

## 1. Design Overview

A bovine quadruped robot modeled after a Highland Cow, attached to a real horse-drawn carriage. The robot walks using QDD (Quasi-Direct Drive) motors with anatomically accurate bovine leg mechanics including a reciprocal apparatus for the rear legs. The carriage carries batteries, computing, and payload — reducing the need for the robot body to support all that weight.

**Key dimensions:**
| Parameter | Value |
|---|---|
| Body (L×W×H) | 1.40 × 0.60 × 0.40 m |
| Standing height (hip to ground) | 0.464 m |
| Leg segments | Thigh 0.20m, Shank 0.18m, Cannon 0.10m |
| Hoof radius | 0.04 m |
| Weight distribution | 55% front / 45% rear |

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
| Body frame (aluminum/steel) | 24.0 | Main structural frame |
| QDD leg motors (×16) | 15.5 | See §2.1 |
| Leg structure (×4) | 19.2 | 4 × (2.0 hip + 1.5 thigh_frame + 1.0 shank_frame + 0.5 cannon + 0.3 hoof) |
| Head/neck assembly | 6.5 | Neck 3.0 + skull 2.0 + jaw 0.3 + servos 0.3 + eyes/lids 0.15 + wiring 0.75 |
| Body decoration (fur/shell) | 5.0 | Faux fur or lightweight panels |
| Wiring & misc | 3.0 | Motor cables, connectors, cable management |
| IMU + sensors | 0.5 | IMU, force sensors in feet |
| **Robot subtotal** | **~73.7 kg** | |
| | | |
| **Carriage** | | |
| Carriage frame | 34.0 | Wooden deck + bench |
| Tongue (draw bar) | 5.0 | |
| Axles (×2) | 6.0 | Iron |
| Front wheels (×2, ∅85cm) | 8.0 | |
| Rear wheels (×2, ∅105cm) + hub motors | 17.0 | 5kg wheel + 3.5kg motor each |
| Hitch hardware | 1.0 | |
| **Carriage subtotal** | **~71.0 kg** | |
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
| **GRAND TOTAL** | **~165.7 kg** | Robot + carriage + electronics |

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
1. Support the robot body weight (~74 kg on 4 legs)
2. Produce walking/trotting gait motion
3. Handle dynamic forces during gait transitions

The legs **do NOT need to:**
1. Generate forward traction force to pull the carriage
2. Accelerate the combined robot+carriage mass
3. Overcome rolling resistance of the carriage

### Force analysis comparison:

| Scenario | Per-leg torque (knee, static) | Motor needed |
|---|---|---|
| Robot only (no carriage) | ~18 Nm | RobStride 04 ✓ (40 Nm cont.) |
| Robot pulling carriage (unmotorized) | ~35 Nm + traction | RobStride 04 (marginal) |
| Robot with motorized carriage | ~18 Nm | RobStride 03 possible (~€200 less) |

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
- [ ] Carriage URDF with wheel physics
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
| Aluminum leg structure | 1 set | €150 | €150 |
| 3D-printed brackets | 1 set | €50 | €50 |
| Test stand (frame) | 1 | €100 | €100 |
| Power supply (bench, 48V) | 1 | €80 | €80 |
| Jetson Orin Nano | 1 | €250 | €250 |
| Misc (wiring, connectors) | — | — | €50 |
| **Phase 1 Total** | | | **€2,210** |

**Goals:** Validate joint ranges, verify torque margins, test reciprocal apparatus mechanism, tune PID controllers, measure actual power consumption.

### Phase 2: Two-Leg (Half-Body) Prototype
Build two legs (one front + one rear) on a partial body frame to test walking mechanics.

| Component | Qty/Notes | Cost |
|---|---|---|
| Additional leg motors | 4 (1 more leg set) | €1,500 |
| Body frame (half) | aluminum extrusion | €300 |
| 48V battery (small, 10Ah) | 1 | €200 |
| IMU (BNO085) | 1 | €25 |
| CAN-bus expansion | 1 | €30 |
| Foot force sensors | 2 | €10 |
| Misc | — | €100 |
| **Phase 2 Total** (incremental) | | **€2,165** |
| **Phase 2 Running Total** | | **€4,375** |

**Goals:** Test 2-leg walking on a rail/guide, validate front-rear weight transfer, test bovine gait timing, measure ground reaction forces.

### Phase 3: Full Quadruped
Complete the robot with all 4 legs, full body frame, head, and carriage.

| Component | Qty/Notes | Cost |
|---|---|---|
| Remaining 2 leg sets | 8 motors | €3,000 |
| Full body frame | aluminum/steel | €400 |
| Head/neck assembly | servos + structure | €150 |
| Decoration (faux fur, shell) | — | €200 |
| Carriage hub motors (×2) | 350W each | €240 |
| 48V 20Ah battery | 1 | €350 |
| Charger + BMS | 1 | €80 |
| Servo controller (head) | 1 | €10 |
| Hub motor ESCs | 2 | €80 |
| Remaining sensors | — | €60 |
| Wiring + cable management | — | €100 |
| USB camera | 1 | €30 |
| **Phase 3 Total** (incremental) | | **€4,700** |
| **Phase 3 Running Total** | | **€9,075** |

---

## 8. Cost Summary

| Phase | Description | Incremental | Running Total |
|---|---|---|---|
| Phase 0 | Simulation | €0 | €0 |
| Phase 1 | Single leg + test stand | €2,210 | €2,210 |
| Phase 2 | Two legs + half body | €2,165 | €4,375 |
| Phase 3 | Full quadruped + carriage | €4,700 | €9,075 |
| | | | |
| **Optional extras** | | | |
| Lidar sensor | obstacle avoidance | €100 | |
| Speaker + amplifier | sound effects / moo | €40 | |
| LED strips (under body) | event lighting | €30 | |
| Spare motors | 2× RobStride 04 | €1,000 | |

**Estimated total project cost: €9,075 — €10,245**

---

## 9. Open Questions / TBD

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
