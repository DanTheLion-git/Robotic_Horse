# Highland Cow Robotic Quadruped — Anatomy & Engineering Research

## 1. REAL HIGHLAND COW DIMENSIONS

| Parameter | Cow (Female) | Bull (Male) | Our Robot Target |
|-----------|-------------|-------------|------------------|
| Shoulder height | 90–106 cm | 110–120 cm | ~100 cm |
| Body length | 120–145 cm | 125–150 cm | ~140 cm |
| Body width | ~60 cm | ~70 cm | ~60 cm |
| Weight | 400–600 kg | 500–800 kg | ~100–120 kg |
| Leg length | 30–35% of height | same ratio | ~35 cm total |
| Weight distribution | 55% front / 45% rear | similar | similar |

---

## 2. BOVINE LEG ANATOMY — BONE-BY-BONE

### Front Leg (Thoracic Limb)

```
SCAPULA (shoulder blade)
  │  ← Shoulder joint (ball-socket, 3 DOF: flex/ext + abd/add + rotation)
  │     Range: ~70° total, mainly flex/ext
HUMERUS (upper arm)
  │  ← Elbow joint (hinge, 1 DOF: flex/ext)
  │     Range: ~60° total
RADIUS + ULNA (forearm)
  │  ← Carpus / "Knee" (complex hinge, 1 DOF: flex/ext)
  │     Range: ~140° total
  │     NOTE: This is NOT the true knee — it's the wrist!
METACARPAL (cannon bone, fused III+IV)
  │  ← Fetlock joint (hinge, 1 DOF: flex/ext)
  │     Range: ~95° total, acts as spring
PROXIMAL PHALANX
  │  ← Pastern joint (hinge, 1 DOF)
MIDDLE PHALANX
  │  ← Coffin joint (hinge, 1 DOF)
DISTAL PHALANX + HOOF
```

**Key insight:** What looks like a "backward-bending knee" on a front leg
is actually the CARPUS (wrist). The true elbow is hidden up near the body.
The long straight segment below is the metacarpal (equivalent to our palm).

### Rear Leg (Pelvic Limb)

```
PELVIS
  │  ← Hip joint (ball-socket, 3 DOF: flex/ext + abd/add + rotation)
  │     Range: flex/ext dominant, abd/add limited
FEMUR (thigh)
  │  ← Stifle / "Knee" (modified hinge, ~1.5 DOF: flex/ext + slight rotation)
  │     Has PATELLA (kneecap) that can LOCK for standing
TIBIA + FIBULA (lower leg)
  │  ← Hock / "Ankle" (hinge, 1 DOF: flex/ext)
  │     COUPLED to stifle via reciprocal apparatus!
METATARSAL (cannon bone)
  │  ← Fetlock joint (hinge, 1 DOF: flex/ext, spring)
PROXIMAL PHALANX
  │  ← Pastern joint
MIDDLE PHALANX
  │  ← Coffin joint
DISTAL PHALANX + HOOF
```

**Key insight:** What looks like a "forward-bending knee" on a rear leg
is actually the HOCK (ankle). The true knee (stifle) is hidden near the body.

---

## 3. PASSIVE MECHANISMS — Why Cows Can Stand All Day

### 3.1 Patellar Locking (Stay Apparatus)
- The patella hooks over a ridge on the femur
- LOCKS the stifle (knee) in extension without muscle effort
- The cow can literally sleep standing up

### 3.2 Reciprocal Apparatus (Stifle-Hock Coupling)
Two tendons mechanically couple the stifle and hock:
- **Peroneus tertius** (cranial/front tendon)
- **Superficial digital flexor** (caudal/rear tendon)
- When stifle bends → hock MUST bend (and vice versa)
- This is essentially a FOUR-BAR LINKAGE in the leg!

### 3.3 Tendon Energy Storage
- Digital flexor tendons and suspensory ligament stretch under load
- Store elastic energy like springs
- Release energy during push-off phase
- Reduces muscular effort by 30-40% during locomotion

**ROBOTICS IMPLICATIONS:**
- We can replicate the reciprocal apparatus with a simple linkage or cable
- This reduces actuators needed: one motor drives both stifle AND hock
- We can add physical springs at the fetlock/pastern for energy storage
- Patellar locking = motor brake or worm gear that holds position

---

## 4. MAPPING ANATOMY TO ROBOT JOINTS

### Minimum viable joints per leg:

| Anatomical Joint | Robot Joint | DOF Needed | Why |
|-----------------|-------------|------------|-----|
| Shoulder/Hip | Hip yaw (Z-axis) | 1 | Steering, lateral placement |
| Shoulder/Hip | Hip pitch (Y-axis) | 1 | Main swing forward/backward |
| Elbow/Stifle | Knee pitch (Y-axis) | 1 | Leg folding, stance height |
| Carpus/Hock | Ankle pitch (Y-axis) | 1 | Ground clearance, shock absorption |
| Fetlock | Passive spring | 0 | Energy return (no motor needed) |

**Total: 4 DOF per leg × 4 legs = 16 actuated joints**
(Matches our current URDF! But the joint functions should be reassigned)

### Realistic segment lengths (scaled to ~100cm shoulder height):

| Segment | Real Cow | Robot Scale | Current URDF |
|---------|----------|-------------|--------------|
| Scapula/Pelvis to shoulder/hip | hidden in body | part of body | — |
| Humerus/Femur (upper leg) | ~25 cm | 20 cm | L1 = 0.20 ✓ |
| Radius/Tibia (lower leg) | ~25 cm | 18 cm | L2 = 0.18 ✓ |
| Cannon bone (metacarpal/metatarsal) | ~15 cm | 10 cm | L3 = 0.10 ✓ |
| Hoof radius | ~5 cm | 4 cm | FOOT_R = 0.04 ✓ |

**Our current proportions are actually very close to real highland cow!**

---

## 5. MODERN ROBOT ACTUATION — WHAT TO USE

### 5.1 Actuation Method Comparison

| Method | Torque/Weight | Speed | Compliance | Cost | Best For |
|--------|:---:|:---:|:---:|:---:|---|
| **QDD (Quasi-Direct Drive)** | ★★★★ | ★★★★★ | ★★★★★ | ★★★ | Dynamic walking, agility |
| **Ballscrew linear** | ★★★★★ | ★★ | ★★ | ★★★★ | High force, slow/precise |
| **High-ratio geared** | ★★★ | ★★ | ★ | ★★★★★ | Budget, static poses |
| **Hydraulic** | ★★★★★ | ★★★ | ★★ | ★ | Military, heavy-duty |

### 5.2 For Our Carriage-Pulling Robot

**Key requirement:** Pull a ~200kg carriage at walking/trotting speed on pavement.

Arguments FOR QDD:
- Better dynamic response for gait control
- Backdrivable = compliant = safer for carriage + passengers
- Can sense ground contact forces
- Industry standard for quadruped robots

Arguments FOR ballscrew:
- Higher sustained holding force (good for standing still)
- Can lock in position without power draw
- Simpler force calculation
- Higher mechanical advantage

**RECOMMENDATION: QDD for hip and knee, consider ballscrew or worm gear
for the cannon/ankle joint where high holding force matters most.**

### 5.3 Recommended QDD Motor Modules

For a ~100-120kg robot pulling a ~200kg carriage:

| Joint | Required Torque | Recommended Motor | Peak Torque | Weight | Price Est. |
|-------|:---:|---|:---:|:---:|:---:|
| Hip yaw | ~30 Nm | RobStride 06 | 36 Nm | 621g | ~€300 |
| Hip pitch (thigh) | ~80 Nm | RobStride 04 | 120 Nm | 1420g | ~€500 |
| Knee | ~60 Nm | RobStride 04 | 120 Nm | 1420g | ~€500 |
| Cannon/Ankle | ~20 Nm | RobStride 02 | 17 Nm | 405g | ~€200 |

**Total motors: 16 × average ~€400 = ~€6,400**
**Total motor weight: 4 × (621+1420+1420+405) = ~15.5 kg**

---

## 6. CARRIAGE ANALYSIS (from reference photos)

### Carriage Type: Dutch Wagonette (Break/Vis-à-vis)
- **4 wooden spoked wheels** (rear ~1.0m diameter, front ~0.7m)
- **Vis-à-vis seating** (facing each other, 4-6 passengers)
- **Two shafts** extending forward for single draft animal
- **Estimated weight:** 200-300 kg empty
- **Estimated loaded weight:** 500-700 kg (with passengers + battery)

### Traction Force Requirements

```
F_pull = (M_carriage × g × μ_roll) + (M_carriage × g × sin θ)

On flat pavement (μ_roll ≈ 0.02 for wooden wheels on smooth surface):
  F_pull = 500 × 9.81 × 0.02 = 98 N (~10 kg-force)

On 5% grade (θ ≈ 2.86°):
  F_pull = 500 × 9.81 × (0.02 + sin(2.86°)) = 98 + 245 = 343 N (~35 kg-force)

Starting from rest (acceleration 0.5 m/s²):
  F_accel = 500 × 0.5 = 250 N additional
```

**The traction force is very manageable!** 343N on a 5% grade is well within
what 4 legs can produce. The main challenge is the ROBOT'S OWN WEIGHT,
not the carriage pull.

### Shaft Attachment Design
- Rigid shafts connect to robot body at a UNIVERSAL JOINT (ball joint)
- Allows pitch (going uphill) and yaw (steering) freedom
- Vertical force on shafts is minimal (carriage balances on its own wheels)
- Horizontal force = traction pull, transmitted through robot's body frame
- Robot does NOT carry the carriage weight — only pulls it

### Advantage: Carriage Provides Stability!
- The shafts prevent the robot from falling left/right
- Like training wheels for the robot's balance
- Lateral forces are absorbed by the shafts
- Robot only needs to balance fore/aft (pitch)
- This SIGNIFICANTLY simplifies the balance controller

---

## 7. WHAT WE NEED TO UPDATE IN SIMULATION

### Phase 1: Accurate Leg Mechanics
- [ ] Implement reciprocal apparatus (stifle-hock / knee-cannon coupling)
- [ ] Add passive springs at fetlock position
- [ ] Review joint ranges of motion vs. real bovine anatomy
- [ ] Consider switching from ballscrew to QDD actuation model

### Phase 2: Carriage Integration
- [ ] Model the carriage (dimensions from photos, ~200-300kg)
- [ ] Add shaft attachment with universal joint
- [ ] Add rolling resistance physics
- [ ] Test traction force requirements

### Phase 3: Force Analysis & Motor Sizing
- [ ] Run simulation with realistic forces
- [ ] Record peak/sustained torques per joint
- [ ] Map torque requirements to available QDD motors
- [ ] Generate bill of materials with motor specifications

### Phase 4: Detailed 3D Model
- [ ] Create accurate leg geometry for Blender/Gazebo
- [ ] Add visual dressing (highland cow appearance)
- [ ] Accurate mass distribution based on chosen motors
- [ ] Run final simulation with accurate weights
