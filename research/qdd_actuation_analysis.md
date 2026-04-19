# QDD (Quasi-Direct Drive) Actuation Analysis

## 1. WHY QDD FOR OUR ROBOT

### Comparison with Ballscrew
| Feature | Ballscrew | QDD |
|---------|-----------|-----|
| Backdrivability | No (self-locking) | Yes (compliant) |
| Bandwidth | Low (~5 Hz) | High (~30+ Hz) |
| Force sensing | Difficult | Inherent (current = torque) |
| Impact tolerance | Poor (rigid) | Good (compliant) |
| Holding position | Excellent (no power) | Requires current |
| Weight/torque | Good | Very good |
| Complexity | High (linear → rotary) | Low (direct rotary) |

### Why QDD Wins for Walking Robots
1. **Compliance**: QDD motors are backdrivable — the robot can absorb impacts and feel the ground
2. **Speed**: Much higher control bandwidth for dynamic gaits
3. **Sensing**: Motor current directly measures joint torque — free force feedback
4. **Safety**: If something goes wrong, limbs can be pushed out of the way
5. **Simplicity**: Direct rotary output matches revolute joints perfectly

## 2. RECOMMENDED MOTOR MODULES

### Per-Joint Motor Selection

| Joint | Required Torque | Motor | Peak Torque | Cont. Torque | No-Load Speed | Mass | Price Est. |
|-------|:---:|---|:---:|:---:|:---:|:---:|:---:|
| Hip yaw | ~30 Nm | RobStride 06 | 36 Nm | 12 Nm | 320 RPM | 621g | ~€300 |
| Hip pitch | ~80 Nm | RobStride 04 | 120 Nm | 40 Nm | 200 RPM | 1420g | ~€500 |
| Knee | ~60 Nm | RobStride 04 | 120 Nm | 40 Nm | 200 RPM | 1420g | ~€500 |
| Cannon/Hock | ~20 Nm | RobStride 02 | 17 Nm | 6 Nm | 500 RPM | 405g | ~€200 |

### Total Per Robot
- 16 motors total (4 per leg × 4 legs)
- Total motor mass: 4 × (0.621 + 1.420 + 1.420 + 0.405) = **15.5 kg**
- Total motor cost: 4 × (€300 + €500 + €500 + €200) = **~€6,000**

### QDD Motor Physics
```
τ_joint = τ_motor × N × η
ω_motor = ω_joint × N
P_motor = τ_motor × ω_motor

Where:
  N = gear ratio (typically 6:1 to 10:1)
  η = gear efficiency (~0.95 for planetary)
  τ_motor = Kt × I_motor (torque constant × current)
```

### Thermal Considerations
- Continuous torque is limited by thermal dissipation
- Peak torque available for short bursts (~1-2 seconds)
- Walking gait: each joint alternates load/unload → effective duty cycle ~50%
- This means we can use close to peak torque values averaged over gait

## 3. ALTERNATIVE MOTORS TO CONSIDER
- **T-Motor AK80-64**: 120 Nm peak, 1.1kg, widely used in MIT Mini Cheetah
- **Unitree A1 motor**: 33.5 Nm peak, used in Unitree Go1/Go2
- **MyActuator RMD-X8**: 36 Nm, affordable, good for prototyping
- **Custom wound BLDC + harmonic drive**: highest performance, highest cost

## 4. MASS BUDGET UPDATE (QDD)

| Component | Mass (kg) | Notes |
|-----------|-----------|-------|
| **Robot body** | | |
| QDD leg motors (×16) | 15.5 | RobStride 06×4 + 04×8 + 02×4 |
| Body frame (aluminum extrusion) | 32.0 | 40×80mm T-slot, 150cm scale |
| Leg structure (×4, front/rear differ) | 26.0 | Front: 5.7kg, Rear: 7.3kg per leg |
| Head/neck assembly | 6.5 | Neck 3.0 + skull 2.0 + servos 0.5 + misc |
| Body decoration (fur/shell) | 8.0 | Faux fur or lightweight panels |
| Wiring & misc | 4.0 | Motor cables, connectors |
| IMU + sensors | 0.5 | IMU, FSR in feet |
| **Robot subtotal** | **~92.5 kg** | |
| | | |
| **Carriage** | | |
| Carriage body + shaft + axles + wheels | 119.0 | See robot_build_plan.md §3 |
| **Driver** | 85.0 | Seated on front bench |
| | | |
| **Electronics (on carriage)** | | |
| Compute + drivers + battery | 21.0 | Jetson + CAN + 48V 20Ah LiFePO4 |
| | | |
| **GRAND TOTAL** | **~317.5 kg** | Robot + carriage + driver + electronics |

Note: Robot body alone is ~92.5 kg (within QDD motor capacity). Carriage weight is carried by its own wheels, not the robot legs.
