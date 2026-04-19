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

| Component | Mass (kg) |
|-----------|-----------|
| Motors (16 × QDD) | 15.5 |
| Motor drivers/ESCs (16 × 100g) | 1.6 |
| Frame (aluminium extrusion) | 20.0 |
| Electronics (computer, sensors, wiring) | 3.0 |
| Battery (48V, 20Ah LiFePO4) | 8.0 |
| Leg links (4 × ~2kg each) | 8.0 |
| Decoration/dressing | 30.0 |
| **Total** | **~86 kg** |

Note: lighter than previous 100kg estimate since QDD motors are lighter than ballscrew assemblies.
