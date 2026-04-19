# Carriage Analysis — Dutch Wagonette

## 1. CARRIAGE IDENTIFICATION
Based on reference photos in `../reference/`:
- **Type**: Dutch Wagonette (Break / Vis-à-vis)
- **Construction**: Wooden body, iron fittings, wooden spoked wheels
- **Seating**: Vis-à-vis (facing each other), 4-6 passengers
- **Draft**: Two shafts extending forward for single draft animal

## 2. DIMENSIONS (estimated from photos)
| Component | Dimension |
|-----------|-----------|
| Overall length | ~3.0-3.5 m |
| Overall width | ~1.4-1.6 m |
| Body height | ~0.7-0.9 m (floor to top) |
| Front wheel diameter | ~0.7 m |
| Rear wheel diameter | ~1.0 m |
| Shaft length | ~2.0-2.5 m from front axle |
| Shaft spacing | ~0.5-0.6 m (fits draft animal body) |

## 3. WEIGHT ESTIMATES
| Configuration | Weight (kg) |
|---------------|-------------|
| Empty carriage | 200-300 |
| With battery/electronics | 250-350 |
| With 2 passengers | 400-500 |
| With 4 passengers | 550-700 |

## 4. TRACTION FORCE REQUIREMENTS

### Rolling Resistance
```
F_roll = M × g × μ_roll

μ_roll estimates:
  Iron-shod wheels on cobblestone: 0.03-0.05
  Iron-shod wheels on smooth pavement: 0.02-0.03
  Rubber tires on pavement: 0.01-0.02
```

### Grade Resistance
```
F_grade = M × g × sin(θ)

Common grades:
  Flat: 0 N
  2% grade: M × 0.196 N
  5% grade: M × 0.490 N
  10% grade: M × 0.981 N
```

### Force Table (500 kg loaded carriage)
| Condition | Force (N) | Equivalent (kg-force) |
|-----------|-----------|----------------------|
| Flat, smooth | 98 | 10.0 |
| Flat, cobblestone | 196 | 20.0 |
| 5% grade, smooth | 343 | 35.0 |
| 10% grade, smooth | 589 | 60.0 |
| Starting (0.5 m/s²) | +250 | +25.5 |

## 5. SHAFT ATTACHMENT DESIGN
- Rigid shafts connect to robot body at a UNIVERSAL JOINT (2 DOF: pitch + yaw)
- Shafts provide LATERAL STABILITY — prevents robot falling sideways
- Vertical force on shafts is minimal (carriage balanced on its own wheels)
- Horizontal force = traction (pulling force)
- Robot does NOT carry the carriage weight — only pulls it

## 6. STABILITY ADVANTAGE
The carriage shafts act like training wheels:
- Prevent roll (left/right tipping)
- Allow pitch (up/downhill adjustment)
- Allow yaw (steering)
- Robot only needs fore/aft (pitch) balance
- Dramatically simplifies the balance controller

## 7. FUTURE SIMULATION WORK (Phase 2)
- [ ] Model carriage as separate URDF/SDF with proper wheel physics
- [ ] Add shaft attachment with ball joint constraints
- [ ] Implement rolling resistance as external force
- [ ] Test turning radius with shaft constraints
- [ ] Simulate hill climbing with grade resistance
