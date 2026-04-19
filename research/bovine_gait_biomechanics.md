# Bovine Gait Biomechanics — Research Reference

## 1. WALK (4-Beat Lateral Sequence)

### Footfall Pattern
- Lateral sequence: LH → LF → RH → RF
- Each footfall separated by ~25% of stride cycle
- Phase offsets: LH=0%, LF=25%, RH=50%, RF=75%

### Timing Parameters
| Parameter | Dairy Cattle | Our Robot (150cm scale) |
|-----------|-------------|------------------|
| Stride frequency | 0.8–1.2 Hz | ~0.9 Hz (T=1.1s) |
| Stride length | 1.0–1.5 m | 0.65 m (walk) |
| Duty factor | 0.60–0.75 | 0.70 |
| Swing fraction | 0.25–0.40 | 0.30 |
| Walking speed | 1.0–1.5 m/s | 0.80 m/s |

### Support Patterns
- Triple support (3 legs on ground): majority of cycle
- Quadruple support (4 legs): brief transition periods
- Never fewer than 2 legs on ground
- Front-rear weight distribution: 55% front / 45% rear

### Swing Phase Characteristics
- Protraction: pendulum-like forward swing of free limb
- Hoof lifts 5-10cm off ground (scaled: ~4-6cm for robot)
- Arc shaped trajectory: lift → forward carry → plant
- Duration: ~250-400ms per leg

### Overtracking
- Rear hoof typically lands 5-10cm ahead of where ipsilateral front hoof was
- This provides efficient forward propulsion
- At slow walk, rear may land exactly where front was (tracking)

## 2. TROT (2-Beat Diagonal)

### Footfall Pattern
- Diagonal pairs move simultaneously: FL+RR, FR+RL
- Each pair separated by 50% of stride cycle
- Phase offsets: FL=0%, RR=0%, FR=50%, RL=50%

### Timing Parameters
| Parameter | Cattle | Our Robot (150cm scale) |
|-----------|--------|------------------|
| Stride frequency | 1.5–2.5 Hz | ~1.4 Hz (T=0.7s) |
| Stride length | 1.5–2.0 m | 0.85 m (trot) |
| Duty factor | 0.40–0.55 | 0.50 |
| Swing fraction | 0.45–0.60 | 0.50 |
| Trotting speed | 2.0–4.0 m/s | 1.50 m/s |

### Aerial Phase
- Brief suspension phase possible at higher trot speeds
- All four feet momentarily off ground
- Not typical for cattle (heavy animals avoid aerial phases)
- Robot should maintain at least 1 diagonal pair grounded

## 3. JOINT KINEMATICS DURING GAIT

### Walk Joint Ranges of Motion
| Joint | Front Leg ROM | Rear Leg ROM |
|-------|:---:|:---:|
| Shoulder/Hip pitch | ~25-35° | ~20-30° |
| Elbow/Stifle (knee) | ~15-25° | ~20-35° |
| Carpus/Hock (cannon) | ~10-20° (reciprocal) | ~15-30° (reciprocal) |
| Fetlock | ~15-25° (passive spring) | ~15-25° (passive spring) |

### Trot Joint Ranges
- All ROMs increase by approximately 30-50% compared to walk
- More rapid joint excursions
- Higher angular velocities at all joints

## 4. FRONT vs REAR LEG DIFFERENCES

### Front Legs (Thoracic)
- Carry 55% of body weight (more during deceleration, up to 65%)
- Act primarily as BRAKES during the first half of stance
- Act as STRUTS during the second half
- Higher peak vertical forces
- Less hip flexion/extension range than rear

### Rear Legs (Pelvic)
- Carry 45% of body weight (more during acceleration, up to 60%)
- Act primarily as PROPULSORS
- Generate forward thrust via hip extension
- Greater hip ROM for propulsion
- Reciprocal apparatus couples stifle-hock for energy efficiency

## 5. REFERENCES
- Phillips & Morris (2001) - Dairy cattle stride characteristics
- Flower et al. (2005) - Hoof pathologies and gait kinematics
- Herlin & Drevemo (1997) - Bovine locomotion analysis
- Blackie et al. (2013) - Dairy cow gait analysis
- Abdoun et al. (2013) - Bovine gait biomechanics
