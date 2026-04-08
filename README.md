# Robotic Horse 🦾

A Python-based simulation of a 1-metre-tall quadruped robot with **linear ballscrew knee joints**, designed to pull a four-wheeled cart.  Built with PyBullet; structured for eventual migration to **ROS2 + Gazebo**.

## Architecture

```
Robotic_Horse/
├── main.py                          # Entry point
├── requirements.txt
├── robot/
│   ├── urdf/
│   │   └── robotic_horse.urdf       # Robot model (4 legs, 2-DOF each)
│   ├── kinematics/
│   │   ├── leg_kinematics.py        # FK / IK + ballscrew geometry
│   │   └── force_calculator.py      # Torque & force analysis
│   ├── gait/
│   │   └── walk_gait.py             # Trot gait trajectory planner
│   └── simulation/
│       └── pybullet_sim.py          # PyBullet sim loop + logging
└── tests/
```

## Leg Design

Each leg is a **2-link (thigh + shank)** design:

```
  Hip hinge  O
             |  ← thigh (0.45 m)
  Knee hinge O ──[ballscrew]── (drives knee angle)
             |  ← shank (0.50 m)
             ●  foot
```

A brushless motor rotates a ballscrew which linearly actuates the knee hinge — similar in concept to Boston Dynamics Spot's knee mechanism.

## Quick Start

```bash
# Install dependencies
pip install -r requirements.txt

# Static force analysis + plots (no simulation window)
python main.py --analysis

# Full simulation with PyBullet GUI
python main.py

# Headless simulation (CI / remote server)
python main.py --no-gui
```

## Force Analysis

`main.py` prints a table of ballscrew forces and motor torques per leg for one gait cycle, and saves `force_analysis.png`.

| Output | Description |
|--------|-------------|
| `F_ballscrew [N]` | Linear force the screw nut must exert |
| `τ_knee [N·m]`    | Equivalent knee joint torque |
| `τ_motor [N·m]`   | Required motor torque (accounts for lead & efficiency) |

## Ballscrew Parameters (edit in `force_calculator.py`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `BALLSCREW_LEAD` | 5 mm/rev | Screw lead |
| `BALLSCREW_EFF`  | 0.90    | Mechanical efficiency |
| `BALLSCREW_A`    | 0.12 m  | Pivot offset on thigh |
| `BALLSCREW_B`    | 0.10 m  | Pivot offset on shank |

## Roadmap

- [x] URDF model
- [x] FK / IK kinematics
- [x] Ballscrew force calculator
- [x] Trot gait planner
- [x] PyBullet simulation loop
- [ ] PD → PID joint controller tuning
- [ ] Terrain (steps, ramps)
- [ ] ROS2 node wrappers (sensor topics, joint command topics)
- [ ] Gazebo SDF conversion
- [ ] Hardware-in-the-loop testing
