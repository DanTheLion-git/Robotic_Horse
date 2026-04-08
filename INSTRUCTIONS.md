# INSTRUCTIONS — Robotic Horse Project

This project has two simulation stacks.  Choose one to start with:

| Stack | Folder | Requirements | Best for |
|-------|--------|-------------|---------|
| **PyBullet** (prototype) | `pybullet/` | Python 3.11–3.12 | Quick force analysis, no ROS2 needed |
| **ROS2 + Gazebo** (full stack) | `ros2_ws/` | Ubuntu 22.04 + ROS2 Humble | Full physics simulation, hardware-ready |

---

## Part A — Force Analysis & PyBullet

### Prerequisites

- **Python 3.11 or 3.12** recommended (PyBullet has no prebuilt wheel for Python 3.14)
- **Git** (to clone the repository)

### 1. Clone the Repository

```bash
git clone https://github.com/DanTheLion-git/Robotic_Horse.git
cd Robotic_Horse
```

### 2. (Recommended) Create a Virtual Environment

```bash
python -m venv .venv
```

Activate it:

- **Windows:** `.venv\Scripts\activate`
- **macOS / Linux:** `source .venv/bin/activate`

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

> **Note:** If `pybullet` fails to install (Python 3.14 has no prebuilt wheel),
> the force analysis still works — only the 3D simulation requires PyBullet.
> ```bash
> pip install numpy scipy matplotlib
> ```

### 4. Run the Static Force Analysis

```bash
python main.py --analysis
```

This will:
- Generate one full trot gait cycle for all four legs
- Print a table of **ballscrew forces** and **motor torques** per leg
- Save a force plot to `force_analysis.png`

Skip the plot window (headless / CI):

```bash
python main.py --analysis --no-plots
```

### 5. (Optional) Run the PyBullet 3D Simulation

Requires PyBullet installed (Python <= 3.12).

```bash
# With 3D GUI window
python pybullet/run_pybullet.py

# Headless
python pybullet/run_pybullet.py --no-gui
```

---

## Part B — ROS2 + Gazebo (Full Stack)

For the full physics simulation with live force monitoring, see the detailed tutorial:

**[`gazebo/INSTRUCTIONS.md`](gazebo/INSTRUCTIONS.md)**

Quick reference once installed:

```bash
cd ~/Robotic_Horse/ros2_ws

# Terminal 1 — Gazebo + robot + controllers
ros2 launch robotic_horse_control gazebo.launch.py

# Terminal 2 — Gait planner + force reporter
ros2 launch robotic_horse_control robot_control.launch.py

# Terminal 3 — Monitor forces live
ros2 topic echo /robotic_horse/ballscrew_forces
```

---

## Understanding the Force Analysis Output

| Column | Description |
|--------|-------------|
| `Peak F_nut [N]` | Maximum linear force the ballscrew nut must exert |
| `Mean F_nut [N]` | Average over the gait cycle |
| `Peak tau_motor [N*m]` | Required motor shaft torque (screw lead + efficiency) |
| `Nut stroke [mm]` | How far the nut travels during one gait cycle |

---

## Adjusting Parameters

| Parameter | File | Description |
|-----------|------|-------------|
| Body mass | `robot/kinematics/force_calculator.py` -> `M_BODY` | Total frame mass [kg] |
| Ballscrew lead | `robot/kinematics/force_calculator.py` -> `BALLSCREW_LEAD` | m/rev |
| Lever arm | `robot/kinematics/leg_kinematics.py` -> `R_ARM` | Pivot-to-nut distance [m] |
| Leg lengths | `robot/kinematics/leg_kinematics.py` -> `L1`, `L2` | Thigh / shank [m] |
| Step length | `robot/gait/walk_gait.py` -> `STEP_LENGTH` | Stride [m] |
| Step height | `robot/gait/walk_gait.py` -> `STEP_HEIGHT` | Foot lift [m] |
| Carriage drag | `robot/kinematics/force_calculator.py` -> `CARRIAGE_DRAG_N` | Currently 0 N |
