# INSTRUCTIONS — Running the Force Analysis

## Prerequisites

- **Python 3.11 or 3.12** recommended (PyBullet has no prebuilt wheel for Python 3.14)
- **Git** (to clone the repository)

---

## 1. Clone the Repository

```bash
git clone https://github.com/DanTheLion-git/Robotic_Horse.git
cd Robotic_Horse
```

---

## 2. (Recommended) Create a Virtual Environment

```bash
python -m venv .venv
```

Activate it:

- **Windows:** `.venv\Scripts\activate`
- **macOS / Linux:** `source .venv/bin/activate`

---

## 3. Install Dependencies

```bash
pip install -r requirements.txt
```

> **Note:** If you are on Python 3.14 and `pybullet` fails to install,
> the force analysis still works — only the live 3D simulation requires PyBullet.
> Install the remaining packages manually if needed:
> ```bash
> pip install numpy scipy matplotlib
> ```

---

## 4. Run the Force Analysis

```bash
python main.py --analysis
```

This will:
- Generate one full trot gait cycle trajectory for all four legs
- Print a table of **peak and mean ballscrew forces** and **motor torques** per leg
- Save a plot to `force_analysis.png`

### Skip the plot window (headless / CI)

```bash
python main.py --analysis --no-plots
```

---

## 5. (Optional) Run the Full PyBullet Simulation

Requires PyBullet to be installed (Python ≤ 3.12 or MSVC Build Tools installed).

```bash
# With 3D GUI window
python main.py

# Headless (no window)
python main.py --no-gui
```

---

## Understanding the Output

```
============================================================
  KINEMATIC FORCE ANALYSIS — one gait cycle
  Body mass: 20.0 kg    Gravity: 9.81 m/s²
============================================================
  Leg    Max F_bs [N]  Mean F_bs [N]  Max τ_motor [N·m]
  ------------------------------------------------------
  FL           1082.6         901.7              0.957
  ...
============================================================
```

| Column | Description |
|--------|-------------|
| `Max F_bs [N]` | Peak linear force the ballscrew nut must exert |
| `Mean F_bs [N]` | Average force across the gait cycle |
| `Max τ_motor [N·m]` | Required motor shaft torque (accounts for screw lead & efficiency) |

---

## Adjusting Parameters

All physical parameters are at the top of their respective files:

| Parameter | File | Description |
|-----------|------|-------------|
| Body mass | `robot/kinematics/force_calculator.py` | `M_BODY` (kg) |
| Ballscrew lead | `robot/kinematics/force_calculator.py` | `BALLSCREW_LEAD` (m/rev) |
| Ballscrew efficiency | `robot/kinematics/force_calculator.py` | `BALLSCREW_EFF` (0–1) |
| Leg segment lengths | `robot/kinematics/leg_kinematics.py` | `L1`, `L2` (m) |
| Step length / height | `robot/gait/walk_gait.py` | `STEP_LENGTH`, `STEP_HEIGHT` (m) |
| Body height | `robot/gait/walk_gait.py` | `BODY_HEIGHT` (m) |
