# Robotic Horse — ROS2 + Gazebo Full Tutorial

This document covers everything you need to know to install, build, and run the Robotic Horse simulation using **ROS2 Humble** and **Gazebo Classic 11** on Ubuntu 22.04.

---

## Table of Contents

1. [What is ROS2?](#1-what-is-ros2)
2. [What is Gazebo?](#2-what-is-gazebo)
3. [How They Work Together](#3-how-they-work-together)
4. [System Requirements](#4-system-requirements)
5. [Setting Up Ubuntu 22.04 (WSL2 on Windows)](#5-setting-up-ubuntu-2204-wsl2-on-windows)
6. [Installing ROS2 Humble](#6-installing-ros2-humble)
7. [Installing Gazebo Classic 11](#7-installing-gazebo-classic-11)
8. [Installing Required ROS2 Packages](#8-installing-required-ros2-packages)
9. [Understanding the Workspace Structure](#9-understanding-the-workspace-structure)
10. [Building the Workspace](#10-building-the-workspace)
11. [Running the Simulation — Step by Step](#11-running-the-simulation--step-by-step)
12. [Understanding Each File](#12-understanding-each-file)
13. [Monitoring Ballscrew Forces Live](#13-monitoring-ballscrew-forces-live)
14. [Tuning PID Gains](#14-tuning-pid-gains)
15. [Adjusting Robot Parameters](#15-adjusting-robot-parameters)
16. [Troubleshooting](#16-troubleshooting)

---

## 1. What is ROS2?

**ROS2 (Robot Operating System 2)** is the industry-standard middleware for robotics software. Despite the name, it is not an operating system — it is a framework that provides:

- **Nodes** — individual programs that each do one job (e.g. "run the gait planner", "read joint sensors")
- **Topics** — named message channels that nodes publish to and subscribe from (e.g. `/joint_states`)
- **Launch files** — Python scripts that start multiple nodes at once, wiring them together
- **Build system (colcon + ament)** — compiles all packages in your workspace

In this project, ROS2 connects:
- The **gait planner node** (outputs joint angle targets)
- The **joint trajectory controller** (sends those targets to Gazebo)
- The **force reporter node** (reads back joint states, computes ballscrew forces)

---

## 2. What is Gazebo?

**Gazebo** is a 3D physics simulator used to test robots before building them in hardware. It simulates:

- Rigid body physics (gravity, inertia, collisions)
- Joint actuators with PID controllers
- Sensor simulation (cameras, LiDAR, IMUs)
- Terrain and environments

In this project Gazebo:
- Loads the robot URDF (the 3D model with physics properties)
- Simulates each joint being driven by a PID position controller
- Reports back joint positions and effort through ROS2 topics

---

## 3. How They Work Together

```
┌─────────────────────────────────────────────────┐
│                    ROS2 Graph                   │
│                                                 │
│  gait_node ──────────────────────────────────▶  │
│  publishes JointTrajectory                      │
│         │                                       │
│         ▼                                       │
│  leg_controller (ros2_control)                  │
│  drives joints via PID                          │
│         │                                       │
│         ▼                                       │
│  ┌─────────────────────┐                        │
│  │      GAZEBO          │  ← physics simulation │
│  │  robot is simulated  │                        │
│  └─────────────────────┘                        │
│         │                                       │
│         ▼                                       │
│  joint_state_broadcaster                        │
│  publishes /joint_states                        │
│         │                                       │
│         ▼                                       │
│  force_node ── computes ballscrew forces        │
│  publishes /robotic_horse/ballscrew_forces      │
└─────────────────────────────────────────────────┘
```

---

## 4. System Requirements

| Requirement | Version |
|-------------|---------|
| Operating System | Ubuntu 22.04 LTS ("Jammy") |
| ROS2 | Humble Hawksbill (LTS) |
| Gazebo | Classic 11 |
| Python | 3.10 (bundled with Ubuntu 22.04) |
| RAM | ≥ 8 GB recommended |
| GPU | Recommended for Gazebo GUI; not required |

> **Windows users:** Use **WSL2** (Windows Subsystem for Linux) with Ubuntu 22.04.
> Full GPU passthrough for the Gazebo GUI requires WSL2 with WSLg.

---

## 5. Setting Up Ubuntu 22.04 (WSL2 on Windows)

Open PowerShell as Administrator and run:

```powershell
wsl --install -d Ubuntu-22.04
```

Restart your computer when prompted. Then open **Ubuntu 22.04** from the Start Menu and create a username and password.

Enable WSLg (GUI support — for Gazebo window):

```powershell
# In PowerShell (admin)
wsl --update
wsl --shutdown
```

Reopen Ubuntu. Test GUI works:

```bash
sudo apt update && sudo apt install -y x11-apps
xclock   # a clock window should appear
```

Clone the repository into your WSL home directory:

```bash
cd ~
git clone https://github.com/DanTheLion-git/Robotic_Horse.git
```

---

## 6. Installing ROS2 Humble

Follow these steps exactly. Each command is explained.

### 6.1 Set up locale

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 6.2 Add the ROS2 apt repository

```bash
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu \
    $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 6.3 Install ROS2 Humble (desktop full)

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop
```

This installs ROS2 core, RViz2, and common tools (~2 GB, takes a few minutes).

### 6.4 Source ROS2 in every terminal

Add to your `~/.bashrc` so you never have to do it manually:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 6.5 Verify installation

```bash
ros2 --version        # should print "ros2 cli interface version X.X.X"
ros2 topic list       # should print /parameter_events and /rosout
```

---

## 7. Installing Gazebo Classic 11

```bash
sudo apt install -y gazebo
```

Verify:

```bash
gazebo --version    # should print "Gazebo multi-robot simulator, version 11.x.x"
```

---

## 8. Installing Required ROS2 Packages

These bridge ROS2 with Gazebo and provide the joint controllers:

```bash
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    python3-colcon-common-extensions
```

---

## 9. Understanding the Workspace Structure

```
~/Robotic_Horse/
├── ros2_ws/                          ← ROS2 workspace root
│   └── src/                          ← source packages go here
│       ├── robotic_horse_description/  ← URDF + RViz launch
│       │   ├── package.xml             (package metadata)
│       │   ├── CMakeLists.txt          (ament_cmake build rules)
│       │   ├── urdf/
│       │   │   └── robotic_horse.urdf  (Gazebo-compatible robot model)
│       │   └── launch/
│       │       └── display.launch.py   (RViz2 viewer, no Gazebo)
│       │
│       └── robotic_horse_control/      ← nodes + launch + config
│           ├── package.xml
│           ├── setup.py                (ament_python build rules)
│           ├── robotic_horse_control/  ← Python package
│           │   ├── gait_node.py        (trot gait → joint trajectories)
│           │   └── force_node.py       (joint states → ballscrew forces)
│           ├── launch/
│           │   ├── gazebo.launch.py        (start Gazebo + spawn robot)
│           │   └── robot_control.launch.py (start gait + force nodes)
│           ├── config/
│           │   └── ros2_control.yaml   (controller config + PID gains)
│           └── worlds/
│               └── robotic_horse.world (flat terrain SDF)
│
├── robot/                  ← shared Python kinematics (used by pybullet/)
├── pybullet/               ← PyBullet prototype simulation
├── gazebo/                 ← this tutorial file
└── main.py                 ← standalone force analysis (no ROS2 needed)
```

**Two package types:**

| Package | Build type | Purpose |
|---------|-----------|---------|
| `robotic_horse_description` | `ament_cmake` | Install URDF/launch files into the ROS2 share directory |
| `robotic_horse_control` | `ament_python` | Install Python nodes as executable commands |

---

## 10. Building the Workspace

### 10.1 Navigate to the workspace

```bash
cd ~/Robotic_Horse/ros2_ws
```

### 10.2 Install package dependencies automatically

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

`rosdep` reads `package.xml` files and installs any missing apt/pip dependencies.

### 10.3 Build with colcon

```bash
colcon build --symlink-install
```

- `colcon` is the ROS2 build tool. It discovers all packages in `src/` and builds them.
- `--symlink-install` means Python files are symlinked rather than copied — changes to node source code take effect immediately without rebuilding.

Expect output like:
```
Starting >>> robotic_horse_description
Starting >>> robotic_horse_control
Finished <<< robotic_horse_description [1.5s]
Finished <<< robotic_horse_control [2.1s]

Summary: 2 packages finished
```

### 10.4 Source the workspace overlay

```bash
source install/setup.bash
```

Add to `~/.bashrc` so you never forget:

```bash
echo "source ~/Robotic_Horse/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 11. Running the Simulation — Step by Step

You will need **three terminal windows** (all inside the `ros2_ws` directory, with `setup.bash` sourced).

### Terminal 1 — Launch Gazebo with the robot

```bash
cd ~/Robotic_Horse/ros2_ws
source install/setup.bash
ros2 launch robotic_horse_control gazebo.launch.py
```

What happens:
1. Gazebo opens with the flat terrain world
2. The robot URDF is published to `/robot_description`
3. The robot model is spawned at position (0, 0, 1.15 m)
4. `joint_state_broadcaster` starts (feeds `/joint_states`)
5. `leg_controller` (JointTrajectoryController) starts

Wait until you see:
```
[INFO] [spawner]: Loaded joint_state_broadcaster
[INFO] [spawner]: Loaded leg_controller
```

### Terminal 2 — Start gait and force nodes

```bash
cd ~/Robotic_Horse/ros2_ws
source install/setup.bash
ros2 launch robotic_horse_control robot_control.launch.py
```

The robot will start walking. You will see log output from the gait node publishing trajectories.

### Terminal 3 — Monitor topics (optional but useful)

```bash
# Watch joint states (positions of all 8 joints, 100 Hz)
ros2 topic echo /joint_states

# Watch computed ballscrew forces [N] for FL, FR, RL, RR
ros2 topic echo /robotic_horse/ballscrew_forces

# Watch required motor torques [N·m] for FL, FR, RL, RR
ros2 topic echo /robotic_horse/motor_torques

# Check what topics exist
ros2 topic list

# See message rate of a topic
ros2 topic hz /joint_states
```

### Inspect the robot in RViz2 (without Gazebo)

```bash
ros2 launch robotic_horse_description display.launch.py
```

A joint slider GUI opens alongside RViz2 — drag the sliders to manually pose the robot and verify the URDF looks correct.

---

## 12. Understanding Each File

### `urdf/robotic_horse.urdf`

The robot model. Key sections:

| Section | Purpose |
|---------|---------|
| `<link>` | Defines a rigid body: mass, inertia, visual geometry, collision geometry |
| `<joint>` | Connects two links: position, axis, limits, damping |
| `<gazebo reference="...">` | Adds Gazebo-specific properties (material colour, friction) |
| `<ros2_control>` | Declares which joints are controllable and what interfaces they expose |
| `<gazebo><plugin ...>` | Loads `libgazebo_ros2_control.so` — the bridge between ros2_control and Gazebo physics |

### `config/ros2_control.yaml`

Tells `controller_manager` what controllers to load and how to configure them:

- `joint_state_broadcaster` — reads joint data from simulation hardware, publishes to `/joint_states`
- `leg_controller` — receives `JointTrajectory` messages and drives 8 joints to target positions using PID

### `launch/gazebo.launch.py`

Orchestrates the full startup sequence:
1. Start Gazebo with our world file
2. Start `robot_state_publisher` (TF tree from URDF)
3. Spawn the robot entity into Gazebo
4. Wait for robot to spawn, then activate controllers

### `gait_node.py`

Runs the trot gait planner:
- Computes swing/stance trajectories using 2-link inverse kinematics
- Publishes a `JointTrajectory` message every half-cycle (0.5 s)
- Each message contains 50 waypoints covering the next full cycle — the controller interpolates smoothly between them

### `force_node.py`

Real-time force monitor:
- Subscribes to `/joint_states` at 100 Hz
- For each leg: reads `theta_thigh` and `theta_knee`, computes `F_nut` and `τ_motor`
- Publishes arrays to `/robotic_horse/ballscrew_forces` and `/robotic_horse/motor_torques`
- This lets you graph forces live in RViz2 or `rqt_plot`

---

## 13. Monitoring Ballscrew Forces Live

Use `rqt_plot` for a live graph:

```bash
# Install rqt tools
sudo apt install -y ros-humble-rqt ros-humble-rqt-common-plugins

# Launch the plot tool
rqt_plot /robotic_horse/ballscrew_forces/data[0]:data[1]:data[2]:data[3]
```

This plots FL / FR / RL / RR ballscrew forces in real time as the robot walks.

---

## 14. Tuning PID Gains

The joint controllers use PID. Default gains are in `config/ros2_control.yaml`:

```yaml
fl_knee_joint: { p: 400.0, i: 0.0, d: 20.0 }
```

**Signs of wrong gains:**

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Legs oscillate / vibrate | P too high or D too low | Reduce P, increase D |
| Legs sag and respond slowly | P too low | Increase P |
| Joints overshoot target | D too low | Increase D |
| Slow drift from target | Steady-state error | Add small I term (0.1–1.0) |

After editing `ros2_control.yaml`, rebuild and relaunch:

```bash
colcon build --symlink-install --packages-select robotic_horse_control
ros2 launch robotic_horse_control gazebo.launch.py
```

---

## 15. Adjusting Robot Parameters

All physical values are in two files. **Edit, rebuild, and relaunch to apply.**

### Leg geometry — `gait_node.py` and `force_node.py`

```python
L1 = 0.45       # thigh length [m]
L2 = 0.50       # shank length [m]
BODY_HEIGHT = 0.90  # nominal ground clearance [m]
```

### Gait tuning — `gait_node.py`

```python
STEP_LENGTH = 0.20   # stride length [m]  — longer = faster walk
STEP_HEIGHT = 0.12   # foot lift [m]      — higher = clears more obstacles
SWING_FRAC  = 0.40   # fraction of cycle in swing (0.3–0.5 typical)
```

### Ballscrew — `force_node.py`

```python
R_ARM          = 0.08    # lever arm from pivot to nut connection [m]
BALLSCREW_LEAD = 0.005   # screw lead [m/rev]  — 5 mm/rev default
BALLSCREW_EFF  = 0.90    # mechanical efficiency
```

### Body mass — `force_node.py`

```python
M_BODY = 20.0    # body frame mass [kg]
```

---

## 16. Troubleshooting

### `gazebo: command not found`
```bash
sudo apt install -y gazebo
```

### `No module named 'rclpy'`
You forgot to source the ROS2 setup:
```bash
source /opt/ros/humble/setup.bash
source ~/Robotic_Horse/ros2_ws/install/setup.bash
```

### `libgazebo_ros2_control.so: cannot open shared object file`
```bash
sudo apt install -y ros-humble-gazebo-ros2-control
```

### Robot falls through the ground
Check the spawn height in `gazebo.launch.py` (`-z 1.15`). Also verify inertia values in the URDF are non-zero.

### Controller not activating
Check the terminal for errors from the `spawner` nodes. Most common cause is a mismatch between joint names in the URDF `<ros2_control>` block and in `ros2_control.yaml`.

### Gait looks wrong / robot bounces
Reduce `STEP_HEIGHT` or increase PID gains. Also try slowing the gait by increasing `_gait_period` in `gait_node.py`.

### WSL2 — Gazebo window doesn't open
```bash
echo $DISPLAY          # should print :0 or similar
sudo apt install -y mesa-utils
glxinfo | grep "OpenGL renderer"   # should show a GPU
```
If no GPU is listed, update WSL2:
```powershell
wsl --update
```
