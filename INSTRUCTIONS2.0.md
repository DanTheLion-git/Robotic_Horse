# Robotic Horse — Simulation Guide v2.0

**Current stack: Ubuntu 24.04 · ROS2 Jazzy · Gazebo Harmonic**

This document supersedes `gazebo/INSTRUCTIONS.md` (which was written for the older
ROS2 Humble + Gazebo Classic 11 stack). Everything here reflects the working
setup on this machine.

---

## Table of Contents

1. [Why `ros2 launch` fails in a new terminal — and the fix](#1-why-ros2-launch-fails-in-a-new-terminal)
2. [Starting the simulation](#2-starting-the-simulation)
3. [Stopping the simulation](#3-stopping-the-simulation)
4. [What you should see](#4-what-you-should-see)
5. [Adjusting simulation values](#5-adjusting-simulation-values)
6. [How ROS2 works in this project](#6-how-ros2-works-in-this-project)
7. [How Gazebo works in this project](#7-how-gazebo-works-in-this-project)
8. [How they connect — the full data flow](#8-how-they-connect--the-full-data-flow)
9. [Monitoring topics live](#9-monitoring-topics-live)
10. [Rebuilding after a change](#10-rebuilding-after-a-change)
11. [Troubleshooting](#11-troubleshooting)

---

## 1. Why `ros2 launch` fails in a new terminal

When you open a fresh terminal, your shell only knows about standard Linux
commands. `ros2` and the `robotic_horse_control` package are installed in
separate directories that your shell does not know about yet.

You need to run two `source` commands to register them:

```bash
# 1. Add ROS2 Jazzy to PATH and library paths
source /opt/ros/jazzy/setup.bash

# 2. Add THIS workspace's packages (robotic_horse_control etc.)
source ~/Robotic-Horse/ros2_ws/install/setup.bash
```

After those two lines, `ros2 launch robotic_horse_control full_sim.launch.py` works.

### Permanent fix — add to `~/.bashrc`

Instead of sourcing manually every time, add both lines to your shell startup
file so every new terminal automatically knows about ROS2:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/Robotic-Horse/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

After that, any new terminal is ready immediately.

---

## 2. Starting the simulation

### Option A — Everything in one command (recommended)

```bash
# If you have NOT added the source lines to ~/.bashrc yet:
source /opt/ros/jazzy/setup.bash
source ~/Robotic-Horse/ros2_ws/install/setup.bash

# Then launch everything:
ros2 launch robotic_horse_control full_sim.launch.py
```

This one command starts in sequence:
| Time | What starts |
|------|-------------|
| T+0 s | Gazebo Harmonic opens with the world + robot |
| T+4 s | RViz2 opens with robot model, TF frames, and force arrows |
| T+9 s | Gait node begins publishing trot trajectories |
| T+9 s | Force node begins computing and publishing ballscrew forces |

Wait about 15 seconds total before the robot starts walking.

### Option B — Gazebo only (no gait, no RViz2)

Useful for inspecting the robot or testing controller behaviour manually:

```bash
ros2 launch robotic_horse_control gazebo.launch.py
```

### Option C — PyBullet (no ROS2 needed)

A lightweight prototype simulation with no GUI requirement:

```bash
cd ~/Robotic-Horse
source .venv/bin/activate
python pybullet/run_pybullet.py --no-gui   # headless
python pybullet/run_pybullet.py             # with GUI
```

---

## 3. Stopping the simulation

Press **Ctrl + C** in the terminal where you ran `ros2 launch`. This sends a
shutdown signal to all nodes and closes Gazebo and RViz2.

If the windows do not close, or if you see errors about duplicate nodes on
the next launch, kill stale processes manually:

```bash
# Find and kill any leftover Gazebo processes
ps aux | grep "gz sim" | grep -v grep
# Then kill by PID (replace 12345 with the actual number):
kill 12345

# Or kill all stale ROS2 nodes at once:
ros2 daemon stop
ros2 daemon start
```

> **Important:** Never use `pkill` or `killall` — always use `kill <PID>`.
> Running multiple launches without killing the previous ones creates duplicate
> `/controller_manager` nodes which breaks controller activation.

---

## 4. What you should see

**Gazebo window**
- Dark-grey horse body with silver saddle, front neck stub
- Orange thighs with silver ballscrew tubes
- Black rubber feet
- Brown wooden wagon (behind the horse) with 4 dark wheels
- Two wooden shaft poles running from the wagon to the horse collar
- The horse is held upright by the shafts — it can turn left/right (yaw)
  but cannot pitch or roll

**RViz2 window**
- Grid (ground reference)
- Robot model with live joint angles
- TF axis frames on every link
- Coloured force arrows at each knee: green = low load → red = high load
- Force labels showing "FL: 234 N" etc., updated live

---

## 5. Adjusting simulation values

### 5.1 Gait parameters

**File:** `ros2_ws/src/robotic_horse_control/robotic_horse_control/gait_node.py`

```python
L1 = 0.45          # thigh link length [m]
L2 = 0.50          # shank link length [m]
BODY_HEIGHT = 0.85  # hip-to-ground target during stance [m]
                    # lower = more crouched, higher = more extended
                    # must be < L1+L2 = 0.95

STEP_LENGTH = 0.20  # how far each foot steps forward per stride [m]
                    # increase for faster walking, decrease for stability
STEP_HEIGHT = 0.12  # how high the foot lifts off the ground [m]
                    # increase to clear obstacles
SWING_FRAC  = 0.40  # fraction of gait cycle spent in swing (0.3–0.5)
                    # lower = shorter swing, longer stance (more stable)
```

Change `_gait_period` (inside the `GaitNode.__init__` method) to slow/speed the walk:
```python
self._gait_period = 1.0    # seconds per full gait cycle
                            # increase to slow down, decrease to speed up
```

No rebuild needed — changes take effect on next launch because
`--symlink-install` symlinks the source file directly.

### 5.2 PID gains (joint stiffness / response)

**File:** `ros2_ws/src/robotic_horse_control/config/ros2_control.yaml`

```yaml
gains:
  fl_thigh_joint: { p: 200.0, i: 0.0, d: 10.0 }
  fl_knee_joint:  { p: 400.0, i: 0.0, d: 20.0 }
  # ... same pattern for fr, rl, rr
```

| Symptom | Fix |
|---------|-----|
| Legs oscillate / shake | Reduce `p`, increase `d` |
| Legs sag, track slowly | Increase `p` |
| Joints overshoot target | Increase `d` |
| Slow drift from target | Add small `i` (try 0.5–2.0) |

No rebuild needed for YAML config changes.

### 5.3 Initial leg pose (default standing position)

**File:** `ros2_ws/src/robotic_horse_description/urdf/robotic_horse.urdf.xacro`

In the `<ros2_control>` block, each thigh and knee joint has an `initial_value`:

```xml
<state_interface name="position">
  <param name="initial_value">0.3</param>   <!-- thigh: 0.3 rad ≈ 17° forward -->
</state_interface>
```
```xml
<state_interface name="position">
  <param name="initial_value">-0.6</param>  <!-- knee: -0.6 rad ≈ 34° bent -->
</state_interface>
```

Increasing the thigh value makes the legs lean more forward.
Making knee more negative bends the knee further.
After changing, rebuild:
```bash
cd ~/Robotic-Horse/ros2_ws
colcon build --symlink-install
```

### 5.4 Joint movement limits

**File:** `ros2_ws/src/robotic_horse_description/urdf/robotic_horse.urdf.xacro`

Find the joint definitions (e.g. `fl_thigh_joint`):

```xml
<limit lower="-0.5" upper="1.2" effort="500" velocity="3.0"/>
```

| Attribute | Meaning |
|-----------|---------|
| `lower` | Minimum angle [rad]. Negative = leg leans backward |
| `upper` | Maximum angle [rad]. 1.2 rad ≈ 69° forward |
| `effort` | Maximum joint torque [N·m] |
| `velocity` | Maximum joint speed [rad/s] |

Rebuild required after changes.

### 5.5 Ballscrew and force parameters

**File:** `ros2_ws/src/robotic_horse_control/robotic_horse_control/force_node.py`

```python
R_ARM          = 0.08    # lever arm: pivot-to-nut distance [m]
                          # this is a mechanical property of your design
BALLSCREW_LEAD = 0.005   # 5 mm/rev — distance nut moves per motor revolution
BALLSCREW_EFF  = 0.90    # 90% efficiency (friction losses)
M_BODY         = 20.0    # body + payload mass [kg]
FORCE_MAX      = 1500.0  # force scale for RViz2 colour map [N]
                          # arrows turn fully red at this value
```

No rebuild needed (symlinked).

### 5.6 Wagon / horse attachment

**File:** `ros2_ws/src/robotic_horse_description/urdf/robotic_horse.urdf.xacro`

The `shaft_to_horse` joint controls how much the horse can steer:

```xml
<joint name="shaft_to_horse" type="revolute">
  <limit lower="-0.52" upper="0.52" .../>   <!-- ±0.52 rad = ±30° yaw -->
  <dynamics damping="20.0" friction="5.0"/>  <!-- resistance to turning -->
</joint>
```

Increase `damping` if the horse swings too freely. Increase the limit values
to allow sharper turns. Rebuild required.

The cart position relative to the horse:

```xml
<joint name="world_to_cart" type="fixed">
  <origin xyz="-2.50 0 0.55" .../>   <!-- cart is 2.5 m behind the horse -->
</joint>
```

Change `-2.50` to move the cart closer or further. Rebuild required.

### 5.7 World / environment

**File:** `ros2_ws/src/robotic_horse_control/worlds/robotic_horse.world`

This is an SDF file. You can add slopes, obstacles, or change ground friction.
After editing, the world reloads on the next launch (no rebuild needed).

---

## 6. How ROS2 works in this project

ROS2 is the **communication backbone** between all programs.

### Core concepts

**Node** — a single running program with one job.
This project has three nodes:

| Node | Job |
|------|-----|
| `robot_state_publisher` | Reads the URDF, computes TF transforms, publishes them |
| `gait_node` | Computes trot gait IK, publishes joint angle targets |
| `force_node` | Reads joint angles, computes ballscrew forces, publishes results |

Plus two nodes provided by `ros2_control`:

| Node | Job |
|------|-----|
| `controller_manager` | Manages the lifecycle of all controllers |
| `joint_state_broadcaster` | Reads joint positions from Gazebo, publishes `/joint_states` |

**Topic** — a named data stream. Any node can publish to or subscribe from a topic.

Key topics in this project:

| Topic | Type | Who publishes | Who reads |
|-------|------|---------------|-----------|
| `/robot_description` | `std_msgs/String` | `robot_state_publisher` | Gazebo spawn, RViz2 |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` | `force_node`, RViz2 |
| `/tf` | `tf2_msgs/TFMessage` | `robot_state_publisher` | RViz2 |
| `/leg_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | `gait_node` | `leg_controller` |
| `/robotic_horse/ballscrew_forces` | `std_msgs/Float64MultiArray` | `force_node` | You / rqt_plot |
| `/robotic_horse/motor_torques` | `std_msgs/Float64MultiArray` | `force_node` | You / rqt_plot |
| `/robotic_horse/force_markers` | `visualization_msgs/MarkerArray` | `force_node` | RViz2 |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo bridge | All nodes (sim time) |

**Launch file** — a Python script that starts multiple nodes at once.
`full_sim.launch.py` starts everything with timed delays so Gazebo is ready
before the controllers try to connect, and the gait starts after controllers
are active.

### Useful ROS2 commands

```bash
# List all running nodes
ros2 node list

# List all active topics
ros2 topic list

# Print messages from a topic in real time
ros2 topic echo /joint_states

# Check publish rate of a topic
ros2 topic hz /leg_controller/joint_trajectory

# Inspect the structure of a message type
ros2 interface show trajectory_msgs/msg/JointTrajectory

# List all loaded controllers
ros2 control list_controllers

# List all registered hardware interfaces
ros2 control list_hardware_interfaces
```

---

## 7. How Gazebo works in this project

Gazebo Harmonic is the **physics engine**. It simulates every rigid body,
joint, gravity, and contact force.

### What Gazebo loads

1. **The world file** (`robotic_horse.world`) — defines the ground plane,
   lighting, and physics settings (gravity, step size).

2. **The URDF** (`robotic_horse.urdf.xacro`) — describes:
   - Every link: geometry (box, cylinder, sphere), mass, inertia
   - Every joint: parent/child links, axis, limits, damping
   - The `<ros2_control>` block that tells Gazebo which joints to expose
     to the ROS2 controller system
   - The `<gazebo><plugin>` block that loads the `gz_ros2_control` bridge

3. **The gz_ros2_control plugin** — connects Gazebo's joint simulation to
   the ROS2 `controller_manager`. Once loaded:
   - `leg_controller` can command joint positions
   - `joint_state_broadcaster` can read joint positions back

### The joint control loop

```
gait_node publishes JointTrajectory
           │
           ▼
   leg_controller (JointTrajectoryController)
   interpolates the trajectory at 100 Hz
           │  position command
           ▼
   gz_ros2_control plugin
   applies force via PID:  effort = P*(target - actual) + D*(d_error/dt)
           │
           ▼
   Gazebo physics steps the simulation
   (joints move, collisions detected, gravity applied)
           │  actual joint position
           ▼
   gz_ros2_control reads back position/velocity/effort
           │
           ▼
   joint_state_broadcaster publishes /joint_states
```

### Key URDF concepts

The robot tree is:
```
world (fixed anchor — prevents horse from falling over)
 └── cart_base (wagon body, fixed to world)
       ├── cart_wheel_fl/fr/rl/rr (spin freely on contact with ground)
       └── shaft_assembly (two shaft poles, fixed to cart)
             └── base_link (horse body — REVOLUTE Z joint here)
                           ← horse can yaw ±30° but cannot pitch or roll
                   ├── fl_hip_link → fl_thigh_link → fl_shank_link → fl_foot_link
                   ├── fr_hip_link → fr_thigh_link → fr_shank_link → fr_foot_link
                   ├── rl_hip_link → rl_thigh_link → rl_shank_link → rl_foot_link
                   └── rr_hip_link → rr_thigh_link → rr_shank_link → rr_foot_link
```

The **8 controlled joints** (position-controlled via PID):
- `{fl,fr,rl,rr}_thigh_joint` — hip flexion/extension
- `{fl,fr,rl,rr}_knee_joint` — knee bend (always ≤ 0, bends backward)

The **passive joints** (simulated by physics, not controlled):
- `shaft_to_horse` — yaw joint, horse steers freely within ±30°
- 4 wheel joints — spin with ground friction

---

## 8. How they connect — the full data flow

```
┌────────────────────────────────────────────────────────────┐
│                    ROS2 Node Graph                         │
│                                                            │
│  robot_state_publisher                                     │
│   reads URDF → publishes /tf (all link frames)             │
│   publishes /robot_description (for RViz2 model display)   │
│                                                            │
│  gait_node                                                 │
│   every 0.5 s: computes 50-point trot trajectory via IK    │
│   → publishes /leg_controller/joint_trajectory             │
│                                                            │
│  leg_controller (in controller_manager)                    │
│   receives trajectory → interpolates at 100 Hz             │
│   → sends position commands to gz_ros2_control             │
│                                                            │
│  ┌─────────────────────────────────────────┐               │
│  │         GAZEBO HARMONIC                 │               │
│  │  Simulates physics at 1000 Hz           │               │
│  │  gz_ros2_control bridge:                │               │
│  │   • receives position commands          │               │
│  │   • applies PID effort to joints        │               │
│  │   • reads back position/velocity/effort │               │
│  └─────────────────────────────────────────┘               │
│                                                            │
│  joint_state_broadcaster                                   │
│   reads joint hardware states → publishes /joint_states    │
│                                                            │
│  force_node                                                │
│   reads /joint_states → FK + ballscrew geometry            │
│   → publishes /robotic_horse/ballscrew_forces  [4 floats]  │
│   → publishes /robotic_horse/motor_torques     [4 floats]  │
│   → publishes /robotic_horse/force_markers     [MarkerArray]│
│                                                            │
│  RViz2                                                     │
│   subscribes: /tf, /robot_description,                     │
│               /robotic_horse/force_markers                 │
│   displays: robot model, TF frames, force arrows           │
└────────────────────────────────────────────────────────────┘
```

---

## 9. Monitoring topics live

```bash
# Watch joint positions (all 8 joints + yaw joint, at 100 Hz)
ros2 topic echo /joint_states

# Watch computed ballscrew forces [N]  — 4 values: FL, FR, RL, RR
ros2 topic echo /robotic_horse/ballscrew_forces

# Watch required motor torques [N·m]
ros2 topic echo /robotic_horse/motor_torques

# Plot forces in real time (install rqt first if not present)
sudo apt install -y ros-jazzy-rqt ros-jazzy-rqt-common-plugins
rqt_plot /robotic_horse/ballscrew_forces/data[0]:data[1]:data[2]:data[3]

# Check all controllers are active
ros2 control list_controllers

# See which hardware interfaces are registered (8 joints × 3 interfaces)
ros2 control list_hardware_interfaces
```

---

## 10. Rebuilding after a change

Most Python files and YAML configs use `--symlink-install`, meaning edits
take effect immediately on the next launch — **no rebuild needed**.

You MUST rebuild when you change:
- The URDF / xacro file (robot shape, joints, limits, initial values)
- `CMakeLists.txt` or `package.xml`
- Any new Python file added to a package

```bash
cd ~/Robotic-Horse/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Re-source after rebuilding so the new install is on your PATH.

---

## 11. Troubleshooting

### `ros2: command not found`
```bash
source /opt/ros/jazzy/setup.bash
```
Or add it permanently to `~/.bashrc` (see Section 1).

### `Package 'robotic_horse_control' not found`
```bash
source ~/Robotic-Horse/ros2_ws/install/setup.bash
```
You need BOTH source commands — the ROS2 one and the workspace one.

### Gazebo opens but robot immediately falls over
The cart-shaft constraint should prevent this. If it still happens:
1. Check for stale `gz sim` processes (`ps aux | grep "gz sim"`) and kill them
2. Make sure you only have ONE terminal running the launch

### Controllers fail to activate / `list_hardware_interfaces` returns empty
Multiple launches are running simultaneously. Kill all `gz sim` processes,
then relaunch from a clean terminal.

### Robot spawns but legs are in a strange pose
The `initial_value` parameters in the URDF set the starting pose.
Check Section 5.3. Make sure you rebuilt after changing the URDF.

### RViz2 shows "No transform from [base_link] to [world]"
The fixed frame should be `world` (updated in the RViz config).
If it shows `base_link`, click the Fixed Frame dropdown in RViz2 and select `world`.

### Force arrows not visible in RViz2
The force node starts at T+9 s. Wait a few more seconds after the gait begins.
Also confirm the `force_node` is running: `ros2 node list` should show `/force_node`.

### `rqt_plot` shows no data
Make sure both `full_sim.launch.py` is running AND you waited for the gait to start
(T+9 s). The force topic only publishes once joint states are available.

### gait looks jerky or joints oscillate
Reduce PID `p` gains in `ros2_control.yaml` (try halving them). Also try
increasing `_gait_period` in `gait_node.py` to slow the walk down.

### `colcon build` fails
Check the error message. Most common causes:
- Missing `source /opt/ros/jazzy/setup.bash` before building
- XML syntax error in the URDF (check with `check_urdf /tmp/test.urdf`)
  ```bash
  xacro ros2_ws/src/robotic_horse_description/urdf/robotic_horse.urdf.xacro \
    > /tmp/test.urdf && check_urdf /tmp/test.urdf
  ```


---

## Section 8 -- Blendspace Control System

The simulation now uses a velocity-driven blendspace controller instead of
a fixed gait loop.  Publish geometry_msgs/Twist to /cmd_vel to control movement.

  linear.x  = forward/backward speed (m/s, positive = forward)
  angular.z = turn rate (rad/s, positive = turn left / CCW)

### States

  |linear.x| < 0.01  -> IDLE: holds Spot neutral pose
  linear.x > 0.01    -> FORWARD trot gait
  linear.x < -0.01   -> BACKWARD trot gait
  angular.z != 0     -> TURNING: differential stride + shoulder yaw

### Keyboard Teleop (second terminal)

  source /opt/ros/jazzy/setup.bash
  source ~/Robotic-Horse/ros2_ws/install/setup.bash
  ros2 run robotic_horse_control teleop_key

  W = Forward   S = Backward   A = Left   D = Right   SPACE = Stop
  Key release automatically publishes zero twist (safe stop).

### Manual publishing

  ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"
  ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.4}, angular: {z: 0.6}}"
  ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{}"

### Blendspace tuning (blendspace_node.py)

  MAX_SPEED        = 0.5    # linear.x mapping to full stride
  STEP_LENGTH_BASE = 0.20   # stride length at MAX_SPEED [m]
  MAX_SHOULDER     = 0.087  # max shoulder yaw for turning [rad, ~5 deg]
  MAX_TURN         = 1.0    # angular.z for max shoulder lean

### 12-joint control

  Per leg: hip_joint (Z yaw), thigh_joint (Y), knee_joint (Y)
  Passive TF: cart_to_shaft_steer, shaft_to_horse

### Spot-like leg kinematics (elbow-up)

  Neutral thigh : -0.344 rad (-19.7 deg, tilted backward like Spot)
  Neutral knee  : +0.651 rad (+37.3 deg, bent forward)

---

## Section 9 -- Dynamic Cart and Steering

The cart is now a free-rolling physics object.

### New tree

  cart_base (root, spawns at x=-2.5 z=0.55)
    cart_to_shaft_steer   revolute-Z front steering (+/-35 deg)
      shaft_steer_link    shaft poles + horse collar
        shaft_to_wheel_fl / shaft_to_wheel_fr   front wheels steer with shaft
        shaft_to_horse    revolute-Y horse pitch (+/-15 deg on hills)
          base_link       horse body
    cart_to_wheel_rl / cart_to_wheel_rr   rear fixed-track wheels

Front wheels steer automatically with the shaft -- just like a real wagon.

### Cart tuning (URDF)

  Wheel rolling resistance : joint dynamics damping (default 2.0)
  Steering stiffness       : cart_to_shaft_steer dynamics damping (default 10.0)
  Cart inertia             : cart_base mass (default 120 kg)

---

## Section 10 — Blendspace v2: 5 Improvements + Gallop + Better Turning

### What changed

| # | Improvement | Details |
|---|---|---|
| 1 | **3D IK** | Hip Z-axis (yaw) angle is computed from foot target for arc-correct lateral foot placement |
| 2 | **Raibert arc-following foot placement** | `fx_mean = (vx - omega*ly)*T_stance/2` — inner legs take shorter steps, outer legs longer |
| 3 | **Active cart steering** | `cart_to_shaft_steer` is now a commanded joint. Turning sends `steer = turn_norm * 0.45 rad` to actively aim front wheels |
| 4 | **Body leveling** | Reads `shaft_to_horse` pitch; front/rear leg heights adjust to keep horse body level on slopes |
| 5 | **Contact detection** | Knee effort spike during swing (>40 Nm, >45% through swing) triggers early stance for uneven terrain |

### New controls

| Key | Action |
|-----|--------|
| **Q** | Toggle gait: **TROT** (1.0 s period) or **GALLOP** (0.65 s period, higher steps) |
| **A / D** | Turn left / right — now uses active cart steering |

### Manually set gait from another terminal

```bash
ros2 topic pub --once /cmd_gait std_msgs/String "data: 'gallop'"
ros2 topic pub --once /cmd_gait std_msgs/String "data: 'trot'"
```

### Tuning reference

| Parameter | File | Default | Effect |
|---|---|---|---|
| MAX_STEER | blendspace_node.py | 0.45 rad | Max cart steering angle (~26 deg) |
| MAX_SHOULDER | blendspace_node.py | 0.20 rad | Max hip yaw from 3D IK |
| TROT period | blendspace_node.py | 1.0 s | Trot gait cycle time |
| GALLOP period | blendspace_node.py | 0.65 s | Gallop gait cycle time |
| CONTACT_EFFORT_THRESHOLD | blendspace_node.py | 40 Nm | Knee effort for early stance |
| TURNING | teleop_key.py | 0.8 rad/s | Turn rate from A/D keys |
| cart_to_shaft_steer gains | ros2_control.yaml | p=800 d=80 | Steering responsiveness |
