# Blender → ROS2 Animation & Blendspace Tutorial

A complete guide to creating robot animations in Blender, exporting them, and running a 2D Animation Blendspace — the equivalent of Unreal Engine's Animation Blendspace — inside ROS2.

---

## Table of Contents

1. [Concept Overview](#1-concept-overview)
2. [What is a Blendspace?](#2-what-is-a-blendspace)
3. [The Pipeline: Blender → JSON → ROS2](#3-the-pipeline-blender--json--ros2)
4. [Software Prerequisites](#4-software-prerequisites)
5. [Setting Up the Blender Project](#5-setting-up-the-blender-project)
6. [Creating the Robot Armature](#6-creating-the-robot-armature)
7. [Bone Naming Convention](#7-bone-naming-convention)
8. [Creating Your First Animation (Idle)](#8-creating-your-first-animation-idle)
9. [Creating the Walk Forward Animation](#9-creating-the-walk-forward-animation)
10. [Creating Turning and Arc Animations](#10-creating-turning-and-arc-animations)
11. [Organising Animations with the NLA Editor](#11-organising-animations-with-the-nla-editor)
12. [Exporting Animations from Blender](#12-exporting-animations-from-blender)
13. [The Blendspace Definition File](#13-the-blendspace-definition-file)
14. [Running the Blendspace in ROS2](#14-running-the-blendspace-in-ros2)
15. [Controlling the Robot Live](#15-controlling-the-robot-live)
16. [How the Blending Algorithm Works](#16-how-the-blending-algorithm-works)
17. [Tips for Smooth Animations](#17-tips-for-smooth-animations)
18. [Troubleshooting](#18-troubleshooting)

---

## 1. Concept Overview

In game engines like Unreal Engine, an **Animation Blendspace** lets you place animations at points on a 2D graph and blend smoothly between them at runtime based on character state (e.g. speed and direction). The same idea applies perfectly to a robot:

```
Angular rate (turning)
     -1          0          +1
      │           │           │
  1.0 ●──────────●──────────●   ← forward speed = 1.0
      │ walk_left │ walk_fwd │ walk_right
      │           │           │
  0.0 ●──────────●──────────●   ← forward speed = 0.0
      │ turn_left │   idle   │ turn_right
```

When you send `/cmd_vel` commands with `linear.x = 0.7` and `angular.z = 0.5`, the blendspace system samples the 4 nearest animation anchors and blends their joint angles together — the result is a smooth, natural-looking motion without hardcoded transition logic.

---

## 2. What is a Blendspace?

In our ROS2 system:

- **Animation** — a JSON file containing joint angles (in radians) for 8 joints over N frames
- **Anchor** — one animation placed at a specific (speed, angular_rate) point in the blendspace
- **Blending** — inverse-distance-weighted interpolation between the nearest anchors
- **Phase** — a normalised [0, 1) counter that drives which frame is currently playing; all blended animations share the same phase so they stay synchronised

The `blendspace_node.py` reads `blendspace.yaml`, loads all animation JSON files, and handles blending + publishing automatically. You only need to:
1. Create the animations in Blender
2. Export them as JSON files
3. Add them to `blendspace.yaml`

---

## 3. The Pipeline: Blender → JSON → ROS2

```
Blender
  ├── Robot armature (bones = joints)
  ├── NLA Actions (one per animation)
  └── export_animation.py  ──────────────▶  animation JSON files
                                                   │
                                           animations/
                                           ├── idle.json
                                           ├── walk_forward.json
                                           ├── ...
                                           └── blendspace.yaml
                                                   │
                                          blendspace_node.py
                                                   │
                                           /leg_controller/joint_trajectory
                                                   │
                                              Gazebo physics
```

---

## 4. Software Prerequisites

| Software | Version | Download |
|----------|---------|----------|
| Blender | 4.x (any recent) | https://www.blender.org/download/ |
| Python (Windows) | 3.11+ | For running generate_animations.py |
| ROS2 Humble | — | See `gazebo/INSTRUCTIONS.md` |

No special Blender add-ons are required for basic joint animation.

---

## 5. Setting Up the Blender Project

### 5.1 Open Blender and create a new project

1. Open Blender → **File → New → General**
2. Delete the default cube: select it, press `X`, confirm Delete
3. Save the file as `blender/robotic_horse.blend`

### 5.2 Set the scene units

1. Go to the **Properties panel** (right side) → **Scene** tab (🎬 icon)
2. Under **Units**, set:
   - Unit System: **Metric**
   - Unit Scale: **1.0**
   - Length: **Meters**

### 5.3 Set the scene frame rate and range

1. Still in Scene properties → **Frame Rate**: `60`
2. Set **Frame Start**: `1`, **Frame End**: `60` (= 1 second at 60 fps)

---

## 6. Creating the Robot Armature

The armature is a skeleton of **bones** that represents the robot's joints. Each bone corresponds to one URDF joint.

### 6.1 Add an armature

1. Press `Shift+A` → **Armature** → **Single Bone**
2. Press `Tab` to enter Edit Mode
3. You will see one bone. We will build the full skeleton from here.

### 6.2 Bone hierarchy for one leg

The Robotic Horse has this structure per leg:

```
Body (root, no bone needed — fixed in space)
└── {leg}_hip      — lateral bracket (fixed joint, no animation)
    └── {leg}_thigh — upper leg tube, rotates fore/aft
        └── {leg}_knee  — lower leg hinge
```

Since the hip is fixed, you only need **2 bones per leg** (thigh + knee), giving **8 bones total**.

### 6.3 Building the FL (front-left) leg

In Edit Mode with the armature selected:

1. **Select the root bone** → rename it to `fl_thigh` in the **Item panel** (press `N`)
2. Set the bone head position: `X=0.35, Y=0.18, Z=-0.13` (hip bracket height)
3. Set the bone tail position: `X=0.35, Y=0.18, Z=-0.58` (knee position)
   - Length = 0.45 m, matching `L1` in the kinematics

4. With `fl_thigh` selected, press `E` to extrude a child bone:
   - This creates `fl_knee`
   - Set its tail to `X=0.35, Y=0.18, Z=-1.08` (foot position, 0.50 m lower)
   - Rename to `fl_knee`

5. In the **Bone properties** (🦴 icon), set `fl_knee`'s parent to `fl_thigh`

### 6.4 Repeat for FR, RL, RR

Mirror the process with these hip positions:

| Leg | Hip X | Hip Y |
|-----|-------|-------|
| FL  | +0.35 | +0.18 |
| FR  | +0.35 | -0.18 |
| RL  | -0.35 | +0.18 |
| RR  | -0.35 | -0.18 |

**Tip:** After creating the FL leg bones, you can select them both, press `Shift+D` to duplicate, then move and rename for each other leg.

### 6.5 Parent all leg bones to a root bone

1. Add one more bone named `body` at the centre `(0, 0, 0)`
2. In Edit Mode, select all thigh bones (Shift+click), then Shift-click `body` last
3. Press `Ctrl+P` → **Keep Offset** — this makes `body` the parent

---

## 7. Bone Naming Convention

**Critical:** Blender bone names must match the `BONE_TO_JOINT` mapping in `export_animation.py`.

The default mapping is:

| Blender Bone Name | URDF Joint Name        |
|-------------------|------------------------|
| `fl_thigh`        | `fl_thigh_joint`       |
| `fl_knee`         | `fl_knee_joint`        |
| `fr_thigh`        | `fr_thigh_joint`       |
| `fr_knee`         | `fr_knee_joint`        |
| `rl_thigh`        | `rl_thigh_joint`       |
| `rl_knee`         | `rl_knee_joint`        |
| `rr_thigh`        | `rr_thigh_joint`       |
| `rr_knee`         | `rr_knee_joint`        |

To rename a bone: **Edit Mode → select bone → Properties → Item panel → Name field**

**Rotation axis:** Each bone rotates around its **local Y-axis**, matching the URDF `<axis xyz="0 1 0">`. In Blender, make sure each bone's Y-axis points along the bone length (this is the default for newly extruded bones).

---

## 8. Creating Your First Animation (Idle)

### 8.1 Pose the robot at the idle (standing) position

1. Go back to **Object Mode** (`Tab`)
2. Select the armature → press `Tab` to enter **Pose Mode**
3. Press `A` to select all bones
4. Set all bones to a neutral standing pose:
   - All thigh bones: **Y rotation = 0°** (straight down)
   - All knee bones: **Y rotation = -37°** (≈ -0.65 rad, matching BODY_HEIGHT=0.90 m)

> To set a rotation: select the bone → press `R Y` → type the angle in degrees → `Enter`

### 8.2 Create an Action for the idle animation

1. Open the **Action Editor** (bottom panel → change type to **Action Editor**)
2. Click **New** to create a new action
3. Rename it to `idle` (click the action name in the header)

### 8.3 Keyframe the idle bob

The idle animation has a gentle vertical bob over 60 frames:

1. At **frame 1**: all knees at -37° → press `I` → **Whole Character**
2. At **frame 30**: all knees at -40° (body lower) → `I` → **Whole Character**
3. At **frame 60**: all knees at -37° (back to start) → `I` → **Whole Character**

This creates a 1-second idle cycle that loops seamlessly.

---

## 9. Creating the Walk Forward Animation

The trot gait moves diagonal pairs of legs simultaneously:
- **Phase 0.0–0.4**: FL + RR swing (lift and move forward), FR + RL stance
- **Phase 0.4–0.5**: transition
- **Phase 0.5–0.9**: FR + RL swing, FL + RR stance
- **Phase 0.9–1.0**: transition

### 9.1 Create a new Action

1. In the Action Editor, click the icon next to the action name → **New**
2. Name it `walk_forward`

### 9.2 Key joint angles for the trot

Use these approximate angles at each key phase. All angles are Y-rotation in **degrees**:

| Frame | FL Thigh | FL Knee | FR Thigh | FR Knee | RL Thigh | RL Knee | RR Thigh | RR Knee |
|-------|----------|---------|----------|---------|----------|---------|----------|---------|
| 1 (0%)  | +11° | -42° | -11° | -32° | -11° | -32° | +11° | -42° |
| 12 (20%)| +22° | -55° | -6°  | -35° | -6°  | -35° | +22° | -55° |
| 24 (40%)| 0°   | -37° | +11° | -42° | +11° | -42° | 0°   | -37° |
| 36 (60%)| -11° | -32° | +22° | -55° | +22° | -55° | -11° | -32° |
| 48 (80%)| -6°  | -35° | 0°   | -37° | 0°   | -37° | -6°  | -35° |
| 60 (100%)| +11° | -42° | -11° | -32° | -11° | -32° | +11° | -42° |

> Frame 1 and Frame 60 must be identical for seamless looping.

### 9.3 Inserting keyframes efficiently

1. Select all bones (`A`)
2. Go to frame 1 → set all rotations → press `I` → **Whole Character**
3. Go to frame 12 → adjust angles → `I` → **Whole Character**
4. Continue for all key frames

**Tip:** Use the **Graph Editor** to smooth the curves with **Auto Bezier** handles for fluid motion between keyframes.

---

## 10. Creating Turning and Arc Animations

### walk_left (arc left, speed=1, angular_rate=-1)

The right legs take **longer strides** (wider arc) and the left legs take **shorter strides**:

- Right legs (FR, RR): step length ≈ +35% of normal → larger thigh rotation (±15°)
- Left legs (FL, RL):  step length ≈ -35% of normal → smaller thigh rotation (±7°)
- All knee angles remain similar (body height unchanged)

Create this as a new action named `walk_left`.

### walk_right (arc right, speed=1, angular_rate=+1)

Mirror of `walk_left`: swap left and right leg stride lengths.

### turn_left (in-place spin, speed=0, angular_rate=-1)

The left legs step **backwards** while the right legs step **forwards**:
- FL + RL: phase shifted by 0.5 (moves opposite to normal)
- FR + RR: normal phase (moves forward)
- All step lengths ≈ 0.08 m (small — turning in place)

### turn_right (in-place spin, speed=0, angular_rate=+1)

Mirror of `turn_left`.

---

## 11. Organising Animations with the NLA Editor

The **NLA (Non-Linear Animation) Editor** is where you store multiple animation clips as independent **Actions** that can be managed separately.

### 11.1 Push each action to the NLA stack

1. With the armature selected, go to the **NLA Editor** (change bottom panel type)
2. For each action you want to store:
   - In the Action Editor, select the action
   - Click the **Push Down** button (↓ icon) in the NLA editor header
   - This moves it to an NLA strip, freeing the active action slot for the next one

### 11.2 NLA layout

After pushing all 6 actions, your NLA editor should show:

```
NLA Track: idle           ████ idle
NLA Track: walk_forward   ████ walk_forward
NLA Track: walk_left      ████ walk_left
NLA Track: walk_right     ████ walk_right
NLA Track: turn_left      ████ turn_left
NLA Track: turn_right     ████ turn_right
```

The export script reads from these NLA tracks automatically.

---

## 12. Exporting Animations from Blender

### 12.1 Configure the export script

Open `blender/export_animation.py` and edit the top section:

```python
# Edit this to point to your animations/ folder
OUTPUT_DIR = "//../../ros2_ws/src/robotic_horse_control/animations"

# Must match the name of your armature object in Blender
ARMATURE_NAME = "RoboticHorse"
```

`//` means relative to the `.blend` file location. The path above assumes the file is saved at `blender/robotic_horse.blend`.

### 12.2 Run the script in Blender

1. Switch to the **Scripting** workspace (top header tab)
2. Click **Open** → navigate to `blender/export_animation.py` → click **Open Text Block**
3. Click the **▶ Run Script** button (or press `Alt+P`)
4. Check the **Info** bar (bottom of screen) and the system console for output

Successful output looks like:
```
=== Robotic Horse Animation Exporter ===
  Exported 'idle'         → .../animations/idle.json         (60 frames, 1.00s)
  Exported 'walk_forward' → .../animations/walk_forward.json (60 frames, 1.00s)
  ...
Done. 6 animation(s) exported.
```

### 12.3 Verify the export

Each exported JSON looks like:
```json
{
  "name": "walk_forward",
  "description": "Trot gait straight ahead at full speed",
  "duration": 1.0,
  "n_frames": 60,
  "joint_names": ["fl_thigh_joint", "fl_knee_joint", ...],
  "frames": [
    [0.192, -0.733, -0.192, -0.559, ...],
    ...
  ]
}
```

---

## 13. The Blendspace Definition File

`animations/blendspace.yaml` defines where each animation sits on the 2D graph:

```yaml
name: "locomotion_blendspace"

axes:
  x:
    label: "forward_speed"   # 0 = stopped, 1 = full speed
    min: 0.0
    max: 1.0
  y:
    label: "angular_rate"    # -1 = full left turn, +1 = full right turn
    min: -1.0
    max: 1.0

animations:
  - file: "idle.json"
    x: 0.0
    y: 0.0
  - file: "walk_forward.json"
    x: 1.0
    y: 0.0
  - file: "walk_left.json"
    x: 1.0
    y: -1.0
  - file: "walk_right.json"
    x: 1.0
    y: 1.0
  - file: "turn_left.json"
    x: 0.0
    y: -1.0
  - file: "turn_right.json"
    x: 0.0
    y: 1.0
```

To add a new animation (e.g. a slow walk):
1. Create and export it from Blender as `walk_slow.json`
2. Add an entry with `x: 0.5, y: 0.0` — it will automatically be blended into transitions through the centre of the graph

---

## 14. Running the Blendspace in ROS2

### Step 1 — Start Gazebo (Terminal 1)

```bash
cd ~/Robotic_Horse/ros2_ws
source install/setup.bash
ros2 launch robotic_horse_control gazebo.launch.py
```

### Step 2 — Start the Blendspace node (Terminal 2)

This replaces `robot_control.launch.py` when using animation files:

```bash
ros2 launch robotic_horse_control blendspace.launch.py
```

You will see:
```
[blendspace_node]: Blendspace node ready — 6 anchor animations loaded.
[blendspace_node]:   Loaded [+0.0,  0.0] → idle
[blendspace_node]:   Loaded [+1.0,  0.0] → walk_forward
[blendspace_node]:   Loaded [+1.0, -1.0] → walk_left
...
```

---

## 15. Controlling the Robot Live

Send `/cmd_vel` messages from any terminal:

```bash
# Walk straight ahead at full speed
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
    "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Arc left (speed 0.8, angular -0.6)
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
    "{linear: {x: 0.8}, angular: {z: -0.6}}"

# Turn right in place
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
    "{linear: {x: 0.0}, angular: {z: 1.5}}"

# Stop
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Use `teleop_twist_keyboard` for interactive keyboard control:

```bash
sudo apt install -y ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Monitor the blendspace state

```bash
# JSON state: current speed, angular_rate, phase
ros2 topic echo /robotic_horse/blendspace_state

# Live force plot
rqt_plot /robotic_horse/ballscrew_forces/data[0]:data[1]:data[2]:data[3]
```

---

## 16. How the Blending Algorithm Works

The `blendspace_node.py` uses **Inverse Distance Weighting (IDW)**:

1. For the current command point `(px, py)`, compute the squared Euclidean distance to every anchor `(ax, ay)`
2. Weight each anchor: `w_i = 1 / distance_i²`
3. Normalise all weights to sum to 1.0
4. Sample each anchor's animation at the shared phase
5. Compute `blended_angle[j] = Σ w_i × anchor_i.angle[j]`

This has nice properties:
- **Exact interpolation**: if the command is exactly on an anchor, that animation plays with weight 1.0
- **Smooth transitions**: as the command moves through the space, weights change smoothly
- **Works with any anchor layout**: no requirement for a regular grid

**Phase synchronisation**: all animations share a single phase counter that advances at a speed proportional to `forward_speed`. This ensures all blended animations are always in the same gait cycle phase — no jarring mid-stride transitions.

---

## 17. Tips for Smooth Animations

### Loop seamlessly
Frame 1 and Frame 60 must have identical joint angles. In the Graph Editor, select all curves → `N` panel → Modifiers → **Cycles** modifier to automatically make any action loop perfectly.

### Use the Graph Editor for fluid motion
- Select all keyframes in the **Graph Editor**
- Press `T` → **Auto Bezier** for smooth, natural-looking curves
- Avoid sudden angle jumps, especially for the knee joint

### Match phase at transition points
When blending between two animations, the smoothest results come when the stance-to-swing transitions happen at the same phase (frame). Use the same phase offsets as `walk_gait.py`:
```
FL + RR: swing at phase 0.0
FR + RL: swing at phase 0.5
```

### Keep consistent body height
All animations should maintain approximately the same body height (same average knee angle). Large height differences cause the body to bob unnaturally when blending.

### Test with placeholder animations first
The `scripts/generate_animations.py` generates mathematically correct placeholder animations from the kinematics model. Run the blendspace with these first to verify the system works before investing time in Blender animation.

---

## 18. Troubleshooting

### Export script: `bone not found`
Check that your bone names in Blender exactly match the `BONE_TO_JOINT` keys in `export_animation.py`. Bone names are case-sensitive.

### Animations load but the robot twitches
The bone rotation axis may be wrong. In `export_animation.py`, try changing `ROTATION_AXIS` from `1` (Y) to `0` (X) or `2` (Z) depending on how your bones are oriented in Blender.

### Blendspace node: `animation file not found`
Make sure you ran `colcon build` after adding new JSON files. The build step copies files from `animations/` into the ROS2 install directory. Alternatively, use `--symlink-install` when building:
```bash
colcon build --symlink-install
```

### Motion looks wrong at blendspace corners
Check that `blendspace.yaml` has the correct (x, y) coordinates for each animation. A misplaced anchor causes the wrong animation to be weighted heavily at incorrect commands.

### Robot falls over during transitions
Increase the PID `d` gain in `ros2_control.yaml` — the derivative term resists fast changes. Also reduce the blendspace `publish_rate_hz` to 30 for debugging.
