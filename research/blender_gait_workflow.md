# Blender → Gazebo Gait Workflow

> How to design gaits in Blender and export them for the Highland Cow robot simulation.
> Last updated: 2026-04-19

---

## Overview

This workflow lets you:
1. **Design gaits visually** in Blender using an armature that matches the robot's skeleton
2. **Export joint angles** frame-by-frame to a JSON gait file
3. **Load the gait** into the ROS2/Gazebo simulation

The armature in Blender mirrors the exact bone lengths, joint positions, and neutral stance angles of the URDF model, so what you see in Blender is what the robot does in simulation.

---

## Prerequisites

- **Blender 3.6+** (4.x recommended) — [blender.org](https://www.blender.org/download/)
- **Python 3.10+** (for the standalone `load_gait.py` converter)
- The scripts in `Robotic_Horse/blender/`:
  - `create_armature.py` — generates the armature `.blend` file
  - `export_gait.py` — exports animation to JSON
  - `load_gait.py` — converts JSON to ROS2 gait trajectory

---

## Step 1: Generate the Armature

Run from the command line (no GUI needed):

```bash
blender --background --python blender/create_armature.py
```

This creates `blender/highland_cow_armature.blend` with:

| Bone | Length | Notes |
|------|--------|-------|
| `body` | 1.80 m | Root bone, horizontal |
| `FL_hip` / `FR_hip` | 0.05 m | At x=+0.65, y=±0.44, z=-0.15 from body center |
| `FL_thigh` / `FR_thigh` | 0.38 m | Humerus, pointing down at 0.220 rad |
| `FL_knee` / `FR_knee` | 0.34 m | Radius, bent at -0.466 rad |
| `FL_cannon` / `FR_cannon` | 0.18 m | Metacarpal (passive linkage) |
| `RL_hip` / `RR_hip` | 0.05 m | At x=-0.65, y=±0.40, z=+0.12 from body center |
| `RL_thigh` / `RR_thigh` | 0.50 m | Femur, pointing down at -0.236 rad |
| `RL_knee` / `RR_knee` | 0.45 m | Tibia, bent at 0.499 rad |
| `RL_cannon` / `RR_cannon` | 0.22 m | Metatarsal (reciprocal apparatus) |
| `neck` | 0.40 m | From front of body, angled up |
| `skull` | 0.30 m | From neck tip |
| `jaw` | 0.15 m | From skull |

Open the `.blend` file in Blender to verify the skeleton looks correct — all four hooves should be touching the ground plane.

---

## Step 2: Animate the Gait in Blender

1. Open `highland_cow_armature.blend` in Blender
2. Select the armature, switch to **Pose Mode** (Ctrl+Tab or dropdown)
3. Set up the timeline:
   - For a **walk cycle**: 48 frames at 24 fps = 2 seconds per full cycle
   - For a **trot cycle**: 24 frames at 24 fps = 1 second per full cycle
4. Keyframe the joint rotations:

### Which bones to animate

| Bone | Axis | What it controls |
|------|------|-----------------|
| `{leg}_hip` | Z rotation | Lateral leg splay (small, ±0.1 rad) |
| `{leg}_thigh` | Y rotation | Main leg swing (forward/back) |
| `{leg}_knee` | Y rotation | Knee bend |
| `{leg}_cannon` | **DO NOT animate** | Auto-calculated from linkage |
| `neck` | Y rotation | Head bob |
| `jaw` | Y rotation | Mouth open/close |

### Tips for realistic bovine gaits

- **Walk** (4-beat lateral): Leg order is RL → FL → RR → FR, each 25% phase offset
- **Trot** (2-beat diagonal): FL+RR move together, FR+RL move together, 50% phase offset
- Use the **Graph Editor** for smooth interpolation (Bézier curves)
- Keep hip yaw small (±0.05 rad) — cows don't splay legs much
- Front knees bend **backward** (negative values), rear knees bend **forward** (positive)
- The cannon bones will be auto-filled by `export_gait.py` using the linkage formulas

### Reference angles

| Joint | Neutral | Swing range (walk) | Swing range (trot) |
|-------|---------|-------------------|-------------------|
| Front thigh | 0.220 | ±0.15 rad | ±0.20 rad |
| Front knee | -0.466 | ±0.12 rad | ±0.18 rad |
| Rear thigh | -0.236 | ±0.18 rad | ±0.25 rad |
| Rear knee | 0.499 | ±0.15 rad | ±0.22 rad |

---

## Step 3: Export the Gait

With the animated `.blend` file open:

```bash
blender highland_cow_armature.blend --background --python blender/export_gait.py -- walk_gait.json
```

Or from Blender's Python console / Scripting tab:
```python
exec(open("blender/export_gait.py").read())
```

This produces a JSON file like:
```json
{
  "name": "walk",
  "fps": 24,
  "frame_count": 48,
  "joints": {
    "fl_hip": [0.0, 0.01, ...],
    "fl_thigh": [0.22, 0.23, ...],
    "fl_knee": [-0.46, -0.45, ...],
    "fl_cannon": [0.32, 0.31, ...],
    ...
  }
}
```

The cannon angles are **auto-computed** from the linkage formulas:
- Front: `cannon = 0.08 - (thigh + knee)`
- Rear: `cannon = 0.3792 - 0.85 × knee`

---

## Step 4: Load into ROS2 Simulation

Convert the JSON gait to a Python trajectory file:

```bash
python blender/load_gait.py walk_gait.json --output ros2_ws/src/robotic_horse_control/robotic_horse_control/gaits/blender_walk.py
```

This generates a Python module with numpy arrays that the `blendspace_node.py` controller can import.

### Using the gait in simulation

The generated Python file contains:
```python
GAIT_NAME = "walk"
FPS = 24
FRAME_COUNT = 48
JOINT_TRAJECTORIES = {
    "fl_hip": np.array([...]),
    "fl_thigh": np.array([...]),
    ...
}
```

You can integrate it into the existing gait system by importing it in `blendspace_node.py` or playing it back frame-by-frame.

---

## Step 5: Iterate

1. Run the simulation: `ros2 launch robotic_horse_control full_sim.launch.py`
2. Observe the motion in Gazebo
3. Go back to Blender, adjust keyframes
4. Re-export and re-run

### Quick iteration loop

```bash
# Terminal 1: Export + load
blender highland_cow_armature.blend --background --python blender/export_gait.py -- walk.json && \
python blender/load_gait.py walk.json --output ros2_ws/src/.../gaits/blender_walk.py

# Terminal 2: Rebuild + run
cd ros2_ws && colcon build --symlink-install && source install/setup.bash && \
ros2 launch robotic_horse_control full_sim.launch.py
```

---

## Coordinate Mapping

| URDF / Gazebo | Blender | Description |
|---------------|---------|-------------|
| X+ | -Y | Forward |
| Y+ | X | Left |
| Z+ | Z | Up |

The `create_armature.py` script handles this mapping automatically. Joint angles exported by `export_gait.py` are already in URDF convention (radians).

---

## Troubleshooting

### Armature doesn't look right
- Make sure you ran `create_armature.py` from the `Robotic_Horse/` directory
- Check Blender version ≥ 3.6

### Legs clip through ground
- Verify the armature origin is at Z=1.12m (body center height)
- Check that neutral stance angles match the reference table above

### Exported angles seem wrong
- Blender rotations in Pose Mode are **relative to rest pose**
- The export script adds the rest-pose angles to get absolute URDF angles
- If you modified the rest pose, re-run `create_armature.py`

### Simulation doesn't match Blender
- Check FPS: simulation runs at ~50 Hz, Blender animation at 24 fps
- `load_gait.py` interpolates between frames automatically
- Verify the cannon angles match the linkage formulas (the script warns if they diverge)

---

## File Reference

| File | Type | Purpose |
|------|------|---------|
| `blender/create_armature.py` | Blender Python | Generate armature `.blend` |
| `blender/export_gait.py` | Blender Python | Export animation → JSON |
| `blender/load_gait.py` | Standalone Python | JSON → ROS2 gait trajectory |
| `blender/highland_cow_armature.blend` | Blender file | Generated armature (do not edit by hand) |
| `research/blender_gait_workflow.md` | Markdown | This tutorial |
