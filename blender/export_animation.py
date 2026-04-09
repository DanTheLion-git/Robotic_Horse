"""
export_animation.py  —  Blender Python script
═══════════════════════════════════════════════
Run this inside Blender's built-in Python console (Scripting workspace)
to export the active armature's animations to the JSON format expected
by the Robotic Horse blendspace_node.

What it does:
  1. Reads the scene's armature object
  2. For each NLA action (animation clip), it bakes the animation to
     joint angles matching the URDF joint names
  3. Exports each action as a separate JSON file in the output directory

Usage in Blender:
  1. Open Blender → Scripting workspace
  2. Open this file: Text Editor → Open → select this file
  3. Edit OUTPUT_DIR below to point to your animations/ folder
  4. Press "Run Script" (▶ button)

Prerequisites:
  • Armature bones must be named to match URDF joint names
    (see blender/INSTRUCTIONS.md for the naming convention)
  • Each animation is stored as a separate NLA Action (strip)

Output format (per action):
  {
    "name":         "walk_forward",
    "description":  "...",
    "duration":     1.0,
    "n_frames":     60,
    "joint_names":  ["fl_thigh_joint", "fl_knee_joint", ...],
    "frames":       [[angle, angle, ...], ...]   // radians
  }
"""

import bpy
import json
import math
import os

# ═══════════════════════════════════════════════════════════════════
# CONFIGURATION — edit these before running
# ═══════════════════════════════════════════════════════════════════

# Path to the animations/ folder in your ros2_ws checkout.
# Use an absolute path or // for relative-to-blend-file.
OUTPUT_DIR = "//../../ros2_ws/src/robotic_horse_control/animations"

# Name of the armature object in the Blender scene
ARMATURE_NAME = "RoboticHorse"

# Maps NLA Action name → blendspace description (optional, cosmetic)
ACTION_DESCRIPTIONS = {
    "idle":         "Standing still — subtle weight-shift bob",
    "walk_forward": "Trot gait straight ahead at full speed",
    "walk_left":    "Arc-left trot",
    "walk_right":   "Arc-right trot",
    "turn_left":    "Spin in place to the left",
    "turn_right":   "Spin in place to the right",
}

# Bone name → URDF joint name mapping.
# Rename your Blender bones to match these URDF joint names exactly,
# OR edit this mapping to bridge Blender's naming to URDF naming.
BONE_TO_JOINT = {
    "fl_thigh": "fl_thigh_joint",
    "fl_knee":  "fl_knee_joint",
    "fr_thigh": "fr_thigh_joint",
    "fr_knee":  "fr_knee_joint",
    "rl_thigh": "rl_thigh_joint",
    "rl_knee":  "rl_knee_joint",
    "rr_thigh": "rr_thigh_joint",
    "rr_knee":  "rr_knee_joint",
}

# Rotation channel to read per bone.
# For a bone rotating around its local Y-axis (matching URDF <axis xyz="0 1 0">),
# use "rotation_euler" index 1 (Y).
# If you use quaternions in Blender, change to "rotation_quaternion".
ROTATION_CHANNEL = "rotation_euler"
ROTATION_AXIS    = 1   # 0=X, 1=Y, 2=Z

# ═══════════════════════════════════════════════════════════════════


def get_bone_angle(pose_bone, frame: int) -> float:
    """Sample a bone's rotation angle at a given frame."""
    bpy.context.scene.frame_set(frame)
    if ROTATION_CHANNEL == "rotation_euler":
        return pose_bone.rotation_euler[ROTATION_AXIS]
    elif ROTATION_CHANNEL == "rotation_quaternion":
        q = pose_bone.rotation_quaternion
        # Convert quaternion to Y-axis angle (simplified for single-axis bones)
        return 2.0 * math.asin(max(-1.0, min(1.0, q[ROTATION_AXIS + 1])))
    return 0.0


def export_action(armature_obj, action, output_dir: str):
    """Export one NLA Action to a JSON file."""
    action_name = action.name
    frame_start = int(action.frame_range[0])
    frame_end   = int(action.frame_range[1])
    n_frames    = frame_end - frame_start + 1
    fps         = bpy.context.scene.render.fps
    duration    = n_frames / fps

    joint_names = list(BONE_TO_JOINT.values())
    frames_data = []

    # Temporarily assign this action to the armature
    armature_obj.animation_data.action = action

    pose_bones = armature_obj.pose.bones
    for frame in range(frame_start, frame_end + 1):
        row = []
        for bone_name, joint_name in BONE_TO_JOINT.items():
            if bone_name in pose_bones:
                angle = get_bone_angle(pose_bones[bone_name], frame)
                row.append(round(angle, 6))
            else:
                print(f"  WARNING: bone '{bone_name}' not found, using 0.0")
                row.append(0.0)
        frames_data.append(row)

    data = {
        "name":        action_name,
        "description": ACTION_DESCRIPTIONS.get(action_name, ""),
        "duration":    round(duration, 4),
        "n_frames":    n_frames,
        "joint_names": joint_names,
        "frames":      frames_data,
    }

    out_path = os.path.join(bpy.path.abspath(output_dir), f"{action_name}.json")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(data, f, indent=2)

    print(f"  Exported '{action_name}' → {out_path}  ({n_frames} frames, {duration:.2f}s)")


def main():
    print("\n=== Robotic Horse Animation Exporter ===\n")

    # Find the armature
    armature_obj = bpy.data.objects.get(ARMATURE_NAME)
    if armature_obj is None or armature_obj.type != 'ARMATURE':
        print(f"ERROR: Armature '{ARMATURE_NAME}' not found.")
        print("       Check ARMATURE_NAME at the top of this script.")
        return

    if armature_obj.animation_data is None:
        print("ERROR: The armature has no animation data.")
        return

    # Collect all actions referenced in NLA tracks
    actions_to_export = set()
    for track in armature_obj.animation_data.nla_tracks:
        for strip in track.strips:
            if strip.action:
                actions_to_export.add(strip.action)

    # Also export the currently active action if any
    if armature_obj.animation_data.action:
        actions_to_export.add(armature_obj.animation_data.action)

    if not actions_to_export:
        print("ERROR: No NLA actions found on the armature.")
        print("       Create animations as NLA Actions — see blender/INSTRUCTIONS.md")
        return

    print(f"Found {len(actions_to_export)} action(s) to export:")
    saved_frame = bpy.context.scene.frame_current

    for action in sorted(actions_to_export, key=lambda a: a.name):
        export_action(armature_obj, action, OUTPUT_DIR)

    # Restore original frame
    bpy.context.scene.frame_set(saved_frame)
    print(f"\nDone. {len(actions_to_export)} animation(s) exported to {OUTPUT_DIR}")


main()
