#!/usr/bin/env python3
"""
export_gait.py — Blender Python script to export armature animation as joint-angle JSON.

Usage:
    blender highland_cow_armature.blend --background --python export_gait.py -- output.json
    blender highland_cow_armature.blend --background --python export_gait.py -- -o walk_gait.json --plot

The script reads the active armature's current Action (animation), samples
every frame, and writes a JSON file mapping joint names to arrays of angles
(one per frame, in radians).

Coordinate convention (Blender → URDF):
    Blender bone local rotation around X-axis  →  URDF pitch (Y-axis)
    Blender bone local rotation around Z-axis  →  URDF yaw  (Z-axis)

Compatible with Blender 3.6+ and 4.0+.
"""

import bpy
import json
import math
import sys
import os
from mathutils import Quaternion, Euler

# ---------------------------------------------------------------------------
#  Joint mapping — bone names to exported joint keys
# ---------------------------------------------------------------------------

LEG_PREFIXES = ["FL", "FR", "RL", "RR"]

# Per-leg bones and the rotation component to extract.
# (bone_suffix, json_suffix, axis)
#   axis: 'pitch' = rotation about bone local X (→ URDF Y-axis pitch)
#          'yaw'  = rotation about bone local Z (→ URDF Z-axis yaw)
LEG_JOINTS = [
    ("hip",    "hip",    "yaw"),
    ("thigh",  "thigh",  "pitch"),
    ("knee",   "knee",   "pitch"),
    ("cannon", "cannon", "pitch"),
]

# Head / neck joints
HEAD_JOINTS = [
    ("neck",  "neck_yaw",   "yaw"),
    ("neck",  "neck_pitch", "pitch"),
    ("neck",  "neck_roll",  "roll"),
    ("skull", "skull_pitch", "pitch"),
    ("jaw",   "jaw_pitch",  "pitch"),
]


# ---------------------------------------------------------------------------
#  Helpers
# ---------------------------------------------------------------------------

def get_pose_bone_angle(pose_bone, axis):
    """
    Extract a single rotation axis from a pose bone's rotation relative to
    its rest pose.

    Blender stores pose-mode rotation as the *delta* from the rest (edit) pose,
    so the quaternion/euler already represents the joint deflection.

    axis: 'pitch' → local X rotation (URDF Y-axis)
          'yaw'   → local Z rotation (URDF Z-axis)
          'roll'  → local Y rotation (URDF X-axis / roll)
    """
    # Normalise to euler XYZ for easy extraction
    rot_mode = pose_bone.rotation_mode
    if rot_mode == 'QUATERNION':
        euler = pose_bone.rotation_quaternion.to_euler('XYZ')
    elif rot_mode == 'AXIS_ANGLE':
        aa = pose_bone.rotation_axis_angle
        q = Quaternion(aa[1:4], aa[0])
        euler = q.to_euler('XYZ')
    else:
        euler = pose_bone.rotation_euler.copy()
        euler.order = 'XYZ'

    if axis == 'pitch':
        return euler.x          # bone-local X → URDF pitch
    elif axis == 'yaw':
        return euler.z          # bone-local Z → URDF yaw
    elif axis == 'roll':
        return euler.y          # bone-local Y → URDF roll
    else:
        raise ValueError(f"Unknown axis: {axis}")


def find_armature():
    """Return the first armature object in the scene."""
    for obj in bpy.data.objects:
        if obj.type == 'ARMATURE':
            return obj
    return None


# ---------------------------------------------------------------------------
#  Export
# ---------------------------------------------------------------------------

def export_gait(output_path, do_plot=False):
    arm_obj = find_armature()
    if arm_obj is None:
        print("[export_gait] ERROR: No armature found in the scene.")
        return

    # Ensure pose mode context
    bpy.context.view_layer.objects.active = arm_obj
    bpy.ops.object.mode_set(mode='POSE')

    scene = bpy.context.scene
    action = arm_obj.animation_data.action if arm_obj.animation_data else None

    if action is None:
        print("[export_gait] WARNING: No animation action found. Exporting single-frame neutral pose.")
        frame_start = scene.frame_start
        frame_end = scene.frame_start
    else:
        frame_start = int(action.frame_range[0])
        frame_end = int(action.frame_range[1])
        print(f"[export_gait] Action '{action.name}': frames {frame_start}–{frame_end}")

    fps = scene.render.fps
    frame_count = frame_end - frame_start + 1

    # Build joint key list and prepare storage
    joint_keys = []
    # Leg joints
    for prefix in LEG_PREFIXES:
        for bone_suffix, json_suffix, axis in LEG_JOINTS:
            jk = f"{prefix.lower()}_{json_suffix}"
            joint_keys.append((jk, f"{prefix}_{bone_suffix}", axis))
    # Head joints
    for bone_name, json_key, axis in HEAD_JOINTS:
        joint_keys.append((json_key, bone_name, axis))

    data = {jk: [] for jk, _, _ in joint_keys}

    # Sample every frame
    for frame in range(frame_start, frame_end + 1):
        scene.frame_set(frame)
        for jk, bone_name, axis in joint_keys:
            pb = arm_obj.pose.bones.get(bone_name)
            if pb is None:
                data[jk].append(0.0)
            else:
                angle = get_pose_bone_angle(pb, axis)
                data[jk].append(round(angle, 6))

    # Determine gait name from action or filename
    gait_name = action.name if action else "neutral"

    result = {
        "name": gait_name,
        "fps": fps,
        "frame_count": frame_count,
        "joints": data,
    }

    # Write JSON
    with open(output_path, 'w') as f:
        json.dump(result, f, indent=2)
    print(f"[export_gait] Wrote {output_path} ({frame_count} frames, {len(data)} joints)")

    # Optional matplotlib preview
    if do_plot:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt

            fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
            frames = list(range(frame_start, frame_end + 1))

            for idx, prefix in enumerate(LEG_PREFIXES):
                ax = axes[idx]
                for bone_suffix, json_suffix, axis in LEG_JOINTS:
                    jk = f"{prefix.lower()}_{json_suffix}"
                    ax.plot(frames, data[jk], label=json_suffix)
                ax.set_ylabel(f"{prefix} (rad)")
                ax.legend(loc='upper right', fontsize=8)
                ax.grid(True, alpha=0.3)

            axes[-1].set_xlabel("Frame")
            fig.suptitle(f"Gait: {gait_name}  ({frame_count} frames @ {fps} fps)")
            fig.tight_layout()

            plot_path = os.path.splitext(output_path)[0] + "_preview.png"
            fig.savefig(plot_path, dpi=100)
            print(f"[export_gait] Preview plot → {plot_path}")
        except ImportError:
            print("[export_gait] matplotlib not available — skipping plot.")

    bpy.ops.object.mode_set(mode='OBJECT')


# ---------------------------------------------------------------------------
#  CLI argument parsing (after Blender's "--" separator)
# ---------------------------------------------------------------------------

def parse_args():
    """Parse arguments that come after '--' in the Blender command line."""
    argv = sys.argv
    if "--" in argv:
        args = argv[argv.index("--") + 1:]
    else:
        args = []

    output_path = "gait_output.json"
    do_plot = False

    i = 0
    while i < len(args):
        if args[i] in ("-o", "--output"):
            i += 1
            if i < len(args):
                output_path = args[i]
        elif args[i] == "--plot":
            do_plot = True
        else:
            # Positional: treat as output path
            output_path = args[i]
        i += 1

    return output_path, do_plot


# ---------------------------------------------------------------------------
#  Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    output_path, do_plot = parse_args()
    export_gait(output_path, do_plot)
