#!/usr/bin/env python3
"""
load_gait.py — Standalone script (no Blender required) to load an exported
gait JSON file and convert it to a ROS2-compatible Python trajectory module.

Usage:
    python load_gait.py gait_output.json
    python load_gait.py gait_output.json -o walk_trajectory.py
    python load_gait.py gait_output.json --verify   # verify cannon linkage formulas

The output Python file defines trajectory arrays that can be imported by
the Highland Cow ROS2 gait controller.
"""

import argparse
import json
import math
import sys
import os
from typing import Dict, List

# ---------------------------------------------------------------------------
#  Linkage formulas (must match create_armature.py)
# ---------------------------------------------------------------------------

def cannon_angle_front(thigh: float, knee: float) -> float:
    """Front cannon passive linkage: cannon = 0.08 - (thigh + knee)."""
    return 0.08 - (thigh + knee)


def cannon_angle_rear(knee: float) -> float:
    """Rear reciprocal apparatus: cannon = 0.3792 - 0.85 * knee."""
    return 0.3792 - 0.85 * knee


# ---------------------------------------------------------------------------
#  Verification
# ---------------------------------------------------------------------------

def verify_cannon_linkage(gait: dict, tolerance: float = 0.05) -> List[str]:
    """
    Compare exported cannon angles against the linkage formulas.
    Returns a list of warning strings (empty if all OK).
    """
    warnings = []
    joints = gait["joints"]
    frame_count = gait["frame_count"]

    for prefix in ["fl", "fr"]:
        thigh_key = f"{prefix}_thigh"
        knee_key = f"{prefix}_knee"
        cannon_key = f"{prefix}_cannon"
        if not all(k in joints for k in (thigh_key, knee_key, cannon_key)):
            continue
        for i in range(frame_count):
            expected = cannon_angle_front(joints[thigh_key][i], joints[knee_key][i])
            actual = joints[cannon_key][i]
            if abs(expected - actual) > tolerance:
                warnings.append(
                    f"  Frame {i}: {prefix.upper()} cannon expected {expected:.4f}, "
                    f"got {actual:.4f} (Δ={abs(expected - actual):.4f})"
                )

    for prefix in ["rl", "rr"]:
        knee_key = f"{prefix}_knee"
        cannon_key = f"{prefix}_cannon"
        if not all(k in joints for k in (knee_key, cannon_key)):
            continue
        for i in range(frame_count):
            expected = cannon_angle_rear(joints[knee_key][i])
            actual = joints[cannon_key][i]
            if abs(expected - actual) > tolerance:
                warnings.append(
                    f"  Frame {i}: {prefix.upper()} cannon expected {expected:.4f}, "
                    f"got {actual:.4f} (Δ={abs(expected - actual):.4f})"
                )

    return warnings


# ---------------------------------------------------------------------------
#  Recompute cannon from linkage (authoritative)
# ---------------------------------------------------------------------------

def recompute_cannon(gait: dict) -> dict:
    """
    Override exported cannon angles with values computed from the linkage
    formulas, since cannon joints are passive (not motor-driven).
    Returns a new gait dict with corrected cannon arrays.
    """
    joints = dict(gait["joints"])
    frame_count = gait["frame_count"]

    for prefix in ["fl", "fr"]:
        thigh = joints.get(f"{prefix}_thigh", [0.0] * frame_count)
        knee = joints.get(f"{prefix}_knee", [0.0] * frame_count)
        joints[f"{prefix}_cannon"] = [
            round(cannon_angle_front(thigh[i], knee[i]), 6)
            for i in range(frame_count)
        ]

    for prefix in ["rl", "rr"]:
        knee = joints.get(f"{prefix}_knee", [0.0] * frame_count)
        joints[f"{prefix}_cannon"] = [
            round(cannon_angle_rear(knee[i]), 6)
            for i in range(frame_count)
        ]

    result = dict(gait)
    result["joints"] = joints
    return result


# ---------------------------------------------------------------------------
#  Generate ROS2 trajectory Python module
# ---------------------------------------------------------------------------

# The 12 actuated joints (cannon is passive, not sent to motors).
# Head joints are optional — included if present.
MOTOR_JOINTS_PER_LEG = ["hip", "thigh", "knee"]
LEG_PREFIXES = ["fl", "fr", "rl", "rr"]
HEAD_JOINTS = ["neck_yaw", "neck_pitch", "neck_roll", "skull_pitch", "jaw_pitch"]

# Full URDF joint names used in the ROS2 controller
URDF_JOINT_MAP = {
    "fl_hip":   "fl_hip_joint",
    "fl_thigh": "fl_thigh_joint",
    "fl_knee":  "fl_knee_joint",
    "fr_hip":   "fr_hip_joint",
    "fr_thigh": "fr_thigh_joint",
    "fr_knee":  "fr_knee_joint",
    "rl_hip":   "rl_hip_joint",
    "rl_thigh": "rl_thigh_joint",
    "rl_knee":  "rl_knee_joint",
    "rr_hip":   "rr_hip_joint",
    "rr_thigh": "rr_thigh_joint",
    "rr_knee":  "rr_knee_joint",
    "neck_yaw":    "neck_yaw_joint",
    "neck_pitch":  "neck_pitch_joint",
    "neck_roll":   "neck_roll_joint",
    "skull_pitch": "skull_pitch_joint",
    "jaw_pitch":   "jaw_pitch_joint",
}


def format_array(arr: List[float], indent: int = 4) -> str:
    """Pretty-print a float array for Python source."""
    prefix = " " * indent
    lines = []
    row = []
    for v in arr:
        row.append(f"{v: .6f}")
        if len(row) >= 8:
            lines.append(prefix + ", ".join(row) + ",")
            row = []
    if row:
        lines.append(prefix + ", ".join(row) + ",")
    return "[\n" + "\n".join(lines) + "\n" + " " * (indent - 4) + "]"


def generate_trajectory_module(gait: dict) -> str:
    """Generate a Python source string defining the trajectory arrays."""
    joints = gait["joints"]
    name = gait.get("name", "unnamed")
    fps = gait.get("fps", 24)
    frame_count = gait["frame_count"]
    dt = round(1.0 / fps, 6)

    lines = [
        '"""',
        f"Auto-generated gait trajectory: {name}",
        f"Frames: {frame_count}  FPS: {fps}  dt: {dt}s  duration: {round(frame_count * dt, 4)}s",
        "",
        "Cannon joints are PASSIVE — driven by mechanical linkage, not motors.",
        "  Front: cannon = 0.08 - (thigh + knee)",
        "  Rear:  cannon = 0.3792 - 0.85 * knee",
        '"""',
        "",
        f'GAIT_NAME = "{name}"',
        f"FPS = {fps}",
        f"FRAME_COUNT = {frame_count}",
        f"DT = {dt}",
        f"DURATION = {round(frame_count * dt, 4)}",
        "",
        "# Ordered list of actuated joint names (matches JointTrajectory message)",
        "JOINT_NAMES = [",
    ]

    # Collect which joints are present
    active_keys = []
    for prefix in LEG_PREFIXES:
        for suffix in MOTOR_JOINTS_PER_LEG:
            key = f"{prefix}_{suffix}"
            if key in joints:
                active_keys.append(key)
    for hj in HEAD_JOINTS:
        if hj in joints:
            active_keys.append(hj)

    for key in active_keys:
        urdf_name = URDF_JOINT_MAP.get(key, key + "_joint")
        lines.append(f'    "{urdf_name}",')
    lines.append("]")
    lines.append("")

    # Trajectory points: list of position arrays (one per frame)
    lines.append("# Trajectory: POSITIONS[frame_index] = [angle_for_each_joint]")
    lines.append("POSITIONS = [")
    for i in range(frame_count):
        row = [joints[k][i] for k in active_keys]
        row_str = ", ".join(f"{v: .6f}" for v in row)
        lines.append(f"    [{row_str}],")
    lines.append("]")
    lines.append("")

    # Also export per-joint arrays for convenience
    lines.append("# Per-joint trajectory arrays (for plotting / debugging)")
    for key in active_keys:
        urdf_name = URDF_JOINT_MAP.get(key, key + "_joint")
        var_name = urdf_name.upper().replace("_JOINT", "")
        lines.append(f"{var_name} = {format_array(joints[key])}")
        lines.append("")

    # Cannon (informational)
    lines.append("# Passive cannon angles (informational — not sent to motors)")
    for prefix in LEG_PREFIXES:
        cannon_key = f"{prefix}_cannon"
        if cannon_key in joints:
            var_name = f"{prefix.upper()}_CANNON"
            lines.append(f"{var_name} = {format_array(joints[cannon_key])}")
            lines.append("")

    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------
#  Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Convert exported gait JSON to ROS2 trajectory Python module."
    )
    parser.add_argument("input", help="Path to gait JSON file")
    parser.add_argument("-o", "--output", default=None,
                        help="Output .py file (default: <input_stem>_trajectory.py)")
    parser.add_argument("--verify", action="store_true",
                        help="Verify cannon linkage formulas and report discrepancies")
    parser.add_argument("--tolerance", type=float, default=0.05,
                        help="Tolerance (rad) for cannon verification (default: 0.05)")
    args = parser.parse_args()

    # Load JSON
    with open(args.input, 'r') as f:
        gait = json.load(f)

    print(f"[load_gait] Loaded '{gait.get('name', '?')}': "
          f"{gait['frame_count']} frames @ {gait.get('fps', '?')} fps, "
          f"{len(gait['joints'])} joints")

    # Verify cannon linkage
    if args.verify:
        warnings = verify_cannon_linkage(gait, args.tolerance)
        if warnings:
            print(f"[load_gait] ⚠ Cannon linkage discrepancies ({len(warnings)}):")
            for w in warnings[:20]:
                print(w)
            if len(warnings) > 20:
                print(f"  ... and {len(warnings) - 20} more")
        else:
            print("[load_gait] ✓ All cannon angles match linkage formulas.")

    # Recompute cannon from linkage (authoritative)
    gait = recompute_cannon(gait)

    # Determine output path
    if args.output:
        out_path = args.output
    else:
        stem = os.path.splitext(os.path.basename(args.input))[0]
        out_path = os.path.join(os.path.dirname(args.input) or ".", f"{stem}_trajectory.py")

    # Generate and write
    module_src = generate_trajectory_module(gait)
    with open(out_path, 'w') as f:
        f.write(module_src)

    print(f"[load_gait] Wrote → {out_path}")


if __name__ == "__main__":
    main()
