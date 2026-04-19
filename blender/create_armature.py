#!/usr/bin/env python3
"""
create_armature.py — Blender Python script to build a Highland Cow quadruped armature.

Run with:
    blender --background --python create_armature.py
  or open Blender → Scripting tab → Run Script

Coordinate convention mapping (URDF → Blender):
    URDF  X (forward)  →  Blender -Y  (Blender's forward is -Y in front view)
    URDF  Y (left)     →  Blender  X
    URDF  Z (up)       →  Blender  Z

All rotations labelled "pitch" are about the local Y-axis in URDF, which maps
to the bone's local X-axis in Blender (perpendicular to the bone, in the
sagittal plane).  Hip yaw (Z in URDF) maps to rotation about the bone's
local Z-axis in Blender (roll-axis).

Compatible with Blender 3.6+ and 4.0+.
"""

import bpy
import math
import os
from mathutils import Vector, Matrix, Euler

# ---------------------------------------------------------------------------
#  Robot parameters (metres / radians)
# ---------------------------------------------------------------------------

BODY_LENGTH = 1.80
BODY_WIDTH = 0.88
BODY_MAIN_HEIGHT = 0.75
BODY_CENTER_Z = 1.12  # above ground at neutral stance

# Hip positions relative to body centre (URDF frame: X fwd, Y left, Z up)
HIP_OFFSETS = {
    "FL": Vector((+0.65, +0.44, -0.15)),
    "FR": Vector((+0.65, -0.44, -0.15)),
    "RL": Vector((-0.65, +0.40, +0.12)),
    "RR": Vector((-0.65, -0.40, +0.12)),
}

# Front leg segments
L1_FRONT = 0.38   # humerus
L2_FRONT = 0.34   # radius
L3_FRONT = 0.18   # metacarpal / cannon
FOOT_R_FRONT = 0.05

# Rear leg segments
L1_REAR = 0.50    # femur
L2_REAR = 0.45    # tibia
L3_REAR = 0.22    # metatarsal / cannon
FOOT_R_REAR = 0.06

# Neutral joint angles (radians)
FRONT_THIGH_NEUTRAL = 0.220
FRONT_KNEE_NEUTRAL = -0.466
REAR_THIGH_NEUTRAL = -0.236
REAR_KNEE_NEUTRAL = 0.499

# Cannon linkage formulas
# Front: cannon_angle = 0.08 - (thigh + knee)
# Rear:  cannon_angle = 0.3792 - 0.85 * knee_angle

NECK_LENGTH = 0.40
SKULL_LENGTH = 0.35
JAW_LENGTH = 0.20
HIP_BONE_LEN = 0.08  # visual marker bone at hip


def cannon_angle_front(thigh, knee):
    return 0.08 - (thigh + knee)


def cannon_angle_rear(knee):
    return 0.3792 - 0.85 * knee


# ---------------------------------------------------------------------------
#  Coordinate helpers
# ---------------------------------------------------------------------------

def urdf_to_blender(v):
    """Convert a Vector from URDF (X-fwd, Y-left, Z-up) to Blender (Y-fwd=-Y, X=left, Z=up)."""
    return Vector(( v.y,  -v.x,  v.z))


def pitch_direction(angle, is_front):
    """
    Return a unit-ish direction vector in Blender space for a bone whose
    pitch angle is *angle* radians (positive = forward tilt in URDF).

    For a downward-hanging leg segment, 0 rad means straight down (-Z).
    A positive pitch tilts the distal end forward (URDF +X → Blender -Y).

    Front legs: elbow-DOWN → knee bends backward (positive thigh tilts
    forward, negative knee tilts backward).
    Rear legs: elbow-UP → knee bends forward.
    """
    # In the sagittal plane (Blender YZ plane for a leg pointing down):
    #   down component = -cos(angle)  along Z
    #   forward component = -sin(angle) along Y  (Blender -Y = URDF +X)
    dz = -math.cos(angle)
    dy = -math.sin(angle)  # forward in Blender = -Y
    return Vector((0.0, dy, dz))


# ---------------------------------------------------------------------------
#  Scene setup
# ---------------------------------------------------------------------------

def clear_scene():
    """Remove all objects from the scene."""
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    # Purge orphan data
    for block in bpy.data.meshes:
        bpy.data.meshes.remove(block)
    for block in bpy.data.armatures:
        bpy.data.armatures.remove(block)


# ---------------------------------------------------------------------------
#  Build armature
# ---------------------------------------------------------------------------

def create_armature():
    clear_scene()

    # Create armature data-block and object
    arm_data = bpy.data.armatures.new("HighlandCow")
    arm_obj = bpy.data.objects.new("HighlandCow", arm_data)
    arm_obj.location = (0.0, 0.0, BODY_CENTER_Z)
    bpy.context.collection.objects.link(arm_obj)

    # Make active & enter edit mode
    bpy.context.view_layer.objects.active = arm_obj
    bpy.ops.object.mode_set(mode='EDIT')

    edit_bones = arm_data.edit_bones

    # --- Body bone -----------------------------------------------------------
    # Horizontal along Blender -Y (URDF +X = forward).
    # Head at rear, tail at front so the bone points forward.
    body = edit_bones.new("body")
    body.head = Vector((0.0, BODY_LENGTH / 2, 0.0))   # rear
    body.tail = Vector((0.0, -BODY_LENGTH / 2, 0.0))   # front
    body.roll = 0.0

    # --- Legs ----------------------------------------------------------------
    legs = {
        "FL": {"side": "L", "pos": "F"},
        "FR": {"side": "R", "pos": "F"},
        "RL": {"side": "L", "pos": "R"},
        "RR": {"side": "R", "pos": "R"},
    }

    for leg_name, info in legs.items():
        is_front = info["pos"] == "F"
        side_sign = 1.0 if info["side"] == "L" else -1.0

        # Segment lengths
        L1 = L1_FRONT if is_front else L1_REAR
        L2 = L2_FRONT if is_front else L2_REAR
        L3 = L3_FRONT if is_front else L3_REAR
        foot_r = FOOT_R_FRONT if is_front else FOOT_R_REAR

        # Neutral angles
        thigh_ang = FRONT_THIGH_NEUTRAL if is_front else REAR_THIGH_NEUTRAL
        knee_ang = FRONT_KNEE_NEUTRAL if is_front else REAR_KNEE_NEUTRAL
        if is_front:
            cannon_ang = cannon_angle_front(thigh_ang, knee_ang)
        else:
            cannon_ang = cannon_angle_rear(knee_ang)

        # Hip position in Blender coords (relative to armature origin = body centre)
        hip_urdf = HIP_OFFSETS[leg_name]
        hip_pos = urdf_to_blender(hip_urdf)

        # --- hip bone (short, connects body to thigh) ---
        hip_bone = edit_bones.new(f"{leg_name}_hip")
        hip_bone.head = hip_pos.copy()
        # Point the hip bone outward (lateral) so its Z-axis can serve as yaw
        lateral_dir = Vector((side_sign, 0.0, 0.0)).normalized()
        hip_bone.tail = hip_pos + lateral_dir * HIP_BONE_LEN
        hip_bone.parent = body
        hip_bone.use_connect = False
        hip_bone.roll = 0.0

        # --- thigh bone ---
        thigh_bone = edit_bones.new(f"{leg_name}_thigh")
        thigh_bone.head = hip_bone.tail.copy()

        # Cumulative pitch from vertical: thigh_ang
        # Direction in Blender YZ: angle measured from -Z toward -Y (forward)
        thigh_dir = pitch_direction(thigh_ang, is_front)
        thigh_bone.tail = thigh_bone.head + thigh_dir.normalized() * L1
        thigh_bone.parent = hip_bone
        thigh_bone.use_connect = True
        # Roll so that local-X is the pitch axis (perpendicular to sagittal plane)
        thigh_bone.roll = 0.0 if info["side"] == "L" else math.pi

        # --- knee bone ---
        knee_bone = edit_bones.new(f"{leg_name}_knee")
        knee_bone.head = thigh_bone.tail.copy()

        # Cumulative pitch = thigh + knee relative to parent
        cumulative_knee = thigh_ang + knee_ang
        knee_dir = pitch_direction(cumulative_knee, is_front)
        knee_bone.tail = knee_bone.head + knee_dir.normalized() * L2
        knee_bone.parent = thigh_bone
        knee_bone.use_connect = True
        knee_bone.roll = 0.0 if info["side"] == "L" else math.pi

        # --- cannon bone ---
        cannon_bone = edit_bones.new(f"{leg_name}_cannon")
        cannon_bone.head = knee_bone.tail.copy()

        cumulative_cannon = cumulative_knee + cannon_ang
        cannon_dir = pitch_direction(cumulative_cannon, is_front)
        cannon_bone.tail = cannon_bone.head + cannon_dir.normalized() * L3
        cannon_bone.parent = knee_bone
        cannon_bone.use_connect = True
        cannon_bone.roll = 0.0 if info["side"] == "L" else math.pi

        # --- hoof bone (tiny marker) ---
        hoof_bone = edit_bones.new(f"{leg_name}_hoof")
        hoof_bone.head = cannon_bone.tail.copy()
        hoof_bone.tail = cannon_bone.tail + Vector((0.0, 0.0, -foot_r))
        hoof_bone.parent = cannon_bone
        hoof_bone.use_connect = True
        hoof_bone.roll = 0.0

    # --- Neck / skull / jaw --------------------------------------------------
    # Neck originates from the front of the body, angled upward ~40°
    neck_start = urdf_to_blender(Vector((BODY_LENGTH / 2 * 0.9, 0.0, 0.10)))
    neck = edit_bones.new("neck")
    neck.head = neck_start.copy()
    neck_angle = math.radians(40)
    neck.tail = neck.head + Vector((0.0, -math.sin(neck_angle), math.cos(neck_angle))) * NECK_LENGTH
    neck.parent = body
    neck.use_connect = False
    neck.roll = 0.0

    skull = edit_bones.new("skull")
    skull.head = neck.tail.copy()
    skull_angle = math.radians(15)
    skull.tail = skull.head + Vector((0.0, -math.cos(skull_angle), -math.sin(skull_angle))) * SKULL_LENGTH
    skull.parent = neck
    skull.use_connect = True
    skull.roll = 0.0

    jaw = edit_bones.new("jaw")
    jaw.head = skull.head.copy()
    jaw.tail = jaw.head + Vector((0.0, -math.cos(math.radians(25)), -math.sin(math.radians(25)))) * JAW_LENGTH
    jaw.parent = skull
    jaw.use_connect = False
    jaw.roll = 0.0

    # --- Finalise edit mode --------------------------------------------------
    bpy.ops.object.mode_set(mode='OBJECT')

    # ---- Display settings ---------------------------------------------------
    arm_data.display_type = 'STICK'
    arm_obj.show_in_front = True

    print("[create_armature] Armature 'HighlandCow' created successfully.")
    print(f"  Body centre at Z = {BODY_CENTER_Z}m")
    print(f"  Front cannon neutral angle = {cannon_angle_front(FRONT_THIGH_NEUTRAL, FRONT_KNEE_NEUTRAL):.4f} rad")
    print(f"  Rear  cannon neutral angle = {cannon_angle_rear(REAR_KNEE_NEUTRAL):.4f} rad")

    return arm_obj


# ---------------------------------------------------------------------------
#  Save .blend
# ---------------------------------------------------------------------------

def save_blend(arm_obj):
    """Save to the same directory as this script."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filepath = os.path.join(script_dir, "highland_cow_armature.blend")
    bpy.ops.wm.save_as_mainfile(filepath=filepath)
    print(f"[create_armature] Saved → {filepath}")


# ---------------------------------------------------------------------------
#  Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    arm_obj = create_armature()
    save_blend(arm_obj)
