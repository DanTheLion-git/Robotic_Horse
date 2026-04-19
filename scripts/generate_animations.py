"""
generate_animations.py
──────────────────────
Generate placeholder animation JSON files for the Robotic Horse blendspace.

Run from the repository root:
    python scripts/generate_animations.py

Produces 6 animation files in:
    ros2_ws/src/robotic_horse_control/animations/

Animation    Speed   Angular   Description
──────────   ──────  ────────  ───────────────────────────────────────────
idle         0.0     0.0       Standing still with subtle weight-shift bob
walk_fwd     1.0     0.0       Trot gait, straight ahead
walk_left    1.0    -1.0       Arc-left trot (left legs shorter stride)
walk_right   1.0     1.0       Arc-right trot (right legs longer stride)
turn_left    0.0    -1.0       Spin in place, left
turn_right   0.0     1.0       Spin in place, right

These are computed from the inverse-kinematics model.  Once you have
real Blender animations, replace these files with the exported JSON.
"""

import sys
import os
import json
import math

# ── Make robot.* importable from repo root ─────────────────────────
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot.kinematics.leg_kinematics import inverse_kinematics

OUT_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "ros2_ws", "src", "robotic_horse_control", "animations",
)

# ── Shared constants (150 cm Highland Cow) ───────────────────────
# Front leg segments
L1_FRONT     = 0.38
L2_FRONT     = 0.34
L3_FRONT     = 0.18
FOOT_R_FRONT = 0.05

# Rear leg segments
L1_REAR      = 0.50
L2_REAR      = 0.45
L3_REAR      = 0.22
FOOT_R_REAR  = 0.06

CANNON_LEAN  = 0.08
BODY_HEIGHT  = 1.08   # body centre Z [m]
ANKLE_HEIGHT_FRONT = 0.70   # front ankle-to-hip [m]
ANKLE_HEIGHT_REAR  = 0.92   # rear ankle-to-hip [m]
BODY_WIDTH   = 0.80   # 2 × 0.40 m lateral hip offset
N_FRAMES     = 60     # frames per animation
DURATION     = 1.0    # seconds per full gait cycle
SWING_FRAC   = 0.40
MAX_STEP     = 0.35   # hard cap on stride half-length [m]
STEP_LENGTH  = 0.50   # full stride at steady walk [m]

JOINT_NAMES = [
    "fl_hip_joint", "fl_thigh_joint", "fl_knee_joint", "fl_cannon_joint",
    "fr_hip_joint", "fr_thigh_joint", "fr_knee_joint", "fr_cannon_joint",
    "rl_hip_joint", "rl_thigh_joint", "rl_knee_joint", "rl_cannon_joint",
    "rr_hip_joint", "rr_thigh_joint", "rr_knee_joint", "rr_cannon_joint",
]

# Trot phase offsets: FL+RR move together, FR+RL move together
TROT_PHASE = {"fl": 0.0, "fr": 0.5, "rl": 0.5, "rr": 0.0}
# Walk phase offsets: 4-beat lateral sequence RL→FL→RR→FR
WALK_PHASE = {"rl": 0.0, "fl": 0.25, "rr": 0.50, "fr": 0.75}

FRONT_LEGS = ("fl", "fr")
REAR_LEGS  = ("rl", "rr")

# Reciprocal apparatus (rear legs)
RECIP_RATIO  = 0.85
RECIP_OFFSET = -0.05 + RECIP_RATIO * 0.505  # ≈ 0.379


def _cannon_angle(leg: str, thigh: float, knee: float) -> float:
    """Front = passive linkage, Rear = reciprocal apparatus."""
    if leg in REAR_LEGS:
        return RECIP_OFFSET - RECIP_RATIO * knee
    return CANNON_LEAN - (thigh + knee)


# ── Foot trajectory helpers ─────────────────────────────────────────

def foot_target(leg: str, phase: float, step_length: float, step_height: float,
                body_height: float = None, phase_offsets=None):
    """Return (foot_x, foot_z) targeting the ankle for a given leg at a gait phase."""
    if body_height is None:
        body_height = ANKLE_HEIGHT_REAR if leg in REAR_LEGS else ANKLE_HEIGHT_FRONT
    offsets = phase_offsets or WALK_PHASE
    local = (phase - offsets[leg]) % 1.0
    if local < SWING_FRAC:
        p = local / SWING_FRAC
        x = -step_length / 2 + step_length * p
        x = max(-MAX_STEP, min(MAX_STEP, x))
        z = -(body_height - step_height * math.sin(math.pi * p))
    else:
        p = (local - SWING_FRAC) / (1.0 - SWING_FRAC)
        x = step_length / 2 - step_length * p
        x = max(-MAX_STEP, min(MAX_STEP, x))
        z = -body_height
    return x, z


def ik_safe(foot_x: float, foot_z: float, leg: str = 'fl'):
    """IK with leg-specific dimensions and clamping."""
    if leg in REAR_LEGS:
        L1, L2, ank_h = L1_REAR, L2_REAR, ANKLE_HEIGHT_REAR
    else:
        L1, L2, ank_h = L1_FRONT, L2_FRONT, ANKLE_HEIGHT_FRONT
    r = math.sqrt(foot_x**2 + foot_z**2)
    max_r = L1 + L2 - 0.01
    if r > max_r:
        scale = max_r / r
        foot_x, foot_z = foot_x * scale, foot_z * scale
    try:
        return inverse_kinematics(foot_x, foot_z)
    except ValueError:
        return inverse_kinematics(0.0, -ank_h)


# ── Per-leg step lengths for differential gait ─────────────────────

def leg_step_lengths(speed: float, angular_rate: float,
                     base_step: float = STEP_LENGTH):
    """
    Compute effective step length per leg for a given speed + angular rate.
    angular_rate > 0 → turn right (right legs shorter, left longer)
    """
    half_w = BODY_WIDTH / 2
    sl = {
        "fl": max(0.0, speed + angular_rate * half_w) * base_step,
        "fr": max(0.0, speed - angular_rate * half_w) * base_step,
        "rl": max(0.0, speed + angular_rate * half_w) * base_step,
        "rr": max(0.0, speed - angular_rate * half_w) * base_step,
    }
    return sl


# ── Animation builders ──────────────────────────────────────────────

def build_idle(n_frames: int = N_FRAMES):
    """
    Static standing pose with a gentle vertical body-bob (±10 mm).
    Produced by slightly varying the knee angle over a slow period.
    """
    frames = []
    for i in range(n_frames):
        phase = i / n_frames
        bob = 0.010 * math.sin(2 * math.pi * phase)   # ±10 mm
        row = []
        for leg in ("fl", "fr", "rl", "rr"):
            ank_h = ANKLE_HEIGHT_REAR if leg in REAR_LEGS else ANKLE_HEIGHT_FRONT
            th, tk = ik_safe(0.0, -(ank_h + bob), leg=leg)
            cannon = _cannon_angle(leg, th, tk)
            row += [0.0, th, tk, cannon]
        frames.append(row)
    return frames


def build_parametric(speed: float, angular_rate: float,
                      step_height_front: float = 0.10,
                      step_height_rear: float = 0.08,
                      n_frames: int = N_FRAMES,
                      phase_offsets=None):
    """General gait with differential step lengths and front/rear step heights."""
    sl = leg_step_lengths(speed, angular_rate)
    frames = []
    for i in range(n_frames):
        phase = i / n_frames
        row = []
        for leg in ("fl", "fr", "rl", "rr"):
            sh = step_height_rear if leg in REAR_LEGS else step_height_front
            fx, fz = foot_target(leg, phase, sl[leg], sh, phase_offsets=phase_offsets)
            th, tk = ik_safe(fx, fz, leg=leg)
            cannon = _cannon_angle(leg, th, tk)
            row += [0.0, th, tk, cannon]
        frames.append(row)
    return frames


def build_turn(angular_rate: float, n_frames: int = N_FRAMES):
    """
    In-place turning: speed = 0 but angular_rate ≠ 0.
    Each leg's step_length is driven purely by the tangential speed
    of the wheel-like rotation.  Left/right sides move in opposite directions.
    """
    half_w = BODY_WIDTH / 2
    stride_inner = abs(angular_rate) * half_w * 0.8
    frames = []
    for i in range(n_frames):
        phase = i / n_frames
        row = []
        for leg in ("fl", "fr", "rl", "rr"):
            side = "left" if leg in ("fl", "rl") else "right"
            if angular_rate > 0:
                leg_phase = phase if side == "left" else (phase + 0.5) % 1.0
            else:
                leg_phase = phase if side == "right" else (phase + 0.5) % 1.0
            sh = 0.07 if leg in REAR_LEGS else 0.08
            fx, fz = foot_target(leg, leg_phase, stride_inner, sh)
            th, tk = ik_safe(fx, fz, leg=leg)
            cannon = _cannon_angle(leg, th, tk)
            row += [0.0, th, tk, cannon]
        frames.append(row)
    return frames


# ── Serialise to JSON ───────────────────────────────────────────────

def save_animation(name: str, description: str, frames: list,
                   speed: float, angular_rate: float,
                   duration: float = DURATION):
    data = {
        "name":          name,
        "description":   description,
        "speed":         speed,
        "angular_rate":  angular_rate,
        "duration":      duration,
        "n_frames":      len(frames),
        "joint_names":   JOINT_NAMES,
        "frames":        [[round(v, 6) for v in row] for row in frames],
    }
    path = os.path.join(OUT_DIR, f"{name}.json")
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"  Saved {name}.json  ({len(frames)} frames)")


# ── Main ─────────────────────────────────────────────────────────────

def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    print(f"Generating animations → {OUT_DIR}")
    print()

    save_animation("idle",       "Standing still with gentle weight-shift bob",
                   build_idle(),              speed=0.0, angular_rate=0.0)

    save_animation("walk_forward", "Bovine walk — 4-beat lateral sequence",
                   build_parametric(1.0, 0.0, phase_offsets=WALK_PHASE),
                   speed=0.5, angular_rate=0.0, duration=1.1)

    save_animation("trot_forward", "Bovine trot — diagonal pair gait",
                   build_parametric(1.0, 0.0, phase_offsets=TROT_PHASE),
                   speed=1.0, angular_rate=0.0, duration=0.7)

    save_animation("walk_left",  "Arc-left walk — left legs shorter stride",
                   build_parametric(1.0, -1.0, phase_offsets=WALK_PHASE),
                   speed=0.5, angular_rate=-1.0, duration=1.1)

    save_animation("walk_right", "Arc-right walk — right legs shorter stride",
                   build_parametric(1.0,  1.0, phase_offsets=WALK_PHASE),
                   speed=0.5, angular_rate=1.0, duration=1.1)

    save_animation("turn_left",  "Spin in place to the left",
                   build_turn(-1.0),           speed=0.0, angular_rate=-1.0)

    save_animation("turn_right", "Spin in place to the right",
                   build_turn(1.0),            speed=0.0, angular_rate=1.0)

    print()
    print("Done.  7 animations generated.")


if __name__ == "__main__":
    main()
