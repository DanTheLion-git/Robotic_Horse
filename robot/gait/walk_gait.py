"""
Bovine gait planner for the Highland Cow quadruped.

Two gaits modeled after real cattle biomechanics:

WALK — 4-beat lateral sequence
  Footfall order: RL → FL → RR → FR  (each 25% apart)
  Duty factor: 0.70 (each foot on ground 70% of cycle)
  Swing fraction: 0.30
  Period: 1.1s (≈0.9 Hz stride frequency)
  Speed: 0.25–0.35 m/s

TROT — 2-beat diagonal pairs
  Diagonal pairs: FL+RR, FR+RL
  Duty factor: 0.50 (each foot on ground 50% of cycle)
  Swing fraction: 0.50
  Period: 0.7s (≈1.4 Hz stride frequency)
  Speed: 0.50–0.70 m/s

Bovine swing trajectory — 3-phase arc:
  LIFT  (0–30%):  Hoof rises from liftoff position
  CARRY (30–70%): Hoof at peak height, sweeps forward
  PLANT (70–100%): Hoof descends to touchdown position

Weight distribution: 55% front / 45% rear
Front legs step slightly higher (load-bearing, need clearance).
Rear legs overtrack slightly (propulsive role).

Highland Cow build — short thick legs, deep crouch stance.
"""

import math
import numpy as np
from robot.kinematics.leg_kinematics import (
    inverse_kinematics,
    ANKLE_HEIGHT_FRONT, ANKLE_HEIGHT_REAR,
    L1_FRONT, L2_FRONT, L1_REAR, L2_REAR,
    cannon_angle_front, cannon_angle_rear,
)


# ── Leg identifiers ───────────────────────────────────────────────
LEG_NAMES = ["fl", "fr", "rl", "rr"]
FRONT_LEGS = {"fl", "fr"}
REAR_LEGS  = {"rl", "rr"}


# ── WALK gait — 4-beat lateral sequence ───────────────────────────
# Real cattle: LH → LF → RH → RF, each 25% apart
WALK = {
    'name': 'WALK',
    'phase_offsets': {'rl': 0.0, 'fl': 0.25, 'rr': 0.50, 'fr': 0.75},
    'swing_frac': 0.30,        # 30% swing / 70% stance (duty factor 0.70)
    'period': 1.1,             # seconds per full stride cycle
    'step_height_front': 0.10, # front legs lift higher (weight-bearing clearance)
    'step_height_rear': 0.08,  # rear legs slightly lower arc
    'stride_length': 0.65,     # full stride at steady-state speed [m]
    'min_stride': 0.15,        # full stride at startup / acceleration [m]
    'speed': 0.80,             # target walking speed [m/s] (real cow ~0.8-1.0)
}

# ── TROT gait — 2-beat diagonal ───────────────────────────────────
# Diagonal pairs: FL+RR, FR+RL
TROT = {
    'name': 'TROT',
    'phase_offsets': {'fl': 0.0, 'rr': 0.0, 'fr': 0.50, 'rl': 0.50},
    'swing_frac': 0.50,        # 50% swing / 50% stance (duty factor 0.50)
    'period': 0.70,            # faster cadence for trot
    'step_height_front': 0.14, # higher clearance needed at trot speed
    'step_height_rear': 0.12,
    'stride_length': 0.85,     # full stride at steady trot [m]
    'min_stride': 0.22,        # full stride during trot startup [m]
    'speed': 1.50,             # target trotting speed [m/s] (real cow ~1.5-2.5)
}

GAIT_SEQUENCE = [WALK, TROT]

# ── Gait parameters ───────────────────────────────────────────────
MAX_STEP       = 0.45    # hard cap on stride half-length [m]

# ── Overtracking ──────────────────────────────────────────────────
# Rear hooves land slightly ahead of where the ipsilateral front hoof was
OVERTRACK_FRONT = 0.0    # front legs: no offset
OVERTRACK_REAR  = 0.01   # rear legs: 1cm forward of front footprint


def _swing_trajectory(phase: float, step_height: float,
                      half_stride: float,
                      ankle_height: float) -> tuple[float, float]:
    """
    3-phase bovine swing arc.

    Phase 0.00–0.30: LIFT   — hoof rises from liftoff position
    Phase 0.30–0.70: CARRY  — hoof at peak height, sweeps forward
    Phase 0.70–1.00: PLANT  — hoof descends to touchdown position

    Parameters
    ----------
    phase        : swing progress [0, 1)
    step_height  : vertical clearance [m]
    half_stride  : horizontal half-stride [m]
    ankle_height : IK target height for this leg type [m]

    Returns (foot_x, foot_z) in hip frame — targeting ankle position.
    """
    LIFT_END  = 0.30
    CARRY_END = 0.70

    if phase < LIFT_END:
        t = phase / LIFT_END
        lift = step_height * math.sin(math.pi * 0.5 * t)
        t_fwd = t * 0.15
        foot_x = -half_stride + 2.0 * half_stride * t_fwd
    elif phase < CARRY_END:
        t = (phase - LIFT_END) / (CARRY_END - LIFT_END)
        lift = step_height
        t_fwd = 0.15 + t * 0.70
        foot_x = -half_stride + 2.0 * half_stride * t_fwd
    else:
        t = (phase - CARRY_END) / (1.0 - CARRY_END)
        lift = step_height * math.cos(math.pi * 0.5 * t)
        t_fwd = 0.85 + t * 0.15
        foot_x = -half_stride + 2.0 * half_stride * t_fwd

    foot_x = max(-MAX_STEP, min(MAX_STEP, foot_x))
    foot_z = -(ankle_height - lift)

    return foot_x, foot_z


def _stance_trajectory(phase: float, half_stride: float,
                       ankle_height: float) -> tuple[float, float]:
    """
    Stance phase: foot fixed on ground, body moves forward over it.

    The foot sweeps from +half_stride backward to -half_stride as the
    body advances. Foot pressed slightly into ground for firm contact.

    Parameters
    ----------
    phase       : stance progress [0, 1)
    half_stride : horizontal half-stride [m]

    Returns (foot_x, foot_z) in hip frame.
    """
    foot_x = half_stride * (1.0 - 2.0 * phase)
    foot_z = -(ankle_height + 0.015)  # press 15mm into ground for contact
    return foot_x, foot_z


def joint_angles_at_phase(leg: str, global_phase: float,
                          gait: dict = None,
                          speed_fraction: float = 1.0) -> tuple[float, float, float]:
    """
    Return (theta_hip, theta_knee, theta_cannon) for a given leg at a gait phase.

    Parameters
    ----------
    leg            : 'fl', 'fr', 'rl', 'rr'
    global_phase   : gait phase ∈ [0, 1)
    gait           : gait parameters dict (defaults to WALK)
    speed_fraction : 0→1, controls stride length (0=min_stride, 1=full stride).
                     Models acceleration: first steps are short, steady state is long.

    Returns
    -------
    (theta_hip, theta_knee, theta_cannon) [rad]
    """
    if gait is None:
        gait = WALK

    is_rear = leg in REAR_LEGS
    elbow_up = is_rear
    ankle_h = ANKLE_HEIGHT_REAR if is_rear else ANKLE_HEIGHT_FRONT

    # Speed-dependent stride: ramp from min_stride to full stride_length
    sf = max(0.0, min(1.0, speed_fraction))
    stride = gait['min_stride'] + (gait['stride_length'] - gait['min_stride']) * sf

    # Step height also scales with speed (70% at startup, 100% at full speed)
    height_scale = 0.70 + 0.30 * sf
    step_h_base = gait['step_height_rear'] if is_rear else gait['step_height_front']
    step_h = step_h_base * height_scale

    overtrack = OVERTRACK_REAR if is_rear else OVERTRACK_FRONT
    half_stride = stride / 2.0 + overtrack

    local_phase = (global_phase - gait['phase_offsets'][leg]) % 1.0

    if local_phase < gait['swing_frac']:
        swing_phase = local_phase / gait['swing_frac']
        foot_x, foot_z = _swing_trajectory(swing_phase, step_h, half_stride, ankle_h)
    else:
        stance_phase = (local_phase - gait['swing_frac']) / (1.0 - gait['swing_frac'])
        foot_x, foot_z = _stance_trajectory(stance_phase, half_stride, ankle_h)

    # Use leg-specific segment lengths for IK
    l1 = L1_REAR if is_rear else L1_FRONT
    l2 = L2_REAR if is_rear else L2_FRONT

    try:
        theta_hip, theta_knee = inverse_kinematics(foot_x, foot_z,
                                                   elbow_up=elbow_up,
                                                   is_rear=is_rear)
    except ValueError:
        max_reach = l1 + l2
        r = math.sqrt(foot_x**2 + foot_z**2)
        scale = 0.90 * max_reach / r
        theta_hip, theta_knee = inverse_kinematics(foot_x * scale, foot_z * scale,
                                                   elbow_up=elbow_up,
                                                   is_rear=is_rear)

    # Cannon angle: reciprocal apparatus for rear, passive linkage for front
    if is_rear:
        theta_cannon = cannon_angle_rear(theta_knee)
    else:
        theta_cannon = cannon_angle_front(theta_hip, theta_knee)

    return theta_hip, theta_knee, theta_cannon


def generate_gait_trajectory(n_steps: int = 200,
                             gait: dict = None,
                             speed_fraction: float = 1.0,
                             ramp_cycles: int = 0) -> dict:
    """
    Generate a full gait cycle trajectory for all four legs.

    Parameters
    ----------
    n_steps        : number of time steps per full gait cycle
    gait           : gait parameters dict (defaults to WALK)
    speed_fraction : constant speed fraction (used when ramp_cycles=0)
    ramp_cycles    : if >0, ramp speed_fraction from 0.3→1.0 over this many
                     stride cycles. Overrides speed_fraction with a per-step ramp.

    Returns
    -------
    dict mapping leg name → {
        'theta_hip': array, 'theta_knee': array, 'theta_cannon': array,
        'phase': array
    }
    """
    if gait is None:
        gait = WALK

    phases = np.linspace(0, 1, n_steps, endpoint=False)
    result = {}

    for leg in LEG_NAMES:
        hips    = np.zeros(n_steps)
        knees   = np.zeros(n_steps)
        cannons = np.zeros(n_steps)

        for i, p in enumerate(phases):
            if ramp_cycles > 0:
                # Ramp from 0.3 to 1.0 over ramp_cycles full cycles
                cycle_progress = p  # within one cycle
                sf = 0.3 + 0.7 * min(1.0, cycle_progress * ramp_cycles)
            else:
                sf = speed_fraction

            hips[i], knees[i], cannons[i] = joint_angles_at_phase(
                leg, p, gait, speed_fraction=sf)

        result[leg] = {
            "theta_hip":    hips,
            "theta_knee":   knees,
            "theta_cannon": cannons,
            "phase":        phases,
        }

    return result
