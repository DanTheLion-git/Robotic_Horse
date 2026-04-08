"""
Trot gait planner for the quadruped.

A trot moves diagonal leg pairs simultaneously:
  Phase 0: FL + RR swing,  FR + RL stance
  Phase 1: FR + RL swing,  FL + RR stance

Each leg cycles through:
  STANCE  – foot on ground, hip moves body forward
  LIFT    – knee bends (raises foot)
  SWING   – hip swings forward
  LOWER   – knee extends (lowers foot to ground)

Joint angle trajectories are parameterised by phase [0, 1).
"""

import numpy as np
from robot.kinematics.leg_kinematics import inverse_kinematics


# ── Gait parameters ────────────────────────────────────────────────
STEP_LENGTH   = 0.20    # stride length  [m]
STEP_HEIGHT   = 0.12    # foot lift height  [m]
BODY_HEIGHT   = 0.90    # nominal hip-to-ground distance  [m]
SWING_FRACTION = 0.40   # fraction of gait cycle spent swinging

LEG_NAMES = ["fl", "fr", "rl", "rr"]

# Diagonal pairs for trot gait
TROT_PAIRS = [
    {"swing": ["fl", "rr"], "stance": ["fr", "rl"]},
    {"swing": ["fr", "rl"], "stance": ["fl", "rr"]},
]

# Phase offset per leg (trot: diagonals 0.5 apart)
PHASE_OFFSET = {
    "fl": 0.0,
    "rr": 0.0,
    "fr": 0.5,
    "rl": 0.5,
}


def _stance_hip_angle(phase: float) -> float:
    """
    Hip angle during stance: sweeps from +step/2 to −step/2 in hip frame.
    phase ∈ [0, 1)
    """
    half = STEP_LENGTH / (2 * BODY_HEIGHT)
    return half - 2 * half * phase   # linear sweep


def _swing_trajectory(phase: float) -> tuple[float, float]:
    """
    Foot trajectory during swing phase.

    phase ∈ [0, 1) within the swing sub-phase.
    Returns (foot_x, foot_z) in hip frame.
    """
    # Horizontal: half-sine from −step/2 to +step/2
    foot_x = -STEP_LENGTH / 2 + STEP_LENGTH * phase

    # Vertical: raised by a half-sine arc
    foot_z = -(BODY_HEIGHT - STEP_HEIGHT * np.sin(np.pi * phase))

    return foot_x, foot_z


def joint_angles_at_phase(leg: str, global_phase: float) -> tuple[float, float]:
    """
    Return (theta_hip, theta_knee) for a given leg at a given gait phase.

    Parameters
    ----------
    leg          : one of 'fl', 'fr', 'rl', 'rr'
    global_phase : gait phase ∈ [0, 1)

    Returns
    -------
    (theta_hip, theta_knee)  [rad]
    """
    local_phase = (global_phase - PHASE_OFFSET[leg]) % 1.0

    if local_phase < SWING_FRACTION:
        # Swing phase
        swing_phase = local_phase / SWING_FRACTION
        foot_x, foot_z = _swing_trajectory(swing_phase)
    else:
        # Stance phase
        stance_phase = (local_phase - SWING_FRACTION) / (1.0 - SWING_FRACTION)
        foot_x = _stance_hip_angle(stance_phase) * BODY_HEIGHT   # approx x
        foot_z = -BODY_HEIGHT

    try:
        return inverse_kinematics(foot_x, foot_z)
    except ValueError:
        # Clamp to reachable workspace edge
        r = np.sqrt(foot_x**2 + foot_z**2)
        scale = (0.90 / r)    # 90 % of max reach
        return inverse_kinematics(foot_x * scale, foot_z * scale)


def generate_gait_trajectory(n_steps: int = 200) -> dict:
    """
    Generate a full gait cycle trajectory for all four legs.

    Parameters
    ----------
    n_steps : number of time steps per full gait cycle

    Returns
    -------
    dict mapping leg name → {'theta_hip': array, 'theta_knee': array, 'phase': array}
    """
    phases = np.linspace(0, 1, n_steps, endpoint=False)
    result = {}

    for leg in LEG_NAMES:
        hips   = np.zeros(n_steps)
        knees  = np.zeros(n_steps)
        for i, p in enumerate(phases):
            hips[i], knees[i] = joint_angles_at_phase(leg, p)
        result[leg] = {
            "theta_hip":  hips,
            "theta_knee": knees,
            "phase":      phases,
        }

    return result
