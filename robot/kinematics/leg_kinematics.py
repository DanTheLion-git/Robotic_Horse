"""
Leg kinematics for a 2-link leg (thigh + shank) with a linear
ballscrew driving the knee hinge.

Geometry (side view, leg hanging down):
                 hip
                  O  ← hip revolute joint
                  |
           thigh  | L1 = 0.45 m
                  |
                  O  ← knee revolute joint  θ_knee
                  |
           shank  | L2 = 0.50 m
                  |
                  ●  ← foot

The ballscrew actuator connects a pivot on the thigh (distance a
from the knee) to a pivot on the shank (distance b from the knee).
Actuator length s(θ) is computed by the law of cosines.

                 A ── (thigh, distance a from knee)
                 |  ╲
              s  |    ╲ ballscrew
                 |      ╲
                 B ── (shank, distance b from knee)

  s² = a² + b² − 2·a·b·cos(π − θ_knee)
     = a² + b² + 2·a·b·cos(θ_knee)   (θ_knee < 0 by URDF convention)
"""

import numpy as np


# ── Robot constants ────────────────────────────────────────────────
L1 = 0.45   # thigh length  [m]
L2 = 0.50   # shank length  [m]

# Ballscrew attachment points measured from knee joint centre
BALLSCREW_A = 0.12   # pivot on thigh  [m]
BALLSCREW_B = 0.10   # pivot on shank  [m]


def forward_kinematics(theta_hip: float, theta_knee: float) -> dict:
    """
    Compute foot position in the hip frame given joint angles.

    Parameters
    ----------
    theta_hip  : float  Hip flexion/extension angle [rad].  Positive = forward.
    theta_knee : float  Knee flexion angle [rad].  Negative = bent (URDF convention).

    Returns
    -------
    dict with keys:
      'knee_pos'  : (x, z) knee position  [m]
      'foot_pos'  : (x, z) foot position  [m]
      'leg_length': straight-line hip-to-foot distance  [m]
    """
    # Thigh end (= knee position) in hip frame
    knee_x = L1 * np.sin(theta_hip)
    knee_z = -L1 * np.cos(theta_hip)

    # Absolute shank angle
    shank_angle = theta_hip + theta_knee
    foot_x = knee_x + L2 * np.sin(shank_angle)
    foot_z = knee_z - L2 * np.cos(shank_angle)

    leg_length = np.sqrt(foot_x**2 + foot_z**2)

    return {
        "knee_pos": (knee_x, knee_z),
        "foot_pos": (foot_x, foot_z),
        "leg_length": leg_length,
    }


def inverse_kinematics(foot_x: float, foot_z: float) -> tuple[float, float]:
    """
    Compute joint angles from a desired foot position in the hip frame.

    Parameters
    ----------
    foot_x : float  Horizontal foot position (forward positive)  [m]
    foot_z : float  Vertical foot position (downward negative)   [m]

    Returns
    -------
    (theta_hip, theta_knee) : joint angles in radians.

    Raises ValueError if the target is out of reach.
    """
    r = np.sqrt(foot_x**2 + foot_z**2)
    if r > L1 + L2:
        raise ValueError(
            f"Target ({foot_x:.3f}, {foot_z:.3f}) is out of reach "
            f"(distance {r:.3f} > {L1 + L2:.3f} m)."
        )
    if r < abs(L1 - L2):
        raise ValueError(
            f"Target ({foot_x:.3f}, {foot_z:.3f}) is too close "
            f"(distance {r:.3f} < {abs(L1 - L2):.3f} m)."
        )

    # Knee angle via law of cosines
    cos_knee = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_knee = np.clip(cos_knee, -1.0, 1.0)
    theta_knee = -np.arccos(cos_knee)   # negative = bent knee

    # Hip angle
    alpha = np.arctan2(foot_x, -foot_z)          # angle to target from vertical
    beta  = np.arcsin(np.clip(L2 * np.sin(-theta_knee) / r, -1.0, 1.0))
    theta_hip = alpha - beta

    return theta_hip, theta_knee


def ballscrew_length(theta_knee: float) -> float:
    """
    Return the required ballscrew actuator length for a given knee angle.

    Parameters
    ----------
    theta_knee : float  Knee angle [rad], negative = bent.

    Returns
    -------
    length [m]
    """
    a, b = BALLSCREW_A, BALLSCREW_B
    # Supplement: interior angle at the knee in the actuator triangle
    phi = np.pi + theta_knee   # theta_knee is negative → phi < π
    length = np.sqrt(a**2 + b**2 - 2 * a * b * np.cos(phi))
    return length


def ballscrew_velocity(theta_knee: float, omega_knee: float) -> float:
    """
    Return ballscrew linear velocity from knee angular velocity.

    ds/dt = (ds/dθ) · ω

    Parameters
    ----------
    theta_knee : float  Knee angle [rad]
    omega_knee : float  Knee angular velocity [rad/s]

    Returns
    -------
    linear velocity [m/s]
    """
    a, b = BALLSCREW_A, BALLSCREW_B
    phi = np.pi + theta_knee
    s = ballscrew_length(theta_knee)
    ds_dtheta = (a * b * np.sin(phi)) / s
    return ds_dtheta * omega_knee
