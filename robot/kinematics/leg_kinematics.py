"""
Leg kinematics for a 2-link leg (thigh + shank) driven by a
linear ballscrew knee actuator.

Physical design (side view)
───────────────────────────

  Motor (brushless) ──▶ spins shaft
        │
  ┌─────▼──────┐  ← top of upper-leg tube
  │   shaft    │    ballscrew nut travels up/down the shaft
  │    ▲ nut   │    nut position y_nut [m] from knee pivot height
  │    │       │
  │  [slot]────┼──── pivot point (knee joint) on outside of tube
  │            │
  └────────────┘  ← bottom of upper-leg tube

  The nut is connected, through the slot, to the lower leg (shank)
  at a distance R_ARM from the knee pivot.  As the nut moves up/down
  the shaft, it swings the shank forward/backward around the pivot.

Virtual-work relationship (exact, no small-angle approximation)
───────────────────────────────────────────────────────────────
  y_nut(θ)   = R_ARM · (1 − cos θ_knee)   [nut displacement from θ=0]
  dy/dθ      = R_ARM · sin θ_knee
  Lever arm  = |dy/dθ| = R_ARM · |sin θ_knee|

  ∴  F_nut  = τ_knee / (R_ARM · |sin θ_knee|)

  Singular at θ_knee = 0 (leg fully straight — never occurs in normal gait).

Coordinate convention (URDF)
─────────────────────────────
  θ_knee = 0   : shank aligned with thigh (straight leg)
  θ_knee < 0   : knee bent (shank swings backward / downward)
  θ_hip  > 0   : hip swings leg forward
"""

import numpy as np


# ── Leg geometry ───────────────────────────────────────────────────
L1 = 0.45       # upper leg (thigh) length — top of tube to knee pivot  [m]
L2 = 0.50       # lower leg (shank) length — knee pivot to foot         [m]

# ── Ballscrew joint geometry ───────────────────────────────────────
R_ARM = 0.08            # distance from knee pivot to nut-connection point
                        # on the shank (lever arm at 90° bend)          [m]
TUBE_RADIUS = 0.03      # outer radius of the upper-leg tube             [m]
PIVOT_FROM_BOTTOM = 0.05  # how far above the bottom of the upper-leg tube
                          # the knee pivot sits                          [m]

# Useful derived quantities
NUT_RANGE_MIN_DEG = -10.0   # knee angle at nearly-straight (start of range) [°]
NUT_RANGE_MAX_DEG = -100.0  # knee angle at maximum bend                     [°]
NUT_STROKE = R_ARM * (
    np.cos(np.radians(NUT_RANGE_MIN_DEG)) - np.cos(np.radians(NUT_RANGE_MAX_DEG))
)  # total nut travel over working range [m]


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


def nut_position(theta_knee: float) -> float:
    """
    Vertical position of the ballscrew nut relative to its position
    at θ_knee = 0 (straight leg).

    y_nut(θ) = R_ARM · (1 − cos θ_knee)

    Returns
    -------
    y_nut [m]  — positive means nut has moved UP from straight-leg position
    """
    return R_ARM * (1.0 - np.cos(theta_knee))


def nut_lever_arm(theta_knee: float) -> float:
    """
    Effective lever arm of the nut force about the knee pivot.

    lever(θ) = |dy_nut/dθ| = R_ARM · |sin θ_knee|

    This is the perpendicular distance from the pivot to the line of
    action of the nut force (vertical), and it determines how much
    torque a given nut force produces (and vice-versa).

    Returns
    -------
    lever [m]  — 0 at fully straight leg (singular), maximum at 90° bend
    """
    return R_ARM * abs(np.sin(theta_knee))


def nut_velocity(theta_knee: float, omega_knee: float) -> float:
    """
    Linear velocity of the ballscrew nut from knee angular velocity.

    v_nut = (dy_nut/dθ) · ω = R_ARM · sin(θ_knee) · ω

    Parameters
    ----------
    theta_knee : float  Knee angle [rad]
    omega_knee : float  Knee angular velocity [rad/s]

    Returns
    -------
    v_nut [m/s]  — positive = nut moving up
    """
    return R_ARM * np.sin(theta_knee) * omega_knee


def nut_stroke_for_range(theta_min: float, theta_max: float) -> float:
    """
    Required nut travel for a given knee angle range.

    Parameters
    ----------
    theta_min : float  Start angle [rad]
    theta_max : float  End angle   [rad]

    Returns
    -------
    stroke [m]
    """
    return abs(nut_position(theta_max) - nut_position(theta_min))
