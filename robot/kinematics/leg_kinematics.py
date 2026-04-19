"""
Leg kinematics for a 3-segment quadruped leg (thigh + shank + cannon)
driven by a linear ballscrew knee actuator.

Highland Cow build — stocky low-CoM quadruped with short thick legs.

Physical design (side view)
───────────────────────────

  Motor (brushless) ──▶ spins shaft
        │
  ┌─────▼──────┐  ← top of upper-leg tube (thigh, L1=0.20m)
  │   shaft    │    ballscrew nut travels up/down the shaft
  │    ▲ nut   │    nut position y_nut [m] from knee pivot height
  │    │       │
  │  [slot]────┼──── pivot point (knee joint) on outside of tube
  │            │
  └────────────┘  ← bottom of upper-leg tube
        │
   ┌────┴────┐   ← shank (L2=0.18m)
   │         │
   └────┬────┘   ← ankle / cannon joint
        │
   ┌────┴────┐   ← cannon bone (L3=0.10m, pantograph-passive)
   │         │
   └────┬────┘
       (●)       ← hoof sphere (radius 0.04m)

  The cannon bone is driven passively by a pantograph linkage:
    cannon_angle = CANNON_LEAN - (theta_thigh + theta_knee)
  This keeps the cannon near-vertical regardless of leg pose.

Virtual-work relationship (exact, no small-angle approximation)
───────────────────────────────────────────────────────────────
  y_nut(θ)   = R_ARM · (1 − cos θ_knee)   [nut displacement from θ=0]
  dy/dθ      = R_ARM · sin θ_knee
  Lever arm  = |dy/dθ| = R_ARM · |sin θ_knee|

  ∴  F_nut  = τ_knee / (R_ARM · |sin θ_knee|)

  Singular at θ_knee = 0 (leg fully straight — never occurs in normal gait).

Coordinate convention (URDF)
─────────────────────────────
  θ_knee < 0   : front leg knee bent (elbow-DOWN / carpal forward)
  θ_knee > 0   : rear leg knee bent  (elbow-UP / hock backward)
  θ_hip  > 0   : hip swings leg forward
"""

import math
import numpy as np


# ── Leg geometry (Highland Cow) ────────────────────────────────────
L1 = 0.20       # thigh length  [m]
L2 = 0.18       # shank length  [m]
L3 = 0.10       # cannon bone length  [m]
FOOT_R = 0.04   # hoof sphere radius  [m]
CANNON_LEAN = 0.08  # cannon forward lean angle [rad]

# ── Ballscrew joint geometry ──────────────────────────────────────
R_ARM = 0.04            # distance from knee pivot to nut-connection point
                        # on the shank (lever arm at 90° bend)          [m]
                        # scaled for shorter highland cow thigh
TUBE_RADIUS = 0.018     # outer radius of the ballscrew tube             [m]
PIVOT_FROM_BOTTOM = 0.03  # how far above the bottom of the thigh tube
                          # the knee pivot sits                          [m]

# Useful derived quantities
NUT_RANGE_MIN_DEG = -10.0   # knee angle at nearly-straight (start of range) [°]
NUT_RANGE_MAX_DEG = -100.0  # knee angle at maximum bend                     [°]
NUT_STROKE = R_ARM * (
    np.cos(np.radians(NUT_RANGE_MIN_DEG)) - np.cos(np.radians(NUT_RANGE_MAX_DEG))
)  # total nut travel over working range [m]

# ── Stance geometry ────────────────────────────────────────────────
# Deep crouch neutral: thigh=0.55 rad, knee=-1.10 rad
# FK hip height: L1*cos(0.55)+L2*cos(-0.55)+L3*cos(0.08)+FOOT_R = 0.464m
# Ankle height (IK target): 0.464 - L3*cos(CANNON_LEAN) - FOOT_R = 0.324m
BODY_HEIGHT = 0.464       # nominal hip-to-ground distance  [m]
ANKLE_HEIGHT = BODY_HEIGHT - L3 * math.cos(CANNON_LEAN) - FOOT_R  # ~0.324m


def forward_kinematics(theta_hip: float, theta_knee: float,
                        include_cannon: bool = True) -> dict:
    """
    Compute joint positions in the hip frame given joint angles.

    Uses the 2-link (thigh+shank) IK target at the ankle, then optionally
    appends the passively-driven cannon bone to get the hoof position.

    Parameters
    ----------
    theta_hip      : float  Hip flexion/extension angle [rad].  Positive = forward.
    theta_knee     : float  Knee flexion angle [rad].  Negative = bent front, positive = bent rear.
    include_cannon : bool   If True, compute cannon + hoof positions too.

    Returns
    -------
    dict with keys:
      'knee_pos'    : (x, z) knee position  [m]
      'ankle_pos'   : (x, z) ankle / cannon-joint position  [m]
      'foot_pos'    : (x, z) hoof contact point  [m]  (same as ankle if no cannon)
      'leg_length'  : straight-line hip-to-ankle distance  [m]
      'cannon_angle': cannon bone absolute angle [rad]  (if include_cannon)
    """
    # Thigh end (= knee position) in hip frame
    knee_x = L1 * np.sin(theta_hip)
    knee_z = -L1 * np.cos(theta_hip)

    # Absolute shank angle
    shank_angle = theta_hip + theta_knee
    ankle_x = knee_x + L2 * np.sin(shank_angle)
    ankle_z = knee_z - L2 * np.cos(shank_angle)

    leg_length = np.sqrt(ankle_x**2 + ankle_z**2)

    result = {
        "knee_pos": (knee_x, knee_z),
        "ankle_pos": (ankle_x, ankle_z),
        "leg_length": leg_length,
    }

    if include_cannon:
        cannon_angle = CANNON_LEAN - (theta_hip + theta_knee)
        foot_x = ankle_x + L3 * np.sin(cannon_angle)
        foot_z = ankle_z - L3 * np.cos(cannon_angle)
        result["foot_pos"] = (foot_x, foot_z)
        result["cannon_angle"] = cannon_angle
    else:
        result["foot_pos"] = (ankle_x, ankle_z)

    return result


def inverse_kinematics(foot_x: float, foot_z: float,
                        elbow_up: bool = False) -> tuple[float, float]:
    """
    Compute thigh and knee angles from a desired ankle position in the hip frame.

    The IK solves for the 2-link (thigh+shank) chain targeting the ankle.
    The cannon bone angle is then computed separately via the pantograph rule.

    Parameters
    ----------
    foot_x   : float  Horizontal ankle position (forward positive)  [m]
    foot_z   : float  Vertical ankle position (downward negative)    [m]
    elbow_up : bool   If True, rear-leg solution (positive knee). Default = front leg.

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

    if elbow_up:
        theta_knee = np.arccos(cos_knee)    # positive = rear leg hock
    else:
        theta_knee = -np.arccos(cos_knee)   # negative = front leg carpal

    # Hip angle
    alpha = np.arctan2(foot_x, -foot_z)
    beta = np.arcsin(np.clip(L2 * np.sin(abs(theta_knee)) / r, -1.0, 1.0))
    if elbow_up:
        theta_hip = alpha - beta
    else:
        theta_hip = alpha + beta

    return theta_hip, theta_knee


def cannon_angle(theta_hip: float, theta_knee: float) -> float:
    """Pantograph-driven cannon bone angle: keeps cannon near-vertical."""
    return CANNON_LEAN - (theta_hip + theta_knee)


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
