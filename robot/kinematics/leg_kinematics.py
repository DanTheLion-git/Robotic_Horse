"""
Leg kinematics for a 3-segment quadruped leg modeled on Highland Cow anatomy.

Highland Cow Build — stocky low-CoM quadruped with bovine leg proportions.

Anatomical mapping
──────────────────
  FRONT LEG (Thoracic limb):
    hip_joint   → Shoulder (scapulohumeral) — yaw for lateral placement
    thigh_joint → Humerus (upper arm)       — pitch flex/ext
    knee_joint  → Elbow/Carpus combined     — pitch flex/ext (elbow-DOWN)
    cannon_joint→ Metacarpal (cannon bone)  — PASSIVE linkage keeps vertical

  REAR LEG (Pelvic limb):
    hip_joint   → Hip (coxofemoral)         — yaw for lateral placement
    thigh_joint → Femur (thigh)             — pitch flex/ext
    knee_joint  → Stifle (true knee)        — pitch flex/ext (elbow-UP)
    cannon_joint→ Metatarsal via HOCK       — RECIPROCAL APPARATUS couples to stifle

Reciprocal Apparatus (rear legs only)
─────────────────────────────────────
  In real cattle, two tendons (peroneus tertius + superficial digital flexor)
  mechanically couple the stifle and hock — forming a four-bar linkage:
    When stifle bends → hock MUST bend proportionally (and vice versa)

  Robot model:
    cannon_rear = RECIP_OFFSET - RECIP_RATIO × knee_angle
    RECIP_RATIO ≈ 0.85 (slightly less than 1:1 real coupling)
    RECIP_OFFSET chosen so cannon ≈ -0.47 rad at neutral stance

Front leg cannon (passive geometry — keeps metacarpal near-vertical):
    cannon_front = CANNON_LEAN - (thigh_angle + knee_angle)

QDD Actuation
─────────────
  All joints use quasi-direct-drive (QDD) motors — low-ratio planetary gearboxes
  that are backdrivable, compliant, and provide torque sensing via motor current.

Coordinate convention (URDF)
────────────────────────────
  θ_knee < 0   : front leg knee bent (elbow-DOWN / carpal forward)
  θ_knee > 0   : rear leg knee bent  (elbow-UP / hock backward)
  θ_hip  > 0   : hip swings leg forward
"""

import math
import numpy as np


# ── Leg geometry (Highland Cow) ────────────────────────────────────
L1 = 0.20       # thigh (humerus/femur) length  [m]
L2 = 0.18       # shank (radius/tibia) length   [m]
L3 = 0.10       # cannon bone (metacarpal/metatarsal) length  [m]
FOOT_R = 0.04   # hoof sphere radius  [m]

# ── Cannon bone passive linkage ────────────────────────────────────
CANNON_LEAN = 0.08  # front leg: cannon forward lean at neutral [rad]

# Reciprocal apparatus coupling (rear legs only)
# At neutral stance: knee_rear = +1.10 rad, cannon_rear = -0.47 rad
# cannon = RECIP_OFFSET - RECIP_RATIO * knee
RECIP_RATIO  = 0.85    # coupling ratio (stifle-to-hock, slightly sub-unity)
RECIP_OFFSET = -0.47 + RECIP_RATIO * 1.10  # ≈ 0.465 rad

# ── Bovine joint range limits ──────────────────────────────────────
# Based on bovine biomechanics research, scaled for our robot.
JOINT_LIMITS = {
    'hip_yaw':       (-0.35, 0.35),    # lateral splay ±20°
    'front_thigh':   (-0.30, 1.20),    # shoulder flex/ext (~-17° to +69°)
    'front_knee':    (-1.80, -0.10),   # elbow-DOWN: carpus flexion (negative)
    'front_cannon':  (-0.50, 1.50),    # metacarpal passive range
    'rear_thigh':    (-1.20, 0.30),    # hip flex/ext (symmetric to front)
    'rear_knee':     (0.10, 1.80),     # elbow-UP: stifle flexion (positive)
    'rear_cannon':   (-1.50, 0.80),    # metatarsal reciprocal range
}

# ── Weight distribution ───────────────────────────────────────────
# Cattle carry ~55% of weight on front legs, ~45% on rear
FRONT_WEIGHT_FRACTION = 0.55
REAR_WEIGHT_FRACTION  = 0.45

# ── Stance geometry ───────────────────────────────────────────────
# Deep crouch neutral: thigh=0.55 rad, knee=-1.10 rad (front)
# FK hip height: L1*cos(0.55) + L2*cos(0.55) + L3*cos(CANNON_LEAN) + FOOT_R
#              = 0.1705 + 0.1535 + 0.0997 + 0.04 = 0.464 m
BODY_HEIGHT = 0.464       # nominal hip-to-ground distance [m]
ANKLE_HEIGHT = BODY_HEIGHT - L3 * math.cos(CANNON_LEAN) - FOOT_R  # ≈ 0.324 m


# ── Cannon bone angle functions ────────────────────────────────────

def cannon_angle_front(theta_thigh: float, theta_knee: float) -> float:
    """
    Front leg cannon (metacarpal) angle — passive linkage keeps near-vertical.

    This is a geometric constraint: the cannon bone is connected via a
    parallelogram linkage that approximately cancels the combined rotation
    of the thigh and shank, keeping the metacarpal pointing downward.
    """
    return CANNON_LEAN - (theta_thigh + theta_knee)


def cannon_angle_rear(theta_knee: float) -> float:
    """
    Rear leg cannon (metatarsal) angle — reciprocal apparatus.

    The peroneus tertius and superficial digital flexor tendons create a
    mechanical coupling between the stifle (knee) and hock (cannon):
      - When stifle flexes → hock flexes proportionally
      - Coupling ratio ≈ 0.85:1 (slightly sub-unity)
      - This is equivalent to a four-bar linkage

    Returns cannon joint angle [rad].
    """
    return RECIP_OFFSET - RECIP_RATIO * theta_knee


def cannon_angle(theta_hip: float, theta_knee: float,
                 is_rear: bool = False) -> float:
    """
    Unified cannon angle function for any leg.

    Parameters
    ----------
    theta_hip   : thigh joint angle [rad]
    theta_knee  : knee joint angle [rad]
    is_rear     : True for rear legs (reciprocal apparatus), False for front

    Returns cannon joint angle [rad].
    """
    if is_rear:
        return cannon_angle_rear(theta_knee)
    return cannon_angle_front(theta_hip, theta_knee)


# ── Forward Kinematics ────────────────────────────────────────────

def forward_kinematics(theta_hip: float, theta_knee: float,
                       include_cannon: bool = True,
                       is_rear: bool = False) -> dict:
    """
    Compute joint positions in the hip frame given joint angles.

    Uses the 2-link (thigh+shank) chain to the ankle, then appends the
    passively-driven cannon bone to get the hoof position.

    Parameters
    ----------
    theta_hip      : Hip flex/ext angle [rad]. Positive = forward.
    theta_knee     : Knee flexion angle [rad]. Negative = front, positive = rear.
    include_cannon : If True, compute cannon + hoof positions.
    is_rear        : Use reciprocal apparatus (rear) vs passive linkage (front).

    Returns
    -------
    dict with keys: 'knee_pos', 'ankle_pos', 'foot_pos', 'leg_length',
                    'cannon_angle' (if include_cannon)
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
        ca = cannon_angle(theta_hip, theta_knee, is_rear=is_rear)
        foot_x = ankle_x + L3 * np.sin(ca)
        foot_z = ankle_z - L3 * np.cos(ca)
        result["foot_pos"] = (foot_x, foot_z)
        result["cannon_angle"] = ca
    else:
        result["foot_pos"] = (ankle_x, ankle_z)

    return result


# ── Inverse Kinematics ────────────────────────────────────────────

def inverse_kinematics(foot_x: float, foot_z: float,
                       elbow_up: bool = False) -> tuple[float, float]:
    """
    Compute thigh and knee angles from a desired ankle position in the hip frame.

    Solves the 2-link (thigh+shank) IK. The cannon bone angle is then
    computed separately via cannon_angle_front() or cannon_angle_rear().

    Parameters
    ----------
    foot_x   : Horizontal ankle position (forward positive) [m]
    foot_z   : Vertical ankle position (downward negative)   [m]
    elbow_up : True = rear leg (positive knee), False = front leg (negative knee)

    Returns
    -------
    (theta_hip, theta_knee) in radians.

    Raises ValueError if target is out of reach.
    """
    r = np.sqrt(foot_x**2 + foot_z**2)
    if r > L1 + L2:
        raise ValueError(
            f"Target ({foot_x:.3f}, {foot_z:.3f}) out of reach "
            f"(dist {r:.3f} > {L1 + L2:.3f} m)."
        )
    if r < abs(L1 - L2):
        raise ValueError(
            f"Target ({foot_x:.3f}, {foot_z:.3f}) too close "
            f"(dist {r:.3f} < {abs(L1 - L2):.3f} m)."
        )

    cos_knee = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_knee = np.clip(cos_knee, -1.0, 1.0)

    if elbow_up:
        theta_knee = np.arccos(cos_knee)     # positive = rear leg hock
    else:
        theta_knee = -np.arccos(cos_knee)    # negative = front leg carpal

    alpha = np.arctan2(foot_x, -foot_z)
    beta = np.arcsin(np.clip(L2 * np.sin(abs(theta_knee)) / r, -1.0, 1.0))
    if elbow_up:
        theta_hip = alpha - beta
    else:
        theta_hip = alpha + beta

    return theta_hip, theta_knee


# ── QDD Motor Utilities ───────────────────────────────────────────

def joint_power(torque: float, omega: float) -> float:
    """Instantaneous mechanical power at a joint [W]."""
    return abs(torque * omega)


def motor_torque_from_joint(joint_torque: float, gear_ratio: float,
                            efficiency: float = 0.95) -> float:
    """Motor-side torque required for a given joint-side torque [N·m]."""
    return joint_torque / (gear_ratio * efficiency)


def motor_speed_from_joint(joint_speed_rad_s: float,
                           gear_ratio: float) -> float:
    """Motor-side speed [rad/s] for a given joint speed."""
    return joint_speed_rad_s * gear_ratio
