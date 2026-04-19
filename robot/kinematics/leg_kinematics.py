"""
Leg kinematics for the Highland Cow quadruped robot (150 cm scale).

Anatomically accurate bovine proportions — front and rear legs have
different segment lengths, matching real cattle skeletal geometry.

Anatomical mapping
──────────────────
  FRONT LEG (Thoracic limb) — shorter, lower shoulder attachment:
    hip_joint   → Shoulder (scapulohumeral) — yaw for lateral placement
    thigh_joint → Humerus (upper arm)       — pitch flex/ext
    knee_joint  → Elbow/Carpus combined     — pitch flex/ext (elbow-DOWN)
    cannon_joint→ Metacarpal (cannon bone)  — PASSIVE linkage keeps vertical

  REAR LEG (Pelvic limb) — longer, higher hip attachment:
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
    RECIP_OFFSET calibrated so cannon ≈ -0.05 rad at neutral rear stance

Front leg cannon (passive geometry — keeps metacarpal near-vertical):
    cannon_front = CANNON_LEAN - (thigh_angle + knee_angle)

Body shape
──────────
  The body is NOT a uniform box — it follows bovine anatomy:
    - Withers (highest point, 1.50 m) at front-top where scapulae meet spine
    - Spine slopes slightly down to rump (~1.40 m)
    - Front shoulder joint is LOW on the body (at ~0.93 m above ground)
    - Rear hip joint is HIGH on the body (at ~1.20 m above ground)
    - Rear legs are longer than front legs (femur > humerus, tibia > radius)

Coordinate convention (URDF)
────────────────────────────
  θ_knee < 0   : front leg knee bent (elbow-DOWN / carpal forward)
  θ_knee > 0   : rear leg knee bent  (elbow-UP / hock backward)
  θ_hip  > 0   : hip swings leg forward
"""

import math
import numpy as np


# ══════════════════════════════════════════════════════════════════
#  LEG SEGMENT LENGTHS  —  Front vs Rear (bovine proportions)
# ══════════════════════════════════════════════════════════════════

# ── Front leg (thoracic limb: humerus + radius + metacarpal) ──────
L1_FRONT = 0.38     # humerus length [m]
L2_FRONT = 0.34     # radius/ulna length [m]
L3_FRONT = 0.18     # metacarpal (cannon bone) length [m]
FOOT_R_FRONT = 0.05 # front hoof sphere radius [m]

# ── Rear leg (pelvic limb: femur + tibia + metatarsal) ────────────
L1_REAR = 0.50      # femur length [m]
L2_REAR = 0.45      # tibia length [m]
L3_REAR = 0.22      # metatarsal (cannon bone) length [m]
FOOT_R_REAR = 0.06  # rear hoof sphere radius [m]

# ── Backward-compatible aliases (default to front) ────────────────
L1 = L1_FRONT
L2 = L2_FRONT
L3 = L3_FRONT
FOOT_R = FOOT_R_FRONT


def get_leg_dims(is_rear: bool = False):
    """Return (L1, L2, L3, FOOT_R) for front or rear leg."""
    if is_rear:
        return L1_REAR, L2_REAR, L3_REAR, FOOT_R_REAR
    return L1_FRONT, L2_FRONT, L3_FRONT, FOOT_R_FRONT


# ══════════════════════════════════════════════════════════════════
#  BODY GEOMETRY & HIP POSITIONS
# ══════════════════════════════════════════════════════════════════

# Body dimensions (bovine-shaped, not a simple box)
BODY_LENGTH = 1.80      # shoulder to rump [m]
BODY_WIDTH  = 0.80      # ribcage width [m]
BODY_MAIN_HEIGHT = 0.55 # main barrel/ribcage height [m]

# Body center height above ground when standing
BODY_CENTER_Z = 1.08    # [m]
WITHERS_HEIGHT = 1.50   # top of withers above ground [m]

# Hip joint positions relative to body center (base_link origin)
# Front: shoulder joint LOW on body (scapula sits against ribcage)
FRONT_HIP_OFFSET = (0.65, 0.40, -0.15)   # (x_fwd, y_lat, z_down)
# Rear: hip joint HIGH on body (near top of pelvis)
REAR_HIP_OFFSET  = (-0.65, 0.36, 0.12)   # (x_back, y_lat, z_up)

# Absolute hip heights above ground
FRONT_HIP_HEIGHT = BODY_CENTER_Z + FRONT_HIP_OFFSET[2]  # 1.08 - 0.15 = 0.93 m
REAR_HIP_HEIGHT  = BODY_CENTER_Z + REAR_HIP_OFFSET[2]   # 1.08 + 0.12 = 1.20 m


# ══════════════════════════════════════════════════════════════════
#  CANNON BONE — Passive linkage & Reciprocal apparatus
# ══════════════════════════════════════════════════════════════════

CANNON_LEAN = 0.08  # front cannon forward lean at neutral [rad]

# Reciprocal apparatus coupling (rear legs only)
RECIP_RATIO = 0.85  # stifle-to-hock coupling ratio (sub-unity)

# Calibrate RECIP_OFFSET so cannon ≈ -0.05 rad at neutral rear stance
# Neutral rear knee ≈ +0.505 rad (from IK at ankle height 0.92m)
_NEUTRAL_KNEE_REAR = 0.505
_NEUTRAL_CANNON_REAR = -0.05  # slight back-lean at stance
RECIP_OFFSET = _NEUTRAL_CANNON_REAR + RECIP_RATIO * _NEUTRAL_KNEE_REAR  # ≈ 0.379


# ══════════════════════════════════════════════════════════════════
#  STANCE GEOMETRY (front and rear have different ankle heights)
# ══════════════════════════════════════════════════════════════════

# Ankle height = hip_height - cannon*cos(lean) - foot_radius
ANKLE_HEIGHT_FRONT = (
    FRONT_HIP_HEIGHT - L3_FRONT * math.cos(CANNON_LEAN) - FOOT_R_FRONT
)  # ≈ 0.70 m

ANKLE_HEIGHT_REAR = (
    REAR_HIP_HEIGHT - L3_REAR * math.cos(CANNON_LEAN) - FOOT_R_REAR
)  # ≈ 0.92 m

# Backward-compatible alias (front leg default)
BODY_HEIGHT = FRONT_HIP_HEIGHT
ANKLE_HEIGHT = ANKLE_HEIGHT_FRONT


def get_ankle_height(is_rear: bool = False) -> float:
    """Return ankle height for front or rear leg."""
    return ANKLE_HEIGHT_REAR if is_rear else ANKLE_HEIGHT_FRONT


# ══════════════════════════════════════════════════════════════════
#  JOINT LIMITS (bovine biomechanics)
# ══════════════════════════════════════════════════════════════════

JOINT_LIMITS = {
    'hip_yaw':       (-0.35, 0.35),
    'front_thigh':   (-0.30, 1.20),
    'front_knee':    (-1.80, -0.10),
    'front_cannon':  (-0.50, 1.50),
    'rear_thigh':    (-1.20, 0.30),
    'rear_knee':     (0.10, 1.80),
    'rear_cannon':   (-1.50, 0.80),
}


# ══════════════════════════════════════════════════════════════════
#  WEIGHT DISTRIBUTION
# ══════════════════════════════════════════════════════════════════

FRONT_WEIGHT_FRACTION = 0.55
REAR_WEIGHT_FRACTION  = 0.45


# ══════════════════════════════════════════════════════════════════
#  CANNON BONE ANGLE FUNCTIONS
# ══════════════════════════════════════════════════════════════════

def cannon_angle_front(theta_thigh: float, theta_knee: float) -> float:
    """Front leg cannon — passive parallelogram keeps metacarpal near-vertical."""
    return CANNON_LEAN - (theta_thigh + theta_knee)


def cannon_angle_rear(theta_knee: float) -> float:
    """Rear leg cannon — reciprocal apparatus couples stifle to hock."""
    return RECIP_OFFSET - RECIP_RATIO * theta_knee


def cannon_angle(theta_hip: float, theta_knee: float,
                 is_rear: bool = False) -> float:
    """Unified cannon angle for any leg."""
    if is_rear:
        return cannon_angle_rear(theta_knee)
    return cannon_angle_front(theta_hip, theta_knee)


# ══════════════════════════════════════════════════════════════════
#  FORWARD KINEMATICS
# ══════════════════════════════════════════════════════════════════

def forward_kinematics(theta_hip: float, theta_knee: float,
                       include_cannon: bool = True,
                       is_rear: bool = False) -> dict:
    """
    Compute joint positions in the hip frame given joint angles.

    Uses leg-specific segment lengths (front vs rear).

    Returns dict with 'knee_pos', 'ankle_pos', 'foot_pos', 'leg_length',
    'cannon_angle' (if include_cannon).
    """
    l1, l2, l3, _ = get_leg_dims(is_rear)

    knee_x = l1 * np.sin(theta_hip)
    knee_z = -l1 * np.cos(theta_hip)

    shank_angle = theta_hip + theta_knee
    ankle_x = knee_x + l2 * np.sin(shank_angle)
    ankle_z = knee_z - l2 * np.cos(shank_angle)

    leg_length = np.sqrt(ankle_x**2 + ankle_z**2)

    result = {
        "knee_pos": (knee_x, knee_z),
        "ankle_pos": (ankle_x, ankle_z),
        "leg_length": leg_length,
    }

    if include_cannon:
        ca = cannon_angle(theta_hip, theta_knee, is_rear=is_rear)
        foot_x = ankle_x + l3 * np.sin(ca)
        foot_z = ankle_z - l3 * np.cos(ca)
        result["foot_pos"] = (foot_x, foot_z)
        result["cannon_angle"] = ca
    else:
        result["foot_pos"] = (ankle_x, ankle_z)

    return result


# ══════════════════════════════════════════════════════════════════
#  INVERSE KINEMATICS
# ══════════════════════════════════════════════════════════════════

def inverse_kinematics(foot_x: float, foot_z: float,
                       elbow_up: bool = False,
                       is_rear: bool = None) -> tuple[float, float]:
    """
    Compute thigh and knee angles from a desired ankle position.

    Uses leg-specific segment lengths when is_rear is specified.
    If is_rear is None, infers from elbow_up for backward compatibility.

    Returns (theta_hip, theta_knee) in radians.
    """
    if is_rear is None:
        is_rear = elbow_up
    l1, l2, _, _ = get_leg_dims(is_rear)

    r = np.sqrt(foot_x**2 + foot_z**2)
    max_reach = l1 + l2
    min_reach = abs(l1 - l2)

    if r > max_reach:
        raise ValueError(
            f"Target ({foot_x:.3f}, {foot_z:.3f}) out of reach "
            f"(dist {r:.3f} > {max_reach:.3f} m)."
        )
    if r < min_reach:
        raise ValueError(
            f"Target ({foot_x:.3f}, {foot_z:.3f}) too close "
            f"(dist {r:.3f} < {min_reach:.3f} m)."
        )

    cos_knee = (r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_knee = np.clip(cos_knee, -1.0, 1.0)

    if elbow_up:
        theta_knee = np.arccos(cos_knee)
    else:
        theta_knee = -np.arccos(cos_knee)

    alpha = np.arctan2(foot_x, -foot_z)
    beta = np.arcsin(np.clip(l2 * np.sin(abs(theta_knee)) / r, -1.0, 1.0))
    if elbow_up:
        theta_hip = alpha - beta
    else:
        theta_hip = alpha + beta

    return theta_hip, theta_knee


# ══════════════════════════════════════════════════════════════════
#  QDD MOTOR UTILITIES
# ══════════════════════════════════════════════════════════════════

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
