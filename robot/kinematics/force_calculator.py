"""
QDD (Quasi-Direct Drive) force calculator for the Highland Cow quadruped.

Replaces the old ballscrew force model with direct-drive torque analysis.
QDD motors output torque directly at the joint through a low-ratio planetary
gearbox, providing backdrivability, compliance, and torque sensing.

QDD motor relationship:
  τ_joint = τ_motor × N × η       (torque)
  ω_motor = ω_joint × N            (speed)
  P       = τ_joint × ω_joint      (power)

Where N = gear ratio, η = efficiency (~0.95 for planetary).

Weight distribution: 55% front / 45% rear (bovine standard).
"""

import numpy as np
from robot.kinematics.leg_kinematics import (
    L1_FRONT, L2_FRONT, L3_FRONT,
    L1_REAR, L2_REAR, L3_REAR,
    get_leg_dims,
    forward_kinematics,
    cannon_angle,
    FRONT_WEIGHT_FRACTION,
    REAR_WEIGHT_FRACTION,
)


# ── Robot mass parameters (150 cm Highland Cow build) ─────────────
G = 9.81            # gravitational acceleration [m/s²]
M_BODY   = 107.0    # total robot mass [kg] (larger frame, same motors)
NUM_LEGS = 4

# Per-leg masses differ for front (shorter) vs rear (longer) legs
M_HIP    = 2.0      # hip bracket mass [kg] (same both)

M_THIGH_FRONT  = 4.0   # humerus + QDD motor [kg]
M_SHANK_FRONT  = 3.5   # radius + QDD motor [kg]
M_CANNON_FRONT = 1.0   # metacarpal [kg]
M_FOOT_FRONT   = 0.4   # front hoof [kg]
M_LEG_FRONT = M_HIP + M_THIGH_FRONT + M_SHANK_FRONT + M_CANNON_FRONT + M_FOOT_FRONT  # 10.9 kg

M_THIGH_REAR   = 5.0   # femur + QDD motor [kg] (longer, heavier)
M_SHANK_REAR   = 4.0   # tibia + QDD motor [kg]
M_CANNON_REAR  = 1.2   # metatarsal [kg]
M_FOOT_REAR    = 0.5   # rear hoof (larger) [kg]
M_LEG_REAR  = M_HIP + M_THIGH_REAR + M_SHANK_REAR + M_CANNON_REAR + M_FOOT_REAR  # 12.7 kg

# Backward-compatible aliases (average)
M_THIGH  = M_THIGH_FRONT
M_SHANK  = M_SHANK_FRONT
M_CANNON = M_CANNON_FRONT
M_FOOT   = M_FOOT_FRONT
M_LEG    = M_LEG_FRONT


def get_leg_masses(is_front: bool):
    """Return (m_thigh, m_shank, m_cannon, m_foot, m_leg_total) for front/rear."""
    if is_front:
        return M_THIGH_FRONT, M_SHANK_FRONT, M_CANNON_FRONT, M_FOOT_FRONT, M_LEG_FRONT
    return M_THIGH_REAR, M_SHANK_REAR, M_CANNON_REAR, M_FOOT_REAR, M_LEG_REAR

# ── QDD Motor specifications ──────────────────────────────────────
QDD_SPECS = {
    'hip_yaw': {
        'name': 'RobStride 06',
        'peak_torque': 36.0,      # N·m at joint
        'cont_torque': 12.0,      # N·m continuous
        'no_load_speed': 33.5,    # rad/s at joint (320 RPM motor / 9:1)
        'gear_ratio': 9.0,
        'efficiency': 0.95,
        'mass': 0.621,            # kg
        'price_eur': 300,
    },
    'hip_pitch': {
        'name': 'RobStride 04',
        'peak_torque': 120.0,
        'cont_torque': 40.0,
        'no_load_speed': 20.9,    # rad/s (200 RPM / 9:1)
        'gear_ratio': 9.0,
        'efficiency': 0.95,
        'mass': 1.420,
        'price_eur': 500,
    },
    'knee': {
        'name': 'RobStride 04',
        'peak_torque': 120.0,
        'cont_torque': 40.0,
        'no_load_speed': 20.9,
        'gear_ratio': 9.0,
        'efficiency': 0.95,
        'mass': 1.420,
        'price_eur': 500,
    },
    'cannon': {
        'name': 'RobStride 02',
        'peak_torque': 17.0,
        'cont_torque': 6.0,
        'no_load_speed': 52.4,    # rad/s (500 RPM / 6:1)
        'gear_ratio': 6.0,
        'efficiency': 0.95,
        'mass': 0.405,
        'price_eur': 200,
    },
}


def weight_per_leg(is_front: bool) -> float:
    """Vertical load on one leg based on bovine 55/45 weight distribution [N]."""
    frac = FRONT_WEIGHT_FRACTION if is_front else REAR_WEIGHT_FRACTION
    m_leg = M_LEG_FRONT if is_front else M_LEG_REAR
    return (M_BODY * frac / 2.0 + m_leg) * G


def joint_torques_static(
    theta_hip: float,
    theta_knee: float,
    is_front: bool = True,
) -> dict:
    """
    Static joint torques for all joints in one leg.

    Computes the torque each QDD motor must produce to hold the leg
    in static equilibrium at the given joint angles.

    Parameters
    ----------
    theta_hip   : Hip pitch angle [rad]
    theta_knee  : Knee angle [rad]
    is_front    : True for front legs (55% weight, elbow-DOWN)

    Returns
    -------
    dict with 'hip_pitch', 'knee', 'cannon' torques [N·m]
    """
    is_rear = not is_front
    l1, l2, l3, _ = get_leg_dims(is_rear)
    m_thigh, m_shank, m_cannon, m_foot, _ = get_leg_masses(is_front)

    fk = forward_kinematics(theta_hip, theta_knee,
                            include_cannon=True, is_rear=is_rear)
    foot_x, foot_z = fk["foot_pos"]
    knee_x, knee_z = fk["knee_pos"]
    ankle_x, ankle_z = fk["ankle_pos"]

    # Vertical ground reaction force at this leg
    grf = weight_per_leg(is_front)

    # ── Hip pitch torque ──
    tau_hip = grf * abs(foot_x)

    # ── Knee torque ──
    tau_knee = grf * abs(foot_x - knee_x)

    # Add gravity torques of shank + cannon segments about knee
    shank_angle = theta_hip + theta_knee
    tau_knee += m_shank * G * (l2 / 2) * abs(np.sin(shank_angle))
    tau_knee += m_cannon * G * l2 * abs(np.sin(shank_angle))
    tau_knee += m_foot * G * l2 * abs(np.sin(shank_angle))

    # ── Cannon torque ──
    ca = fk["cannon_angle"]
    tau_cannon = (m_cannon / 2 + m_foot) * G * l3 * abs(np.sin(ca))

    return {
        'hip_pitch': tau_hip,
        'knee': tau_knee,
        'cannon': tau_cannon,
    }


def check_motor_limits(joint_torques: dict, joint_speeds: dict = None) -> dict:
    """
    Check if required torques/speeds are within QDD motor limits.

    Returns dict mapping joint name → {required, peak, continuous,
    within_peak, within_continuous, utilization}.
    """
    results = {}
    mapping = {'hip_pitch': 'hip_pitch', 'knee': 'knee', 'cannon': 'cannon'}

    for joint, motor_key in mapping.items():
        spec = QDD_SPECS[motor_key]
        required = abs(joint_torques.get(joint, 0.0))
        results[joint] = {
            'required_torque': required,
            'peak_torque': spec['peak_torque'],
            'cont_torque': spec['cont_torque'],
            'within_peak': required <= spec['peak_torque'],
            'within_continuous': required <= spec['cont_torque'],
            'utilization_peak': required / spec['peak_torque'],
            'utilization_cont': required / spec['cont_torque'],
            'motor': spec['name'],
        }

    return results


def analyse_gait_forces(
    theta_hip_seq: np.ndarray,
    theta_knee_seq: np.ndarray,
    omega_hip_seq: np.ndarray = None,
    omega_knee_seq: np.ndarray = None,
    is_front: bool = True,
) -> dict:
    """
    Compute joint torques and motor requirements over a gait sequence.

    Parameters
    ----------
    theta_hip_seq   : (N,) hip angles [rad]
    theta_knee_seq  : (N,) knee angles [rad]
    omega_hip_seq   : (N,) hip angular velocities [rad/s] (optional)
    omega_knee_seq  : (N,) knee angular velocities [rad/s] (optional)
    is_front        : True for front legs (55% weight)

    Returns
    -------
    dict with arrays: tau_hip, tau_knee, tau_cannon, motor_torque_hip,
    motor_torque_knee, power_hip, power_knee, foot_x, foot_z
    """
    n = len(theta_hip_seq)
    if omega_hip_seq is None:
        omega_hip_seq = np.zeros(n)
    if omega_knee_seq is None:
        omega_knee_seq = np.zeros(n)

    tau_hip    = np.zeros(n)
    tau_knee   = np.zeros(n)
    tau_cannon = np.zeros(n)
    pwr_hip    = np.zeros(n)
    pwr_knee   = np.zeros(n)
    foot_x     = np.zeros(n)
    foot_z     = np.zeros(n)

    is_rear = not is_front
    for i in range(n):
        th, tk = theta_hip_seq[i], theta_knee_seq[i]
        oh, ok = omega_hip_seq[i], omega_knee_seq[i]

        torques = joint_torques_static(th, tk, is_front=is_front)
        tau_hip[i]    = torques['hip_pitch']
        tau_knee[i]   = torques['knee']
        tau_cannon[i] = torques['cannon']

        pwr_hip[i]  = abs(torques['hip_pitch'] * oh)
        pwr_knee[i] = abs(torques['knee'] * ok)

        fk = forward_kinematics(th, tk, is_rear=is_rear)
        foot_x[i], foot_z[i] = fk["foot_pos"]

    # Motor-side torques
    hip_spec = QDD_SPECS['hip_pitch']
    knee_spec = QDD_SPECS['knee']

    return {
        "tau_hip":          tau_hip,
        "tau_knee":         tau_knee,
        "tau_cannon":       tau_cannon,
        "motor_torque_hip": tau_hip / (hip_spec['gear_ratio'] * hip_spec['efficiency']),
        "motor_torque_knee": tau_knee / (knee_spec['gear_ratio'] * knee_spec['efficiency']),
        "power_hip":        pwr_hip,
        "power_knee":       pwr_knee,
        "foot_x":           foot_x,
        "foot_z":           foot_z,
    }
