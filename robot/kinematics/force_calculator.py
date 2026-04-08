"""
Force calculator for the linear ballscrew knee actuator.

The ballscrew converts a linear force F_screw [N] into a knee torque τ [N·m]
through a slider-crank geometry.  We also compute the required motor torque
from the ballscrew lead (mm/rev).

Key relationships
─────────────────
1. Torque ↔ force via virtual work principle:
       τ = F_screw · (ds/dθ)
   where s(θ) is the ballscrew length as a function of knee angle.
   Therefore:
       F_screw = τ / (ds/dθ)

2. Required knee torque from static equilibrium of the shank under
   gravity and a vertical ground reaction force (GRF):
       τ_knee = m_shank · g · (L2/2) · sin(−(θ_hip + θ_knee))
              + F_GRF · moment_arm(θ_hip, θ_knee)

3. Motor torque from ballscrew lead:
       τ_motor = F_screw · (lead / (2π)) / η
   where lead [m/rev], η = mechanical efficiency (≈ 0.90 for ballscrew).

All angles follow the URDF convention (knee negative when bent).
"""

import numpy as np
from robot.kinematics.leg_kinematics import (
    L1, L2, BALLSCREW_A, BALLSCREW_B,
    ballscrew_length, forward_kinematics,
)


# ── Constants ──────────────────────────────────────────────────────
G = 9.81            # m/s²
M_BODY = 20.0       # body frame mass  [kg]  (legs carry ~body/4 each)
M_THIGH = 1.5       # thigh link mass  [kg]
M_SHANK = 1.0       # shank + foot mass [kg]
M_FOOT = 0.2        # foot sphere mass  [kg]

BALLSCREW_LEAD = 0.005   # 5 mm / rev → 0.005 m/rev
BALLSCREW_EFF  = 0.90    # mechanical efficiency

NUM_LEGS = 4


def ds_dtheta(theta_knee: float) -> float:
    """
    Derivative of ballscrew length w.r.t. knee angle (used as lever arm).

    ds/dθ = a·b·sin(π + θ_knee) / s(θ_knee)
    """
    a, b = BALLSCREW_A, BALLSCREW_B
    phi = np.pi + theta_knee
    s = ballscrew_length(theta_knee)
    return (a * b * np.sin(phi)) / s


def knee_torque_static(
    theta_hip: float,
    theta_knee: float,
    body_mass: float = M_BODY,
    include_grf: bool = True,
) -> float:
    """
    Estimate knee joint torque required for static equilibrium.

    The knee must resist:
      - Weight of the shank (mass at L2/2)
      - Weight of the foot (mass at L2)
      - A share of body weight acting vertically through the foot
        (body_mass / NUM_LEGS, balanced via GRF)

    Parameters
    ----------
    theta_hip    : Hip flexion angle [rad]
    theta_knee   : Knee angle [rad] (negative = bent)
    body_mass    : Body frame mass [kg]
    include_grf  : Whether to include the body-weight ground reaction

    Returns
    -------
    τ [N·m]  — positive = knee extension torque required
    """
    shank_angle = theta_hip + theta_knee  # absolute angle from vertical

    # Gravity torque from shank mass (COM at L2/2 from knee)
    tau_shank = (M_SHANK * G * (L2 / 2) * np.abs(np.sin(shank_angle)))

    # Gravity torque from foot mass (at L2 from knee)
    tau_foot = (M_FOOT * G * L2 * np.abs(np.sin(shank_angle)))

    tau_grf = 0.0
    if include_grf:
        # GRF = body share + leg masses (simplified: vertical force, horizontal moment arm)
        fk = forward_kinematics(theta_hip, theta_knee)
        foot_x, foot_z = fk["foot_pos"]
        knee_x, knee_z = fk["knee_pos"]

        # Moment arm from knee to foot (horizontal component)
        moment_arm = abs(foot_x - knee_x)
        vertical_load = (body_mass / NUM_LEGS + M_THIGH + M_SHANK + M_FOOT) * G
        tau_grf = vertical_load * moment_arm

    return tau_shank + tau_foot + tau_grf


def ballscrew_force(theta_hip: float, theta_knee: float, **kwargs) -> float:
    """
    Linear force the ballscrew must exert at the given configuration.

    F = τ_knee / (ds/dθ)

    Returns
    -------
    F [N]
    """
    tau = knee_torque_static(theta_hip, theta_knee, **kwargs)
    lever = ds_dtheta(theta_knee)
    if abs(lever) < 1e-6:
        return float("inf")   # singular configuration
    return tau / abs(lever)


def motor_torque_from_force(force_n: float) -> float:
    """
    Convert ballscrew linear force to required motor shaft torque.

    τ_motor = F · (lead / 2π) / η

    Returns
    -------
    τ_motor [N·m]
    """
    return force_n * (BALLSCREW_LEAD / (2 * np.pi)) / BALLSCREW_EFF


def analyse_gait_forces(
    theta_hip_seq: np.ndarray,
    theta_knee_seq: np.ndarray,
    body_mass: float = M_BODY,
) -> dict:
    """
    Compute ballscrew force and motor torque over a gait sequence.

    Parameters
    ----------
    theta_hip_seq  : (N,) array of hip angles  [rad]
    theta_knee_seq : (N,) array of knee angles [rad]
    body_mass      : Body mass [kg]

    Returns
    -------
    dict with arrays: 'tau_knee', 'ballscrew_force', 'motor_torque',
                      'ballscrew_length', 'foot_x', 'foot_z'
    """
    n = len(theta_hip_seq)
    tau_k  = np.zeros(n)
    f_bs   = np.zeros(n)
    tau_m  = np.zeros(n)
    s_bs   = np.zeros(n)
    foot_x = np.zeros(n)
    foot_z = np.zeros(n)

    for i, (th, tk) in enumerate(zip(theta_hip_seq, theta_knee_seq)):
        tau_k[i] = knee_torque_static(th, tk, body_mass=body_mass)
        f_bs[i]  = ballscrew_force(th, tk, body_mass=body_mass)
        tau_m[i] = motor_torque_from_force(f_bs[i])
        s_bs[i]  = ballscrew_length(tk)
        fk = forward_kinematics(th, tk)
        foot_x[i], foot_z[i] = fk["foot_pos"]

    return {
        "tau_knee":        tau_k,
        "ballscrew_force": f_bs,
        "motor_torque":    tau_m,
        "ballscrew_length": s_bs,
        "foot_x":          foot_x,
        "foot_z":          foot_z,
    }
