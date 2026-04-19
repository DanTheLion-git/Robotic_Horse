"""
Force calculator for the linear ballscrew knee actuator.

Physical design summary
───────────────────────
  • Brushless motor at the top of the upper-leg tube spins a ballscrew shaft.
  • The ballscrew nut travels up/down the shaft.
  • A pin through a slot in the tube wall connects the nut to the lower leg
    (shank) at distance R_ARM from the knee pivot.
  • Moving the nut up bends the knee (swings shank forward/up);
    moving it down extends the knee (swings shank back/down).

Key force relationships
───────────────────────
1.  Nut force ↔ knee torque  (virtual work, exact):
        F_nut = τ_knee / (R_ARM · |sin θ_knee|)
    Lever arm = R_ARM · |sin θ_knee|  (0 at straight, max at 90° bend)

2.  Knee torque from static equilibrium:
        τ_knee = weight of shank/foot + body-weight GRF × moment arm

3.  Motor shaft torque from nut force:
        τ_motor = F_nut · (lead / 2π) / η
    where  lead = screw lead [m/rev],  η = mechanical efficiency

4.  Motor shaft speed from knee angular velocity:
        ω_motor = v_nut · (2π / lead)
        v_nut   = R_ARM · |sin θ_knee| · ω_knee

5.  Carriage drag (set to 0 for now — negligible):
        F_carriage = 0  N

All angles follow the URDF convention: θ_knee < 0 when bent.
"""

import numpy as np
from robot.kinematics.leg_kinematics import (
    L1, L2, R_ARM, NUT_STROKE,
    nut_position, nut_lever_arm, nut_velocity,
    forward_kinematics,
)


# ── Robot mass parameters (Highland Cow build) ────────────────────
G = 9.81            # gravitational acceleration  [m/s²]
M_BODY   = 100.0    # body frame mass (motors 48 + frame 24 + electronics 8
                     #   + decoration 35 + legs 5)  [kg]
M_HIP    = 4.0      # hip bracket mass per leg  [kg]
M_THIGH  = 6.0      # upper-leg tube + motor mass per leg  [kg]
M_SHANK  = 5.0      # lower-leg tube mass per leg  [kg]
M_CANNON = 1.2      # cannon bone per leg  [kg]
M_FOOT   = 0.4      # hoof sphere mass per leg  [kg]
NUM_LEGS = 4

# ── Ballscrew / motor parameters ───────────────────────────────────
BALLSCREW_LEAD = 0.005   # screw lead: 5 mm/rev = 0.005 m/rev
BALLSCREW_EFF  = 0.90    # mechanical efficiency of the ballscrew

# ── Carriage parameters ────────────────────────────────────────────
# Resistance is negligible for now; increase once cart inertia is known.
CARRIAGE_DRAG_N = 0.0    # horizontal drag force from carriage  [N]


def knee_torque_static(
    theta_hip: float,
    theta_knee: float,
    body_mass: float = M_BODY,
    include_grf: bool = True,
) -> float:
    """
    Knee joint torque required for static equilibrium.

    Includes:
      • Gravity torque of the shank (COM at L2/2 from pivot)
      • Gravity torque of the foot  (at L2 from pivot)
      • Ground reaction force (GRF) from body weight + upper-leg mass,
        acting vertically at the foot — creates a moment at the knee

    Parameters
    ----------
    theta_hip   : Hip flexion angle [rad]
    theta_knee  : Knee bend angle [rad] (negative = bent)
    body_mass   : Total body frame mass [kg]
    include_grf : Include body-weight GRF contribution

    Returns
    -------
    τ [N·m]  — magnitude of extension torque the actuator must produce
    """
    shank_angle = theta_hip + theta_knee   # absolute angle of shank from vertical

    # Gravity torques about the knee pivot (horizontal moment arm of each mass)
    tau_shank  = M_SHANK  * G * (L2 / 2) * abs(np.sin(shank_angle))
    tau_cannon = M_CANNON * G *  L2       * abs(np.sin(shank_angle))
    tau_foot   = M_FOOT   * G *  L2       * abs(np.sin(shank_angle))

    tau_grf = 0.0
    if include_grf:
        fk = forward_kinematics(theta_hip, theta_knee)
        foot_x,  _  = fk["foot_pos"]
        knee_x,  _  = fk["knee_pos"]

        # Horizontal distance from knee pivot to foot = moment arm for GRF
        moment_arm  = abs(foot_x - knee_x)

        # Vertical load this leg carries: ¼ body + full leg mass stack
        vertical_load = (body_mass / NUM_LEGS + M_HIP + M_THIGH + M_SHANK + M_CANNON + M_FOOT) * G

        tau_grf = vertical_load * moment_arm

    return tau_shank + tau_cannon + tau_foot + tau_grf


def nut_force_required(
    theta_hip: float,
    theta_knee: float,
    body_mass: float = M_BODY,
) -> float:
    """
    Linear force the ballscrew nut must exert to hold/move the knee.

    F_nut = τ_knee / lever_arm = τ_knee / (R_ARM · |sin θ_knee|)

    Returns float('inf') near the singular point (θ_knee ≈ 0).

    Returns
    -------
    F_nut [N]
    """
    tau   = knee_torque_static(theta_hip, theta_knee, body_mass=body_mass)
    lever = nut_lever_arm(theta_knee)
    if lever < 1e-6:
        return float("inf")
    return tau / lever


def motor_torque_from_nut_force(force_n: float) -> float:
    """
    Motor shaft torque required for a given nut force.

    τ_motor = F_nut · (lead / 2π) / η

    Returns
    -------
    τ_motor [N·m]
    """
    return force_n * (BALLSCREW_LEAD / (2.0 * np.pi)) / BALLSCREW_EFF


def motor_speed_rpm(theta_knee: float, omega_knee_rad_s: float) -> float:
    """
    Motor shaft speed (RPM) required to produce a given knee angular velocity.

    ω_motor = v_nut / (lead / 2π)
    v_nut   = R_ARM · |sin θ_knee| · ω_knee

    Returns
    -------
    speed [RPM]
    """
    v = abs(nut_velocity(theta_knee, omega_knee_rad_s))
    omega_motor_rad_s = v / (BALLSCREW_LEAD / (2.0 * np.pi))
    return omega_motor_rad_s * 60.0 / (2.0 * np.pi)


def motor_power(theta_knee: float, omega_knee_rad_s: float, force_n: float) -> float:
    """
    Instantaneous mechanical power the motor must deliver.

    P = F_nut · v_nut

    Returns
    -------
    P [W]
    """
    v = abs(nut_velocity(theta_knee, omega_knee_rad_s))
    return force_n * v


def analyse_gait_forces(
    theta_hip_seq: np.ndarray,
    theta_knee_seq: np.ndarray,
    omega_knee_seq: np.ndarray | None = None,
    body_mass: float = M_BODY,
) -> dict:
    """
    Compute nut force, motor torque, speed, and power over a gait sequence.

    Parameters
    ----------
    theta_hip_seq   : (N,) hip angles  [rad]
    theta_knee_seq  : (N,) knee angles [rad]
    omega_knee_seq  : (N,) knee angular velocities [rad/s]  (optional)
    body_mass       : Body frame mass [kg]

    Returns
    -------
    dict with arrays:
      tau_knee, nut_force, motor_torque, motor_rpm, motor_power_w,
      lever_arm, nut_position_m, foot_x, foot_z
    """
    n = len(theta_hip_seq)
    if omega_knee_seq is None:
        omega_knee_seq = np.zeros(n)

    tau_k   = np.zeros(n)
    f_nut   = np.zeros(n)
    tau_m   = np.zeros(n)
    rpm_m   = np.zeros(n)
    pwr_m   = np.zeros(n)
    lever   = np.zeros(n)
    y_nut   = np.zeros(n)
    foot_x  = np.zeros(n)
    foot_z  = np.zeros(n)

    for i, (th, tk, ok) in enumerate(
        zip(theta_hip_seq, theta_knee_seq, omega_knee_seq)
    ):
        tau_k[i]  = knee_torque_static(th, tk, body_mass=body_mass)
        f_nut[i]  = nut_force_required(th, tk, body_mass=body_mass)
        tau_m[i]  = motor_torque_from_nut_force(f_nut[i])
        rpm_m[i]  = motor_speed_rpm(tk, ok)
        pwr_m[i]  = motor_power(tk, ok, f_nut[i])
        lever[i]  = nut_lever_arm(tk)
        y_nut[i]  = nut_position(tk)
        fk = forward_kinematics(th, tk)
        foot_x[i], foot_z[i] = fk["foot_pos"]

    return {
        "tau_knee":       tau_k,
        "nut_force":      f_nut,
        "motor_torque":   tau_m,
        "motor_rpm":      rpm_m,
        "motor_power_w":  pwr_m,
        "lever_arm":      lever,
        "nut_position_m": y_nut,
        "foot_x":         foot_x,
        "foot_z":         foot_z,
    }
