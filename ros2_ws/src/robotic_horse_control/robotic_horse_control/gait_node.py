"""
gait_node.py  —  ROS2 node that runs a bovine gait planner and publishes
joint trajectory commands to the leg_controller.

Topics published:
    /leg_controller/joint_trajectory  [trajectory_msgs/JointTrajectory]

The gait cycle repeats indefinitely. Joint angles computed using Highland Cow
kinematics with bovine leg mechanics:
  Front legs: passive cannon linkage (parallelogram)
  Rear legs:  reciprocal apparatus (stifle-hock tendon coupling)

NOTE: The blendspace_node.py is the preferred controller (supports teleop,
IMU balance, gait transitions). This gait_node is a simple walk-only fallback.
"""

import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# ── Highland Cow geometry (must match URDF xacro) ──────────────────────────
L1 = 0.20          # thigh length [m]
L2 = 0.18          # shank length [m]
L3 = 0.10          # cannon bone length [m]
CANNON_LEAN = 0.08 # front leg: cannon forward lean [rad]
FOOT_R = 0.04      # hoof radius [m]

# Reciprocal apparatus (rear legs)
RECIP_RATIO  = 0.85
RECIP_OFFSET = -0.47 + RECIP_RATIO * 1.10  # ≈ 0.465

BODY_HEIGHT   = 0.464
ANKLE_HEIGHT  = BODY_HEIGHT - L3 * math.cos(CANNON_LEAN) - FOOT_R  # ~0.324m

# Bovine walk gait parameters
STEP_LENGTH   = 0.10
STEP_HEIGHT_FRONT = 0.06
STEP_HEIGHT_REAR  = 0.05
SWING_FRAC    = 0.30   # 30% swing / 70% stance (bovine walk duty factor)
MAX_STEP      = 0.14

# 4-beat lateral sequence: RL → FL → RR → FR
PHASE_OFFSET = {'rl': 0.0, 'fl': 0.25, 'rr': 0.50, 'fr': 0.75}

JOINT_ORDER = [
    'fl_hip_joint', 'fl_thigh_joint', 'fl_knee_joint', 'fl_cannon_joint',
    'fr_hip_joint', 'fr_thigh_joint', 'fr_knee_joint', 'fr_cannon_joint',
    'rl_hip_joint', 'rl_thigh_joint', 'rl_knee_joint', 'rl_cannon_joint',
    'rr_hip_joint', 'rr_thigh_joint', 'rr_knee_joint', 'rr_cannon_joint',
]

FRONT_LEGS = ('fl', 'fr')
REAR_LEGS  = ('rl', 'rr')


def _ik(foot_x: float, foot_z: float, elbow_up: bool = False):
    """2-link IK targeting ankle. Returns (theta_hip, theta_knee)."""
    r = math.sqrt(foot_x**2 + foot_z**2)
    r = max(abs(L1 - L2) + 0.001, min(r, L1 + L2 - 0.001))
    cos_k = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_k = max(-1.0, min(1.0, cos_k))
    if elbow_up:
        theta_knee = +math.acos(cos_k)
        alpha = math.atan2(foot_x, -foot_z)
        beta  = math.asin(max(-1.0, min(1.0, L2 * math.sin(theta_knee) / r)))
        return alpha - beta, theta_knee
    else:
        theta_knee = -math.acos(cos_k)
        alpha = math.atan2(foot_x, -foot_z)
        beta  = math.asin(max(-1.0, min(1.0, L2 * math.sin(abs(theta_knee)) / r)))
        return alpha + beta, theta_knee


def _cannon(leg: str, thigh: float, knee: float) -> float:
    """Cannon angle: front = passive linkage, rear = reciprocal apparatus."""
    if leg in REAR_LEGS:
        return RECIP_OFFSET - RECIP_RATIO * knee
    return CANNON_LEAN - (thigh + knee)


def _foot_target(leg: str, phase: float):
    """Return (foot_x, foot_z) in hip frame for a given gait phase."""
    is_rear = leg in REAR_LEGS
    step_h = STEP_HEIGHT_REAR if is_rear else STEP_HEIGHT_FRONT

    local = (phase - PHASE_OFFSET[leg]) % 1.0
    if local < SWING_FRAC:
        # 3-phase swing arc (bovine lift-carry-plant)
        p = local / SWING_FRAC
        LIFT_END = 0.30
        CARRY_END = 0.70
        if p < LIFT_END:
            t = p / LIFT_END
            lift = step_h * math.sin(math.pi * 0.5 * t)
            t_fwd = t * 0.15
        elif p < CARRY_END:
            t = (p - LIFT_END) / (CARRY_END - LIFT_END)
            lift = step_h
            t_fwd = 0.15 + t * 0.70
        else:
            t = (p - CARRY_END) / (1.0 - CARRY_END)
            lift = step_h * math.cos(math.pi * 0.5 * t)
            t_fwd = 0.85 + t * 0.15
        x = -STEP_LENGTH / 2 + STEP_LENGTH * t_fwd
        x = max(-MAX_STEP, min(MAX_STEP, x))
        z = -(ANKLE_HEIGHT - lift)
    else:
        # Stance: foot on ground, body advances
        p = (local - SWING_FRAC) / (1.0 - SWING_FRAC)
        half = STEP_LENGTH / 2
        x = half * (1.0 - 2.0 * p)
        x = max(-MAX_STEP, min(MAX_STEP, x))
        z = -(ANKLE_HEIGHT + 0.015)  # press 15mm for contact
    return x, z


class GaitNode(Node):
    def __init__(self):
        super().__init__('gait_node')

        self._pub = self.create_publisher(
            JointTrajectory,
            '/leg_controller/joint_trajectory',
            10,
        )

        self._gait_period = 1.1    # bovine walk period
        self._n_points    = 50
        self._dt          = self._gait_period / self._n_points

        self._phase = 0.0
        self.create_timer(self._gait_period / 2, self._publish_trajectory)
        self.get_logger().info('Gait node started — bovine walk (4-beat lateral)')

    def _publish_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = JOINT_ORDER

        for i in range(self._n_points):
            phase = (self._phase + i / self._n_points) % 1.0
            positions = []
            for leg in ('fl', 'fr', 'rl', 'rr'):
                fx, fz = _foot_target(leg, phase)
                elbow_up = leg in REAR_LEGS
                th, tk = _ik(fx, fz, elbow_up=elbow_up)
                cannon = _cannon(leg, th, tk)
                positions += [0.0, th, tk, cannon]

            pt = JointTrajectoryPoint()
            pt.positions = positions
            pt.velocities = [0.0] * 16
            pt.time_from_start = Duration(
                sec=int(i * self._dt),
                nanosec=int((i * self._dt % 1.0) * 1e9),
            )
            msg.points.append(pt)

        self._pub.publish(msg)
        self._phase = (self._phase + 0.5) % 1.0


def main(args=None):
    rclpy.init(args=args)
    node = GaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
