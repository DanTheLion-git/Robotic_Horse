"""
gait_node.py  —  ROS2 node that runs the trot gait planner and publishes
joint trajectory commands to the leg_controller.

Topics published:
    /leg_controller/joint_trajectory  [trajectory_msgs/JointTrajectory]

The gait cycle repeats indefinitely.  Joint angles are computed using the
same inverse-kinematics model as the PyBullet prototype (L1=0.45, L2=0.50).
"""

import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# ── Gait / kinematics constants (mirrors robot/kinematics and robot/gait) ──
L1 = 0.45          # thigh length [m]
L2 = 0.50          # shank length [m]
BODY_HEIGHT   = 0.90   # nominal hip-to-ground height [m]
STEP_LENGTH   = 0.20   # stride length [m]
STEP_HEIGHT   = 0.12   # foot lift [m]
SWING_FRAC    = 0.40   # fraction of cycle in swing

PHASE_OFFSET = {'fl': 0.0, 'fr': 0.5, 'rl': 0.5, 'rr': 0.0}

JOINT_ORDER = [
    'fl_thigh_joint', 'fl_knee_joint',
    'fr_thigh_joint', 'fr_knee_joint',
    'rl_thigh_joint', 'rl_knee_joint',
    'rr_thigh_joint', 'rr_knee_joint',
]


# ── Pure kinematics (no external imports needed in ROS2 node) ──────────────

def _ik(foot_x: float, foot_z: float):
    """2-link IK.  Returns (theta_hip, theta_knee)."""
    r = math.sqrt(foot_x**2 + foot_z**2)
    r = max(abs(L1 - L2) + 0.001, min(r, L1 + L2 - 0.001))
    cos_k = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_k = max(-1.0, min(1.0, cos_k))
    theta_knee = +math.acos(cos_k)  # elbow-up (Spot-like)
    alpha = math.atan2(foot_x, -foot_z)
    beta  = math.asin(max(-1.0, min(1.0, L2 * math.sin(theta_knee) / r)))
    return alpha - beta, theta_knee


def _foot_target(leg: str, phase: float):
    """Return (foot_x, foot_z) in hip frame for a given gait phase."""
    local = (phase - PHASE_OFFSET[leg]) % 1.0
    if local < SWING_FRAC:
        p = local / SWING_FRAC
        x = -STEP_LENGTH / 2 + STEP_LENGTH * p
        z = -(BODY_HEIGHT - STEP_HEIGHT * math.sin(math.pi * p))
    else:
        p = (local - SWING_FRAC) / (1.0 - SWING_FRAC)
        half = STEP_LENGTH / (2 * BODY_HEIGHT)
        x = (half - 2 * half * p) * BODY_HEIGHT
        z = -BODY_HEIGHT
    return x, z


class GaitNode(Node):
    def __init__(self):
        super().__init__('gait_node')

        self._pub = self.create_publisher(
            JointTrajectory,
            '/leg_controller/joint_trajectory',
            10,
        )

        # Gait parameters
        self._gait_period = 1.0    # seconds per cycle
        self._n_points    = 50     # trajectory points per publish
        self._dt          = self._gait_period / self._n_points

        # Publish a full trajectory every half-cycle so the controller always
        # has a look-ahead buffer.
        self._phase = 0.0
        self.create_timer(self._gait_period / 2, self._publish_trajectory)
        self.get_logger().info('Gait node started — trot gait active.')

    def _publish_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = JOINT_ORDER

        for i in range(self._n_points):
            phase = (self._phase + i / self._n_points) % 1.0
            positions = []
            for leg in ('fl', 'fr', 'rl', 'rr'):
                fx, fz = _foot_target(leg, phase)
                th, tk = _ik(fx, fz)
                positions += [th, tk]

            pt = JointTrajectoryPoint()
            pt.positions = positions
            pt.velocities = [0.0] * 8
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
