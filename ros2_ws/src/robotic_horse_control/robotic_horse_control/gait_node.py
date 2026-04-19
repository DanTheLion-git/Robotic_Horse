"""
gait_node.py  —  ROS2 node that runs the trot gait planner and publishes
joint trajectory commands to the leg_controller.

Topics published:
    /leg_controller/joint_trajectory  [trajectory_msgs/JointTrajectory]

The gait cycle repeats indefinitely.  Joint angles are computed using the
Highland Cow kinematics (L1=0.20, L2=0.18, 3-segment legs with cannon bone).

NOTE: The blendspace_node.py is the preferred controller for the Highland Cow
build (supports WALK/TROT gaits, IMU balance, teleop). This gait_node is kept
as a simple trot-only fallback.
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
CANNON_LEAN = 0.08 # cannon forward lean [rad]
FOOT_R = 0.04      # hoof radius [m]

# Ankle height = hip_height - cannon - hoof
BODY_HEIGHT   = 0.464  # nominal hip-to-ground height [m]
ANKLE_HEIGHT  = BODY_HEIGHT - L3 * math.cos(CANNON_LEAN) - FOOT_R  # ~0.324m

STEP_LENGTH   = 0.10   # stride length [m]
STEP_HEIGHT   = 0.06   # foot lift [m]
SWING_FRAC    = 0.40   # fraction of cycle in swing
MAX_STEP      = 0.14   # hard cap on stride half-length [m]

PHASE_OFFSET = {'fl': 0.0, 'fr': 0.5, 'rl': 0.5, 'rr': 0.0}

JOINT_ORDER = [
    'fl_hip_joint', 'fl_thigh_joint', 'fl_knee_joint', 'fl_cannon_joint',
    'fr_hip_joint', 'fr_thigh_joint', 'fr_knee_joint', 'fr_cannon_joint',
    'rl_hip_joint', 'rl_thigh_joint', 'rl_knee_joint', 'rl_cannon_joint',
    'rr_hip_joint', 'rr_thigh_joint', 'rr_knee_joint', 'rr_cannon_joint',
]

FRONT_LEGS = ('fl', 'fr')
REAR_LEGS  = ('rl', 'rr')


# ── Pure kinematics (no external imports needed in ROS2 node) ──────────────

def _ik(foot_x: float, foot_z: float, elbow_up: bool = False):
    """2-link IK targeting ankle.  Returns (theta_hip, theta_knee)."""
    r = math.sqrt(foot_x**2 + foot_z**2)
    r = max(abs(L1 - L2) + 0.001, min(r, L1 + L2 - 0.001))
    cos_k = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_k = max(-1.0, min(1.0, cos_k))
    if elbow_up:
        theta_knee = +math.acos(cos_k)  # rear leg (hock backward)
        alpha = math.atan2(foot_x, -foot_z)
        beta  = math.asin(max(-1.0, min(1.0, L2 * math.sin(theta_knee) / r)))
        return alpha - beta, theta_knee
    else:
        theta_knee = -math.acos(cos_k)  # front leg (carpal forward)
        alpha = math.atan2(foot_x, -foot_z)
        beta  = math.asin(max(-1.0, min(1.0, L2 * math.sin(abs(theta_knee)) / r)))
        return alpha + beta, theta_knee


def _foot_target(leg: str, phase: float):
    """Return (foot_x, foot_z) in hip frame for a given gait phase (ankle target)."""
    local = (phase - PHASE_OFFSET[leg]) % 1.0
    if local < SWING_FRAC:
        p = local / SWING_FRAC
        x = -STEP_LENGTH / 2 + STEP_LENGTH * p
        x = max(-MAX_STEP, min(MAX_STEP, x))
        z = -(ANKLE_HEIGHT - STEP_HEIGHT * math.sin(math.pi * p))
    else:
        p = (local - SWING_FRAC) / (1.0 - SWING_FRAC)
        half = STEP_LENGTH / (2 * ANKLE_HEIGHT)
        x = (half - 2 * half * p) * ANKLE_HEIGHT
        x = max(-MAX_STEP, min(MAX_STEP, x))
        z = -ANKLE_HEIGHT
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
                elbow_up = leg in REAR_LEGS
                th, tk = _ik(fx, fz, elbow_up=elbow_up)
                cannon = CANNON_LEAN - (th + tk)
                positions += [0.0, th, tk, cannon]  # hip_yaw=0, thigh, knee, cannon

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
