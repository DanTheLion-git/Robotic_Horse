"""
force_node.py  —  ROS2 node that subscribes to /joint_states, computes the
required ballscrew nut force and motor torque for each leg, and logs/publishes
the results.

Topics subscribed:
    /joint_states  [sensor_msgs/JointState]

Topics published:
    /robotic_horse/ballscrew_forces  [std_msgs/Float32MultiArray]
        Layout: [fl_force, fr_force, rl_force, rr_force]  [N]

    /robotic_horse/motor_torques     [std_msgs/Float32MultiArray]
        Layout: [fl_torque, fr_torque, rl_torque, rr_torque]  [N·m]
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


# ── Physical constants (mirrors force_calculator.py) ───────────────────────
G              = 9.81
M_BODY         = 20.0
M_THIGH        = 1.5
M_SHANK        = 1.0
M_FOOT         = 0.2
NUM_LEGS       = 4
L1             = 0.45
L2             = 0.50
R_ARM          = 0.08    # ballscrew lever arm [m]
BALLSCREW_LEAD = 0.005   # m/rev
BALLSCREW_EFF  = 0.90


def _fk(th: float, tk: float):
    """Forward kinematics — returns (knee_x, foot_x)."""
    knee_x = L1 * math.sin(th)
    knee_z = -L1 * math.cos(th)
    shank_a = th + tk
    foot_x = knee_x + L2 * math.sin(shank_a)
    return knee_x, foot_x


def _knee_torque(th: float, tk: float) -> float:
    """Static knee torque [N·m]."""
    shank_a = th + tk
    tau_shank = M_SHANK * G * (L2 / 2) * abs(math.sin(shank_a))
    tau_foot  = M_FOOT  * G *  L2      * abs(math.sin(shank_a))
    knee_x, foot_x = _fk(th, tk)
    moment_arm   = abs(foot_x - knee_x)
    vertical_load = (M_BODY / NUM_LEGS + M_THIGH + M_SHANK + M_FOOT) * G
    return tau_shank + tau_foot + vertical_load * moment_arm


def _nut_force(th: float, tk: float) -> float:
    """Required ballscrew nut force [N]."""
    lever = R_ARM * abs(math.sin(tk))
    if lever < 1e-6:
        return 0.0
    return _knee_torque(th, tk) / lever


def _motor_torque(force_n: float) -> float:
    """Motor shaft torque [N·m] from nut force."""
    return force_n * (BALLSCREW_LEAD / (2 * math.pi)) / BALLSCREW_EFF


LEG_JOINTS = {
    'fl': ('fl_thigh_joint', 'fl_knee_joint'),
    'fr': ('fr_thigh_joint', 'fr_knee_joint'),
    'rl': ('rl_thigh_joint', 'rl_knee_joint'),
    'rr': ('rr_thigh_joint', 'rr_knee_joint'),
}

# Hip socket position in base_link frame (hip_link origin + 0.08 down)
HIP_SOCKET = {
    'fl': ( 0.35,  0.18, -0.13),
    'fr': ( 0.35, -0.18, -0.13),
    'rl': (-0.35,  0.18, -0.13),
    'rr': (-0.35, -0.18, -0.13),
}

FORCE_MAX = 1500.0   # N – maps to full red / max arrow length
ARROW_SCALE = 1.0 / FORCE_MAX   # 1500 N → 1.0 m arrow


def _force_color(force_n: float):
    """Return (r, g, b) on a green→yellow→red gradient for 0..FORCE_MAX."""
    t = max(0.0, min(1.0, force_n / FORCE_MAX))
    r = min(1.0, 2.0 * t)
    g = min(1.0, 2.0 * (1.0 - t))
    return r, g, 0.0


def _knee_pos(leg: str, th: float) -> tuple:
    """Knee position in base_link frame given hip socket and thigh angle."""
    sx, sy, sz = HIP_SOCKET[leg]
    kx = sx + L1 * math.sin(th)
    ky = sy
    kz = sz - L1 * math.cos(th)
    return kx, ky, kz


class ForceNode(Node):
    def __init__(self):
        super().__init__('force_node')

        self._sub = self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10
        )
        self._pub_force  = self.create_publisher(
            Float32MultiArray, '/robotic_horse/ballscrew_forces', 10
        )
        self._pub_torque = self.create_publisher(
            Float32MultiArray, '/robotic_horse/motor_torques', 10
        )
        self._pub_markers = self.create_publisher(
            MarkerArray, '/robotic_horse/force_markers', 10
        )
        self._joint_pos: dict[str, float] = {}
        self.get_logger().info('Force node started — monitoring ballscrew forces.')

    def _on_joint_states(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._joint_pos[name] = pos

        forces  = []
        torques = []
        for leg in ('fl', 'fr', 'rl', 'rr'):
            thigh_j, knee_j = LEG_JOINTS[leg]
            th = self._joint_pos.get(thigh_j, 0.0)
            tk = self._joint_pos.get(knee_j,  -1.0)  # default ~60° bend
            f  = _nut_force(th, tk)
            t  = _motor_torque(f)
            forces.append(f)
            torques.append(t)

        self._pub_force.publish(self._make_msg(forces))
        self._pub_torque.publish(self._make_msg(torques))
        self._pub_markers.publish(
            self._make_markers(list(zip(('fl', 'fr', 'rl', 'rr'), forces)))
        )

    def _make_markers(self, leg_forces: list) -> MarkerArray:
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, (leg, force) in enumerate(leg_forces):
            th  = self._joint_pos.get(LEG_JOINTS[leg][0], 0.0)
            kx, ky, kz = _knee_pos(leg, th)
            r, g, b = _force_color(force)
            arrow_len = force * ARROW_SCALE

            # Arrow marker: points downward from knee, length ∝ force
            arrow = Marker()
            arrow.header.frame_id = 'base_link'
            arrow.header.stamp    = now
            arrow.ns              = 'ballscrew_forces'
            arrow.id              = i
            arrow.type            = Marker.ARROW
            arrow.action          = Marker.ADD
            arrow.scale.x = 0.015   # shaft diameter
            arrow.scale.y = 0.03    # head diameter
            arrow.scale.z = 0.06    # head length
            arrow.color.r = r
            arrow.color.g = g
            arrow.color.b = b
            arrow.color.a = 0.9
            start = Point(x=kx, y=ky, z=kz)
            end   = Point(x=kx, y=ky, z=kz - arrow_len)
            arrow.points = [start, end]
            arr.markers.append(arrow)

            # Text label above the knee
            label = Marker()
            label.header.frame_id = 'base_link'
            label.header.stamp    = now
            label.ns              = 'ballscrew_labels'
            label.id              = i + 10
            label.type            = Marker.TEXT_VIEW_FACING
            label.action          = Marker.ADD
            label.pose.position.x = kx
            label.pose.position.y = ky
            label.pose.position.z = kz + 0.08
            label.pose.orientation.w = 1.0
            label.scale.z         = 0.07
            label.color.r         = r
            label.color.g         = g
            label.color.b         = b
            label.color.a         = 1.0
            label.text            = f'{leg.upper()}: {force:.0f} N'
            arr.markers.append(label)

        return arr

    @staticmethod
    def _make_msg(values: list) -> Float32MultiArray:
        msg = Float32MultiArray()
        dim = MultiArrayDimension()
        dim.label  = 'legs'
        dim.size   = len(values)
        dim.stride = len(values)
        msg.layout.dim = [dim]
        msg.data = [float(v) for v in values]
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = ForceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
