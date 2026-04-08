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
        rclpy.shutdown()
