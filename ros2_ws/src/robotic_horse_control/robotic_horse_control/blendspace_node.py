"""
blendspace_node.py
──────────────────
ROS2 node that implements a 2D Animation Blendspace for the Robotic Horse,
analogous to Unreal Engine's Animation Blendspace.

Concept
───────
  • Several animations are placed at (x, y) points on a 2D grid where:
        x = forward_speed   [0 … 1]
        y = angular_rate   [-1 … 1]   (negative = left, positive = right)

  • At runtime, a /cmd_vel command is received and mapped to (x, y).
  • The node finds the surrounding anchor animations and bilinearly
    interpolates their joint angles frame-by-frame.
  • The blended angles are published as a JointTrajectory.

  This allows smooth transitions between:
    idle → walk forward → arc left/right → turn in place

Architecture
────────────

  /cmd_vel  (geometry_msgs/Twist)
       │   linear.x  → forward_speed
       │   angular.z → angular_rate
       ▼
  BlendspaceNode
       │  loads animations from blendspace.yaml
       │  bilinear interpolation
       ▼
  /leg_controller/joint_trajectory  (trajectory_msgs/JointTrajectory)

  /robotic_horse/blendspace_state  (std_msgs/String, JSON debug info)

Topics
──────
  Subscribed:
    /cmd_vel                         geometry_msgs/Twist
  Published:
    /leg_controller/joint_trajectory trajectory_msgs/JointTrajectory
    /robotic_horse/blendspace_state  std_msgs/String  (JSON debug)
"""

import os
import json
import math
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from builtin_interfaces.msg import Duration
from ament_index_python.packages import get_package_share_directory


JOINT_ORDER = [
    'fl_thigh_joint', 'fl_knee_joint',
    'fr_thigh_joint', 'fr_knee_joint',
    'rl_thigh_joint', 'rl_knee_joint',
    'rr_thigh_joint', 'rr_knee_joint',
]

# How many trajectory points to publish per cycle for smooth interpolation
TRAJ_LOOKAHEAD_FRAMES = 30


class Animation:
    """One loaded animation from a JSON file."""

    def __init__(self, path: str):
        with open(path) as f:
            data = json.load(f)
        self.name        = data['name']
        self.description = data.get('description', '')
        self.duration    = float(data['duration'])
        self.n_frames    = int(data['n_frames'])
        self.joint_names = data['joint_names']
        # frames[i] = list of joint angles for frame i
        self.frames      = [list(map(float, row)) for row in data['frames']]

    def sample(self, phase: float) -> list[float]:
        """
        Sample joint angles at a given normalised phase [0, 1).
        Linearly interpolates between adjacent keyframes.
        """
        phase = phase % 1.0
        raw   = phase * self.n_frames
        i0    = int(raw) % self.n_frames
        i1    = (i0 + 1) % self.n_frames
        t     = raw - int(raw)
        f0, f1 = self.frames[i0], self.frames[i1]
        return [f0[j] * (1.0 - t) + f1[j] * t for j in range(len(f0))]


class BlendspaceNode(Node):

    def __init__(self):
        super().__init__('blendspace_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('blendspace_file', '')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('max_speed',        1.0)   # m/s → normalised 1.0
        self.declare_parameter('max_angular',      1.5)   # rad/s → normalised 1.0

        pkg_share = get_package_share_directory('robotic_horse_control')
        default_bs = os.path.join(pkg_share, 'animations', 'blendspace.yaml')
        bs_file = self.get_parameter('blendspace_file').get_parameter_value().string_value
        if not bs_file:
            bs_file = default_bs

        self._max_speed   = self.get_parameter('max_speed').value
        self._max_angular = self.get_parameter('max_angular').value

        # ── Load blendspace ────────────────────────────────────────────
        self._animations: dict[str, Animation] = {}
        self._anchors: list[dict]               = []   # {x, y, anim}
        self._load_blendspace(bs_file)

        # ── State ─────────────────────────────────────────────────────
        self._speed   = 0.0
        self._angular = 0.0
        self._phase   = 0.0    # gait phase [0, 1)

        # ── ROS interfaces ─────────────────────────────────────────────
        self._sub_vel = self.create_subscription(
            Twist, '/cmd_vel', self._on_cmd_vel, 10)

        self._pub_traj = self.create_publisher(
            JointTrajectory, '/leg_controller/joint_trajectory', 10)

        self._pub_state = self.create_publisher(
            String, '/robotic_horse/blendspace_state', 10)

        hz = self.get_parameter('publish_rate_hz').value
        self._dt = 1.0 / hz
        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f'Blendspace node ready — {len(self._anchors)} anchor animations loaded.'
        )

    # ── Blendspace loader ──────────────────────────────────────────────

    def _load_blendspace(self, yaml_path: str):
        anim_dir = os.path.dirname(yaml_path)
        with open(yaml_path) as f:
            cfg = yaml.safe_load(f)

        for entry in cfg['animations']:
            filepath = os.path.join(anim_dir, entry['file'])
            if not os.path.exists(filepath):
                self.get_logger().warn(f'Animation file not found: {filepath}')
                continue
            anim = Animation(filepath)
            self._animations[anim.name] = anim
            self._anchors.append({
                'x':    float(entry['x']),
                'y':    float(entry['y']),
                'anim': anim,
            })
            self.get_logger().info(
                f'  Loaded [{entry["x"]:+.1f}, {entry["y"]:+.1f}] → {anim.name}'
            )

    # ── cmd_vel subscriber ─────────────────────────────────────────────

    def _on_cmd_vel(self, msg: Twist):
        # Normalise to [0,1] and [-1,1]
        self._speed   = max(0.0, min(1.0,
                           msg.linear.x / self._max_speed))
        self._angular = max(-1.0, min(1.0,
                           msg.angular.z / self._max_angular))

    # ── Bilinear interpolation ─────────────────────────────────────────

    def _blend(self, px: float, py: float, phase: float) -> list[float]:
        """
        Blend animations at point (px, py) using inverse-distance weighting
        over all anchors.  Works for any anchor layout, not just grids.
        """
        if not self._anchors:
            return [0.0] * 8

        # Compute squared distance to each anchor
        dists = []
        for a in self._anchors:
            dx = px - a['x']
            dy = py - a['y']
            d2 = dx * dx + dy * dy
            dists.append(d2)

        # If we are exactly on an anchor, use it directly
        min_d2 = min(dists)
        if min_d2 < 1e-8:
            exact = self._anchors[dists.index(min_d2)]
            return exact['anim'].sample(phase)

        # Inverse-distance weighting (power = 2)
        weights = [1.0 / d for d in dists]
        total   = sum(weights)
        weights = [w / total for w in weights]

        n_joints = len(JOINT_ORDER)
        blended  = [0.0] * n_joints

        for w, anchor in zip(weights, self._anchors):
            sampled = anchor['anim'].sample(phase)
            # Ensure we only use the joints in JOINT_ORDER
            for j, jname in enumerate(JOINT_ORDER):
                if jname in anchor['anim'].joint_names:
                    src_idx = anchor['anim'].joint_names.index(jname)
                    blended[j] += w * sampled[src_idx]

        return blended

    # ── Timer tick ─────────────────────────────────────────────────────

    def _tick(self):
        # Advance phase based on current speed
        # Faster speed = faster gait cycle
        speed_factor = max(0.2, self._speed)   # keep phase moving even at idle
        phase_rate   = speed_factor / 1.0       # full cycle per second at speed=1
        self._phase  = (self._phase + self._dt * phase_rate) % 1.0

        # Build a short lookahead trajectory
        msg             = JointTrajectory()
        msg.joint_names = JOINT_ORDER

        for k in range(TRAJ_LOOKAHEAD_FRAMES):
            lookahead_phase = (self._phase + k * self._dt * phase_rate) % 1.0
            positions = self._blend(self._speed, self._angular, lookahead_phase)

            pt = JointTrajectoryPoint()
            pt.positions  = positions
            pt.velocities = [0.0] * 8
            t = k * self._dt
            pt.time_from_start = Duration(
                sec=int(t),
                nanosec=int((t % 1.0) * 1e9),
            )
            msg.points.append(pt)

        self._pub_traj.publish(msg)

        # ── Debug state message ───────────────────────────────────────
        state = {
            'speed':        round(self._speed, 3),
            'angular_rate': round(self._angular, 3),
            'phase':        round(self._phase, 3),
        }
        smsg      = String()
        smsg.data = json.dumps(state)
        self._pub_state.publish(smsg)


def main(args=None):
    rclpy.init(args=args)
    node = BlendspaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
