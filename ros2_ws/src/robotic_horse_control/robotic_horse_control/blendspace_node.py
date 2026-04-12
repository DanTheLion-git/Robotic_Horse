"""
blendspace_node.py  —  Elk-accurate velocity-driven gait controller (v3).

Deer/elk leg anatomy (3-segment):
  thigh (femur/humerus, 0.38m)
  → knee (stifle/carpus, 0.38m shank)
  → cannon bone (metatarsal/metacarpal, 0.22m)  ← NEW
  → hoof

Leg configuration:
  FRONT (FL/FR): elbow-DOWN — carpal ("wrist") bends FORWARD (away from cart)
                 Neutral: thigh=+0.564, knee=-1.127, cannon=+0.714
  REAR  (RL/RR): elbow-UP   — hock bends BACKWARD  (toward cart)
                 Neutral: thigh=-0.564, knee=+1.127, cannon=-0.414

Cannon bone (pantograph principle):
  A 4-bar linkage keeps the cannon near-vertical regardless of leg pose.
  Simulated as: cannon_angle = CANNON_LEAN - (thigh + knee)
  This is how red deer check-ligaments work biologically — zero extra motors.

Elk gaits (retained from v2):
  WALK   — 4-beat lateral sequence RL→FL→RR→FR, period 1.6 s
  TROT   — Diagonal pairs with suspension phase, period 0.90 s
  GALLOP — Rotary gallop FL→RL→FR→RR, period 0.50 s

Elk step profile (retained from v2):
  Height: sin(pi * p^0.7) — peaks at 40% of swing, broad sustained lift
  Forward: p^1.3          — leg lifts THEN swings forward

Topics:
  /cmd_vel   geometry_msgs/Twist   — linear.x=speed, angular.z=turn
  /cmd_gait  std_msgs/String       — "walk" | "trot" | "gallop"
  /joint_states sensor_msgs/JointState
"""

import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from builtin_interfaces.msg import Duration


# ── Robot geometry (must match URDF) ─────────────────────────────────────────
L1 = 0.38           # thigh length        [m]
L2 = 0.38           # shank length        [m]
L3 = 0.22           # cannon bone length  [m]
CANNON_LEAN = 0.15  # cannon forward lean from vertical [rad] (~8.6 deg)
FOOT_R = 0.04       # foot sphere radius  [m]

# ANKLE_HEIGHT = hip-to-ankle drop (IK targets ankle, not foot sphere)
ANKLE_HEIGHT = 0.90 - L3 * math.cos(CANNON_LEAN) - FOOT_R  # = 0.6425m
BODY_HEIGHT  = 0.90     # hip-to-ground height [m]

# Hip pivot positions relative to base_link (x=fwd, y=left, z=up)
LEG_POS = {
    'fl': ( 0.35,  0.18),
    'fr': ( 0.35, -0.18),
    'rl': (-0.35,  0.18),
    'rr': (-0.35, -0.18),
}
HALF_BODY_LENGTH = 0.35

FRONT_LEGS = ('fl', 'fr')
REAR_LEGS  = ('rl', 'rr')

# ── Neutral poses (verified by forward kinematics — foot at ground) ───────────
NEUTRAL_THIGH_FRONT  = +0.564
NEUTRAL_KNEE_FRONT   = -1.127
NEUTRAL_CANNON_FRONT = +0.714   # = CANNON_LEAN - (0.564 + -1.127) = 0.563

NEUTRAL_THIGH_REAR   = -0.564
NEUTRAL_KNEE_REAR    = +1.127
NEUTRAL_CANNON_REAR  = -0.414   # = CANNON_LEAN - (-0.564 + 1.127) = -0.413

# ── Elk gait parameters ───────────────────────────────────────────────────────

WALK = dict(
    phase_offsets={'rl': 0.0, 'fl': 0.25, 'rr': 0.50, 'fr': 0.75},
    swing_frac=0.35,
    step_height=0.12,
    period=1.6,
    name='WALK',
)

TROT = dict(
    phase_offsets={'fl': 0.0, 'fr': 0.5, 'rl': 0.5, 'rr': 0.0},
    swing_frac=0.50,
    step_height=0.17,
    period=0.90,
    name='TROT',
)

GALLOP = dict(
    phase_offsets={'fl': 0.0, 'rl': 0.15, 'fr': 0.50, 'rr': 0.65},
    swing_frac=0.55,
    step_height=0.22,
    period=0.50,
    name='GALLOP',
)

GAIT_SEQUENCE = [WALK, TROT, GALLOP]

# ── Control limits ────────────────────────────────────────────────────────────
MAX_STEER    = 0.45   # max cart steer angle  [rad]
MAX_SHOULDER = 0.20   # max hip Z-yaw from IK [rad]
MAX_STEP     = 0.30   # hard cap on stride half-length [m]

CONTACT_EFFORT_THRESHOLD = 45.0
CONTACT_MIN_SWING_FRAC   = 0.45

# ── Joint order (17 joints) ───────────────────────────────────────────────────
JOINT_ORDER = [
    'cart_to_shaft_steer',
    'fl_hip_joint', 'fl_thigh_joint', 'fl_knee_joint', 'fl_cannon_joint',
    'fr_hip_joint', 'fr_thigh_joint', 'fr_knee_joint', 'fr_cannon_joint',
    'rl_hip_joint', 'rl_thigh_joint', 'rl_knee_joint', 'rl_cannon_joint',
    'rr_hip_joint', 'rr_thigh_joint', 'rr_knee_joint', 'rr_cannon_joint',
]
N_JOINTS  = len(JOINT_ORDER)   # 17
LEG_ORDER = ('fl', 'fr', 'rl', 'rr')


# ── 3D IK — front legs (elbow-DOWN, carpal bends away from cart) ──────────────

def _ik_3d_front(fx: float, fy: float, fz: float):
    """
    Front-leg IK: carpal (wrist) bends FORWARD, away from cart.
    knee < 0, thigh > 0.
    Returns (hip_z, thigh_y, knee_y, cannon_y).
    """
    reach = math.sqrt(fx**2 + fz**2)
    hip_z = math.atan2(fy, reach) if reach > 0.001 else 0.0
    hip_z = max(-MAX_SHOULDER, min(MAX_SHOULDER, hip_z))

    fx_2d = fx * math.cos(hip_z) + fy * math.sin(hip_z)
    fz_2d = fz   # vertical component unchanged

    r     = math.sqrt(fx_2d**2 + fz_2d**2)
    r     = max(abs(L1 - L2) + 0.001, min(r, L1 + L2 - 0.001))
    cos_k = (r**2 - L1**2 - L2**2) / (2.0 * L1 * L2)
    cos_k = max(-1.0, min(1.0, cos_k))

    knee   = -math.acos(cos_k)          # negative = elbow-DOWN
    alpha  = math.atan2(fx_2d, -fz_2d)
    beta   = math.asin(max(-1.0, min(1.0, L2 * math.sin(abs(knee)) / r)))
    thigh  = alpha + beta               # positive = leans forward

    cannon = CANNON_LEAN - (thigh + knee)
    cannon = max(-2.0, min(2.0, cannon))
    return hip_z, thigh, knee, cannon


# ── 3D IK — rear legs (elbow-UP, hock bends toward cart) ─────────────────────

def _ik_3d_rear(fx: float, fy: float, fz: float):
    """
    Rear-leg IK: hock bends BACKWARD, toward cart.
    knee > 0, thigh < 0.
    Returns (hip_z, thigh_y, knee_y, cannon_y).
    """
    reach = math.sqrt(fx**2 + fz**2)
    hip_z = math.atan2(fy, reach) if reach > 0.001 else 0.0
    hip_z = max(-MAX_SHOULDER, min(MAX_SHOULDER, hip_z))

    fx_2d = fx * math.cos(hip_z) + fy * math.sin(hip_z)
    fz_2d = fz

    r     = math.sqrt(fx_2d**2 + fz_2d**2)
    r     = max(abs(L1 - L2) + 0.001, min(r, L1 + L2 - 0.001))
    cos_k = (r**2 - L1**2 - L2**2) / (2.0 * L1 * L2)
    cos_k = max(-1.0, min(1.0, cos_k))

    knee   = +math.acos(cos_k)          # positive = elbow-UP (hock-back)
    alpha  = math.atan2(fx_2d, -fz_2d)
    beta   = math.asin(max(-1.0, min(1.0, L2 * math.sin(knee) / r)))
    thigh  = alpha - beta               # negative = leans backward

    cannon = CANNON_LEAN - (thigh + knee)
    cannon = max(-2.0, min(2.0, cannon))
    return hip_z, thigh, knee, cannon


# ── Elk-specific swing profile ────────────────────────────────────────────────

def _elk_swing(p: float, fx_mean: float, fy_mean: float,
               ankle_height: float, step_height: float):
    """
    Elk-inspired swing arc: sin(pi*p^0.7) height, p^1.3 forward swing.
    Height peaks at 40% of swing; forward reach is delayed (lift first).
    Targets the ANKLE position (IK handles cannon+foot below that).
    """
    lift = step_height * math.sin(math.pi * (p ** 0.7))
    t    = p ** 1.3
    fx   = -fx_mean + 2.0 * fx_mean * t
    fy   = -fy_mean + 2.0 * fy_mean * t
    fz   = -(ankle_height - lift)
    return fx, fy, fz


# ── Arc-following foot target ─────────────────────────────────────────────────

def _foot_target_3d(leg: str, phase: float, linear_v: float, angular_v: float,
                    ankle_height: float, gait: dict):
    """
    Raibert arc-following foot target.
    stride_half = leg_arc_velocity × T_stance / 2
    Returns (fx, fy, fz) in hip frame, targeting the ANKLE.
    """
    lx, ly   = LEG_POS[leg]
    T_stance = gait['period'] * (1.0 - gait['swing_frac'])

    fx_mean  = (linear_v  - angular_v * ly) * T_stance / 2.0
    fy_mean  = (angular_v * lx)             * T_stance / 2.0
    fx_mean  = max(-MAX_STEP,       min(MAX_STEP,       fx_mean))
    fy_mean  = max(-MAX_STEP * 0.5, min(MAX_STEP * 0.5, fy_mean))

    local    = (phase - gait['phase_offsets'][leg]) % 1.0

    if local < gait['swing_frac']:
        p = local / gait['swing_frac']
        return _elk_swing(p, fx_mean, fy_mean, ankle_height, gait['step_height'])
    else:
        p  = (local - gait['swing_frac']) / (1.0 - gait['swing_frac'])
        fx = fx_mean * (1.0 - 2.0 * p)
        fy = fy_mean * (1.0 - 2.0 * p)
        fz = -ankle_height
        return fx, fy, fz


# ── Main node ─────────────────────────────────────────────────────────────────

class BlendspaceNode(Node):

    def __init__(self):
        super().__init__('blendspace_node')

        self._pub = self.create_publisher(
            JointTrajectory, '/leg_controller/joint_trajectory', 10)
        self.create_subscription(Twist,      '/cmd_vel',      self._cmd_vel_cb,      10)
        self.create_subscription(String,     '/cmd_gait',     self._gait_cb,         10)
        self.create_subscription(JointState, '/joint_states', self._joint_states_cb, 10)

        self._linear_x  = 0.0
        self._angular_z = 0.0

        self._gait_idx  = 1          # start in TROT
        self._gait      = TROT.copy()
        self._phase     = 0.0

        self._shaft_pitch   = 0.0
        self._early_contact = {l: False for l in LEG_ORDER}
        self._knee_effort   = {l: 0.0   for l in LEG_ORDER}

        self._gait_timer = self.create_timer(
            self._gait['period'] / 2.0, self._tick)

        self.get_logger().info('=' * 60)
        self.get_logger().info('  Elk Blendspace v3  —  17 joints, deer leg anatomy')
        self.get_logger().info('  Front legs: elbow-DOWN (carpal bends forward)')
        self.get_logger().info('  Rear legs:  elbow-UP   (hock bends backward)')
        self.get_logger().info('  Cannon bone: pantograph formula cannon=LEAN-(th+kn)')
        self.get_logger().info('  Gaits: WALK / TROT / GALLOP  |  Q = cycle')
        self.get_logger().info('=' * 60)

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        self._linear_x  = msg.linear.x
        self._angular_z = msg.angular.z

    def _gait_cb(self, msg: String):
        mode    = msg.data.lower().strip()
        idx_map = {'walk': 0, 'trot': 1, 'gallop': 2}
        if mode not in idx_map:
            return
        new_idx = idx_map[mode]
        if new_idx == self._gait_idx:
            return
        self._gait_idx = new_idx
        self._gait     = GAIT_SEQUENCE[new_idx].copy()
        self._reschedule_timer()
        self.get_logger().info(
            f"Gait -> {self._gait['name']}  "
            f"(period={self._gait['period']}s, "
            f"height={self._gait['step_height']}m)")

    def _reschedule_timer(self):
        self._gait_timer.cancel()
        self._gait_timer = self.create_timer(
            self._gait['period'] / 2.0, self._tick)

    def _joint_states_cb(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        if 'shaft_to_horse' in name_to_idx:
            self._shaft_pitch = msg.position[name_to_idx['shaft_to_horse']]
        if len(msg.effort) == len(msg.name):
            for leg in LEG_ORDER:
                key = f'{leg}_knee_joint'
                if key in name_to_idx:
                    self._knee_effort[leg] = abs(msg.effort[name_to_idx[key]])

    # ── Main tick ───────────────────────────────────────────────────────────

    def _tick(self):
        lin = self._linear_x
        ang = self._angular_z

        if abs(lin) < 0.01 and abs(ang) < 0.01:
            self._publish_idle()
            return

        gait = self._gait

        # Body leveling: front/rear heights compensate for cart-shaft pitch
        pitch_adj = math.tan(self._shaft_pitch) * HALF_BODY_LENGTH
        pitch_adj = max(-0.12, min(0.12, pitch_adj))
        ankle_heights = {
            'fl': ANKLE_HEIGHT + pitch_adj,
            'fr': ANKLE_HEIGHT + pitch_adj,
            'rl': ANKLE_HEIGHT - pitch_adj,
            'rr': ANKLE_HEIGHT - pitch_adj,
        }

        # Active steering: command cart steer joint
        turn_norm   = max(-1.0, min(1.0, ang / 1.0))
        steer_angle = turn_norm * MAX_STEER

        # Build trajectory — 50 waypoints over one period
        n_pts = 50
        dt    = gait['period'] / n_pts

        msg_out = JointTrajectory()
        msg_out.joint_names = JOINT_ORDER

        for i in range(n_pts):
            phase     = (self._phase + i / n_pts) % 1.0
            positions = [steer_angle]

            for leg in LEG_ORDER:
                ank_h   = ankle_heights[leg]
                eff_ph  = self._adjusted_phase(leg, phase, gait)

                fx, fy, fz = _foot_target_3d(leg, eff_ph, lin, ang, ank_h, gait)

                if leg in FRONT_LEGS:
                    hip_z, thigh, knee, cannon = _ik_3d_front(fx, fy, fz)
                else:
                    hip_z, thigh, knee, cannon = _ik_3d_rear(fx, fy, fz)

                positions += [hip_z, thigh, knee, cannon]

            pt = JointTrajectoryPoint()
            pt.positions  = positions
            pt.velocities = [0.0] * N_JOINTS
            t_sec = i * dt
            pt.time_from_start = Duration(
                sec=int(t_sec),
                nanosec=int((t_sec % 1.0) * 1e9))
            msg_out.points.append(pt)

        self._pub.publish(msg_out)
        self._update_contact(self._phase, gait)
        self._phase = (self._phase + 0.5) % 1.0

    # ── Contact detection ───────────────────────────────────────────────

    def _adjusted_phase(self, leg: str, phase: float, gait: dict) -> float:
        local = (phase - gait['phase_offsets'][leg]) % 1.0
        if self._early_contact[leg] and local < gait['swing_frac']:
            compression = CONTACT_MIN_SWING_FRAC / gait['swing_frac']
            local       = min(local * compression, gait['swing_frac'] - 0.001)
            return (gait['phase_offsets'][leg] + local) % 1.0
        return phase

    def _update_contact(self, phase: float, gait: dict):
        for leg in LEG_ORDER:
            local    = (phase - gait['phase_offsets'][leg]) % 1.0
            in_swing = local < gait['swing_frac']
            if in_swing:
                prog = local / gait['swing_frac']
                self._early_contact[leg] = (
                    prog > CONTACT_MIN_SWING_FRAC and
                    self._knee_effort[leg] > CONTACT_EFFORT_THRESHOLD)
            else:
                self._early_contact[leg] = False

    # ── Idle pose ───────────────────────────────────────────────────────

    def _publish_idle(self):
        turn_norm   = max(-1.0, min(1.0, self._angular_z / 1.0))
        steer_angle = turn_norm * MAX_STEER
        shoulder    = max(-0.08, min(0.08, turn_norm * 0.08))

        msg_out = JointTrajectory()
        msg_out.joint_names = JOINT_ORDER
        pt = JointTrajectoryPoint()
        pt.positions = [
            steer_angle,
            # fl
            shoulder, NEUTRAL_THIGH_FRONT, NEUTRAL_KNEE_FRONT, NEUTRAL_CANNON_FRONT,
            # fr
            shoulder, NEUTRAL_THIGH_FRONT, NEUTRAL_KNEE_FRONT, NEUTRAL_CANNON_FRONT,
            # rl
            shoulder, NEUTRAL_THIGH_REAR,  NEUTRAL_KNEE_REAR,  NEUTRAL_CANNON_REAR,
            # rr
            shoulder, NEUTRAL_THIGH_REAR,  NEUTRAL_KNEE_REAR,  NEUTRAL_CANNON_REAR,
        ]
        pt.velocities      = [0.0] * N_JOINTS
        pt.time_from_start = Duration(sec=0, nanosec=200_000_000)
        msg_out.points.append(pt)
        self._pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = BlendspaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
