"""
blendspace_node.py  —  Elk-accurate velocity-driven gait controller (v4).

Red deer (Cervus elaphus) male prime dimensions:
  Hip height    : 1.45 m   (body height, back ~1.60 m)
  Thigh / Shank : 0.60 m each
  Cannon bone   : 0.35 m
  Hoof radius   : 0.06 m

Leg configuration:
  FRONT (FL/FR): elbow-DOWN — carpal bends FORWARD (away from cart)
  REAR  (RL/RR): elbow-UP   — hock bends BACKWARD  (toward cart)
  Cannon bone pantograph: cannon = CANNON_LEAN - (thigh + knee)

Gait auto-selection by speed:
  0 – 1.5 m/s  → WALK   (4-beat lateral, elk deliberate step)
  1.5 – 3.5 m/s → TROT   (diagonal suspension trot)
  > 3.5 m/s    → GALLOP (rotary gallop, hind legs reach through)

No backward movement — speed is clamped to [0, MAX_SPEED].
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
L1 = 0.60           # thigh length        [m]
L2 = 0.60           # shank length        [m]
L3 = 0.35           # cannon bone length  [m]
CANNON_LEAN = 0.15  # cannon forward lean [rad] (~8.6 deg)
FOOT_R = 0.06       # hoof radius         [m]

BODY_HEIGHT  = 1.45
ANKLE_HEIGHT = BODY_HEIGHT - L3 * math.cos(CANNON_LEAN) - FOOT_R  # ~1.044 m

# Hip positions in base_link (x=fwd-from-cart, y=left, z=up)
LEG_POS = {
    'fl': ( 0.55,  0.28),
    'fr': ( 0.55, -0.28),
    'rl': (-0.55,  0.28),
    'rr': (-0.55, -0.28),
}
HALF_BODY_LENGTH = 0.55

FRONT_LEGS = ('fl', 'fr')
REAR_LEGS  = ('rl', 'rr')

# ── Neutral poses (verified by FK: ankle at (0,0,-ANKLE_HEIGHT)) ──────────────
NEUTRAL_THIGH_FRONT  = +0.516
NEUTRAL_KNEE_FRONT   = -1.031
NEUTRAL_CANNON_FRONT = +0.666   # CANNON_LEAN - (0.516 + -1.031)

NEUTRAL_THIGH_REAR   = -0.516
NEUTRAL_KNEE_REAR    = +1.031
NEUTRAL_CANNON_REAR  = -0.366   # CANNON_LEAN - (-0.516 + 1.031)

# ── Elk gait parameters ───────────────────────────────────────────────────────
WALK = dict(
    phase_offsets={'rl': 0.0, 'fl': 0.25, 'rr': 0.50, 'fr': 0.75},
    swing_frac=0.35,
    step_height=0.20,   # scaled for 1.45m elk: ~20cm lift at walk
    period=1.6,
    name='WALK',
)

TROT = dict(
    phase_offsets={'fl': 0.0, 'fr': 0.5, 'rl': 0.5, 'rr': 0.0},
    swing_frac=0.50,
    step_height=0.30,   # elk trot: energetic, ~30cm lift
    period=0.90,
    name='TROT',
)

GALLOP = dict(
    phase_offsets={'fl': 0.0, 'rl': 0.15, 'fr': 0.50, 'rr': 0.65},
    swing_frac=0.55,
    step_height=0.40,   # elk gallop: large bounding strides
    period=0.50,
    name='GALLOP',
)

GAIT_SEQUENCE = [WALK, TROT, GALLOP]

# ── Gait auto-selection speed thresholds ─────────────────────────────────────
WALK_MAX_SPEED   = 1.5   # m/s — below this: WALK
TROT_MAX_SPEED   = 3.5   # m/s — below this: TROT, above: GALLOP

# ── Control limits ────────────────────────────────────────────────────────────
MAX_SPEED    = 6.0    # m/s  hard cap
MAX_STEER    = 0.45   # rad  max cart steer angle
MAX_SHOULDER = 0.20   # rad  max hip Z-yaw from IK
MAX_STEP     = 0.55   # m    hard cap on stride half-length (deer stride is large)

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


# ── IK — front legs (elbow-DOWN) ──────────────────────────────────────────────

def _ik_3d_front(fx: float, fy: float, fz: float):
    reach = math.sqrt(fx**2 + fz**2)
    hip_z = math.atan2(fy, reach) if reach > 0.001 else 0.0
    hip_z = max(-MAX_SHOULDER, min(MAX_SHOULDER, hip_z))

    fx_2d = fx * math.cos(hip_z) + fy * math.sin(hip_z)
    fz_2d = fz

    r     = math.sqrt(fx_2d**2 + fz_2d**2)
    r     = max(abs(L1 - L2) + 0.001, min(r, L1 + L2 - 0.001))
    cos_k = (r**2 - L1**2 - L2**2) / (2.0 * L1 * L2)
    cos_k = max(-1.0, min(1.0, cos_k))

    knee   = -math.acos(cos_k)
    alpha  = math.atan2(fx_2d, -fz_2d)
    beta   = math.asin(max(-1.0, min(1.0, L2 * math.sin(abs(knee)) / r)))
    thigh  = alpha + beta
    cannon = CANNON_LEAN - (thigh + knee)
    cannon = max(-2.0, min(2.0, cannon))
    return hip_z, thigh, knee, cannon


# ── IK — rear legs (elbow-UP) ─────────────────────────────────────────────────

def _ik_3d_rear(fx: float, fy: float, fz: float):
    reach = math.sqrt(fx**2 + fz**2)
    hip_z = math.atan2(fy, reach) if reach > 0.001 else 0.0
    hip_z = max(-MAX_SHOULDER, min(MAX_SHOULDER, hip_z))

    fx_2d = fx * math.cos(hip_z) + fy * math.sin(hip_z)
    fz_2d = fz

    r     = math.sqrt(fx_2d**2 + fz_2d**2)
    r     = max(abs(L1 - L2) + 0.001, min(r, L1 + L2 - 0.001))
    cos_k = (r**2 - L1**2 - L2**2) / (2.0 * L1 * L2)
    cos_k = max(-1.0, min(1.0, cos_k))

    knee   = +math.acos(cos_k)
    alpha  = math.atan2(fx_2d, -fz_2d)
    beta   = math.asin(max(-1.0, min(1.0, L2 * math.sin(knee) / r)))
    thigh  = alpha - beta
    cannon = CANNON_LEAN - (thigh + knee)
    cannon = max(-2.0, min(2.0, cannon))
    return hip_z, thigh, knee, cannon


# ── Elk swing profile ─────────────────────────────────────────────────────────

def _elk_swing(p: float, fx_mean: float, fy_mean: float,
               ankle_height: float, step_height: float):
    lift = step_height * math.sin(math.pi * (p ** 0.7))
    t    = p ** 1.3
    fx   = -fx_mean + 2.0 * fx_mean * t
    fy   = -fy_mean + 2.0 * fy_mean * t
    fz   = -(ankle_height - lift)
    return fx, fy, fz


# ── Arc-following foot target (Raibert) ──────────────────────────────────────

def _foot_target_3d(leg: str, phase: float, linear_v: float, angular_v: float,
                    ankle_height: float, gait: dict):
    """
    Raibert arc-following foot placement.
    Horse base_link +x = away from cart (nose direction).
    Positive linear_v = step forward (away from cart) = correct.
    """
    lx, ly   = LEG_POS[leg]
    T_stance = gait['period'] * (1.0 - gait['swing_frac'])

    # Standard Raibert: positive linear_v = foot placed forward = body moves forward
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
        self.create_subscription(JointState, '/joint_states', self._joint_states_cb, 10)

        self._linear_x  = 0.0
        self._angular_z = 0.0

        self._gait_idx  = 0          # start in WALK
        self._gait      = WALK.copy()
        self._phase     = 0.0

        self._shaft_pitch   = 0.0
        self._early_contact = {l: False for l in LEG_ORDER}
        self._knee_effort   = {l: 0.0   for l in LEG_ORDER}

        self._gait_timer = self.create_timer(
            self._gait['period'] / 2.0, self._tick)

        self.get_logger().info('=' * 60)
        self.get_logger().info('  Elk Blendspace v4  —  Red Deer 160cm, deer legs')
        self.get_logger().info('  Speed thresholds: 0-1.5=WALK, 1.5-3.5=TROT, >3.5=GALLOP')
        self.get_logger().info('  No backward movement. W ramps up, S ramps down.')
        self.get_logger().info('=' * 60)

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        # Clamp to [0, MAX_SPEED]: no backward movement
        self._linear_x  = max(0.0, min(MAX_SPEED, msg.linear.x))
        self._angular_z = msg.angular.z
        self._auto_select_gait()

    def _auto_select_gait(self):
        v = self._linear_x
        if v < WALK_MAX_SPEED:
            new_idx = 0
        elif v < TROT_MAX_SPEED:
            new_idx = 1
        else:
            new_idx = 2

        if new_idx != self._gait_idx:
            self._gait_idx = new_idx
            self._gait     = GAIT_SEQUENCE[new_idx].copy()
            self._reschedule_timer()
            self.get_logger().info(
                f"Auto gait -> {self._gait['name']}  (v={v:.2f} m/s)")

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

        # Body leveling
        pitch_adj = math.tan(self._shaft_pitch) * HALF_BODY_LENGTH
        pitch_adj = max(-0.15, min(0.15, pitch_adj))
        ankle_heights = {
            'fl': ANKLE_HEIGHT + pitch_adj,
            'fr': ANKLE_HEIGHT + pitch_adj,
            'rl': ANKLE_HEIGHT - pitch_adj,
            'rr': ANKLE_HEIGHT - pitch_adj,
        }

        # Active cart steering
        turn_norm   = max(-1.0, min(1.0, ang / 1.5))
        steer_angle = turn_norm * MAX_STEER

        n_pts = 50
        dt    = gait['period'] / n_pts

        msg_out = JointTrajectory()
        msg_out.joint_names = JOINT_ORDER

        for i in range(n_pts):
            phase     = (self._phase + i / n_pts) % 1.0
            positions = [steer_angle]

            for leg in LEG_ORDER:
                ank_h  = ankle_heights[leg]
                eff_ph = self._adjusted_phase(leg, phase, gait)

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
        turn_norm   = max(-1.0, min(1.0, self._angular_z / 1.5))
        steer_angle = turn_norm * MAX_STEER
        shoulder    = max(-0.08, min(0.08, turn_norm * 0.08))

        msg_out = JointTrajectory()
        msg_out.joint_names = JOINT_ORDER
        pt = JointTrajectoryPoint()
        pt.positions = [
            steer_angle,
            shoulder, NEUTRAL_THIGH_FRONT, NEUTRAL_KNEE_FRONT, NEUTRAL_CANNON_FRONT,
            shoulder, NEUTRAL_THIGH_FRONT, NEUTRAL_KNEE_FRONT, NEUTRAL_CANNON_FRONT,
            shoulder, NEUTRAL_THIGH_REAR,  NEUTRAL_KNEE_REAR,  NEUTRAL_CANNON_REAR,
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
