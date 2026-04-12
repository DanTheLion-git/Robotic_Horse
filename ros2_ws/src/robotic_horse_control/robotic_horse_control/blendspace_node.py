"""
blendspace_node.py  —  Elk-accurate velocity-driven gait controller.

Elk (Cervus canadensis) gait characteristics implemented:
  WALK  — 4-beat lateral sequence (RL→FL→RR→FR), period 1.6 s
           Duty factor 65%: 3 feet always on ground, very stable
  TROT  — Diagonal pairs with suspension phase, period 0.90 s
           Elk trot has a brief airborne moment; duty factor ~50%
  GALLOP— Rotary gallop (hind legs reach ahead of fore legs), period 0.50 s

Step profile:
  All gaits use an elk-inspired swing arc:
    - Height peaks at 40% of swing (sin^0.7 profile → early, sustained lift)
    - Forward extension is slightly delayed (p^1.3) — leg lifts THEN swings
  This mimics the characteristic high-stepping elk walk and elastic trot.

Physics improvements (anti-bounce):
  - Robot body mass increased to 28 kg (62 kg total) — see URDF
  - Leg joint damping increased — see URDF
  - PID gains scaled proportionally — see ros2_control.yaml

5 core improvements (from previous session) retained:
  1. 3D IK          — hip Z-yaw for arc-correct lateral foot placement
  2. Raibert placement — per-leg stride = arc velocity × T_stance / 2
  3. Active steering — cart_to_shaft_steer commanded joint
  4. Body leveling   — shaft_to_horse pitch adjusts front/rear height
  5. Contact detect  — knee effort spike → early stance

Topics:
  /cmd_vel     geometry_msgs/Twist  — linear.x=speed, angular.z=turn
  /cmd_gait    std_msgs/String      — "walk" | "trot" | "gallop"
  /joint_states sensor_msgs/JointState

Keys (via teleop_key): W/S=fwd/bwd  A/D=turn  Q=cycle gait  SPACE=stop
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
L1 = 0.45          # thigh length  [m]
L2 = 0.50          # shank length  [m]

# Hip pivot positions relative to base_link (x=fwd, y=left, z=up)
LEG_POS = {
    'fl': ( 0.35,  0.18),
    'fr': ( 0.35, -0.18),
    'rl': (-0.35,  0.18),
    'rr': (-0.35, -0.18),
}
HALF_BODY_LENGTH = 0.35

# ── Elk gait parameters ───────────────────────────────────────────────────────
# Neutral stance pose (Spot-like elbow-up, verified by IK)
BODY_HEIGHT   = 0.90
NEUTRAL_THIGH = -0.344
NEUTRAL_KNEE  =  0.651

# ─── WALK — 4-beat lateral sequence ────────────────────────────────────────
# Footfall order:  RL(0.0) → FL(0.25) → RR(0.50) → FR(0.75)
# At any instant: 3 feet on ground (3-point support), very stable.
# Elk walk: deliberate, high knee lift, long stride, head bobs gently.
WALK = dict(
    phase_offsets={'rl': 0.0, 'fl': 0.25, 'rr': 0.50, 'fr': 0.75},
    swing_frac=0.35,    # 65% stance  → 3 feet always down
    step_height=0.12,   # moderate lift at walk
    period=1.6,         # slow cadence; increase SPEED in teleop for realistic pace
    name='WALK',
)

# ─── TROT — diagonal pairs with brief suspension ────────────────────────────
# FL+RR move together, FR+RL move together.
# Elk trot: energetic, elastic, higher lift than walk, brief airborne phase.
TROT = dict(
    phase_offsets={'fl': 0.0, 'fr': 0.5, 'rl': 0.5, 'rr': 0.0},
    swing_frac=0.50,    # equal swing/stance → suspension phase emerges
    step_height=0.17,   # noticeable lift — elk trot is showy
    period=0.90,        # faster cadence
    name='TROT',
)

# ─── GALLOP — rotary gallop (elk/deer signature) ────────────────────────────
# Hind legs reach far ahead of fore legs between bounds.
# Left-lead rotary sequence: FL → RL → FR → RR
# The hindquarters "reach through" for powerful propulsion.
GALLOP = dict(
    phase_offsets={'fl': 0.0, 'rl': 0.15, 'fr': 0.50, 'rr': 0.65},
    swing_frac=0.55,    # more time airborne per leg in gallop
    step_height=0.22,   # large bounding strides
    period=0.50,        # fast cadence
    name='GALLOP',
)

GAIT_SEQUENCE = [WALK, TROT, GALLOP]   # Q cycles through this list

# ── Control limits ────────────────────────────────────────────────────────────
MAX_STEER    = 0.45   # max cart steer angle [rad] (~26 deg)
MAX_SHOULDER = 0.20   # max hip yaw from 3D IK [rad]
MAX_STEP     = 0.30   # hard cap on stride half-width [m]

CONTACT_EFFORT_THRESHOLD = 45.0
CONTACT_MIN_SWING_FRAC   = 0.45

# ── Joint order (13 joints) ───────────────────────────────────────────────────
JOINT_ORDER = [
    'cart_to_shaft_steer',
    'fl_hip_joint', 'fl_thigh_joint', 'fl_knee_joint',
    'fr_hip_joint', 'fr_thigh_joint', 'fr_knee_joint',
    'rl_hip_joint', 'rl_thigh_joint', 'rl_knee_joint',
    'rr_hip_joint', 'rr_thigh_joint', 'rr_knee_joint',
]
N_JOINTS  = len(JOINT_ORDER)
LEG_ORDER = ('fl', 'fr', 'rl', 'rr')


# ── 3D IK ─────────────────────────────────────────────────────────────────────

def _ik_3d(fx: float, fy: float, fz: float):
    """
    Elbow-up 3D IK: hip-Z yaw to follow lateral foot offset,
    then 2D elbow-up IK in the rotated plane.
    Returns (hip_z, thigh_y, knee_y).
    """
    reach = math.sqrt(fx**2 + fz**2)
    hip_z = math.atan2(fy, reach) if reach > 0.001 else 0.0
    hip_z = max(-MAX_SHOULDER, min(MAX_SHOULDER, hip_z))

    cos_h  = math.cos(hip_z)
    sin_h  = math.sin(hip_z)
    fx_2d  = fx * cos_h + fy * sin_h
    fz_2d  = fz

    r      = math.sqrt(fx_2d**2 + fz_2d**2)
    r      = max(abs(L1 - L2) + 0.001, min(r, L1 + L2 - 0.001))
    cos_k  = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_k  = max(-1.0, min(1.0, cos_k))
    knee   = +math.acos(cos_k)
    alpha  = math.atan2(fx_2d, -fz_2d)
    beta   = math.asin(max(-1.0, min(1.0, L2 * math.sin(knee) / r)))
    thigh  = alpha - beta

    return hip_z, thigh, knee


# ── Elk-specific swing profile ────────────────────────────────────────────────

def _elk_swing(p: float, fx_mean: float, fy_mean: float,
               body_height: float, step_height: float):
    """
    Elk-inspired swing trajectory for swing-phase fraction p ∈ [0, 1).

    Characteristics:
      - Height peaks early (~40% into swing) using sin(π·p^0.7)
        The exponent <1 shifts the sine peak earlier, giving a broad plateau
        at full lift — the leg "dwells" high before reaching forward.
      - Forward extension is slightly delayed: p^1.3
        This means the leg lifts UP before it swings FORWARD — exactly what
        you see in slow-motion elk and horse video footage.

    Returns (fx, fy, fz) in hip frame.
    """
    # Elk-style height: broad, early peak
    lift = step_height * math.sin(math.pi * (p ** 0.7))

    # Elk-style forward swing: delayed (lift first, reach later)
    t = p ** 1.3
    fx = -fx_mean + 2.0 * fx_mean * t
    fy = -fy_mean + 2.0 * fy_mean * t
    fz = -(body_height - lift)

    return fx, fy, fz


# ── Arc-following foot target ─────────────────────────────────────────────────

def _foot_target_3d(leg: str, phase: float, linear_v: float, angular_v: float,
                    body_height: float, gait: dict):
    """
    Raibert-inspired arc-following foot target.

    For a leg at (lx, ly) from the body centre moving at (vx, ω):
        v_hip_x = vx - ω·ly    (inner legs travel shorter arc)
        v_hip_y = ω·lx         (lateral arc sweep)

    Step half-length = v_hip × T_stance / 2
    Produces differential stride automatically for any turn radius.

    Returns (fx, fy, fz) in hip frame.
    """
    lx, ly = LEG_POS[leg]
    T_stance = gait['period'] * (1.0 - gait['swing_frac'])

    fx_mean = (linear_v  - angular_v * ly) * T_stance / 2.0
    fy_mean = (angular_v * lx)             * T_stance / 2.0

    fx_mean = max(-MAX_STEP,       min(MAX_STEP,       fx_mean))
    fy_mean = max(-MAX_STEP * 0.5, min(MAX_STEP * 0.5, fy_mean))

    local = (phase - gait['phase_offsets'][leg]) % 1.0

    if local < gait['swing_frac']:
        p = local / gait['swing_frac']
        return _elk_swing(p, fx_mean, fy_mean, body_height, gait['step_height'])
    else:
        p  = (local - gait['swing_frac']) / (1.0 - gait['swing_frac'])
        fx = fx_mean  * (1.0 - 2.0 * p)
        fy = fy_mean  * (1.0 - 2.0 * p)
        fz = -body_height
        return fx, fy, fz


# ── Main node ─────────────────────────────────────────────────────────────────

class BlendspaceNode(Node):

    def __init__(self):
        super().__init__('blendspace_node')

        self._pub = self.create_publisher(
            JointTrajectory, '/leg_controller/joint_trajectory', 10)
        self.create_subscription(Twist,      '/cmd_vel',      self._cmd_vel_cb,  10)
        self.create_subscription(String,     '/cmd_gait',     self._gait_cb,     10)
        self.create_subscription(JointState, '/joint_states', self._joint_states_cb, 10)

        self._linear_x  = 0.0
        self._angular_z = 0.0

        self._gait_idx  = 1            # start in TROT
        self._gait      = TROT.copy()
        self._phase     = 0.0

        # Body leveling
        self._shaft_pitch = 0.0

        # Contact detection
        self._early_contact = {l: False for l in LEG_ORDER}
        self._knee_effort   = {l: 0.0   for l in LEG_ORDER}

        self._gait_timer = self.create_timer(
            self._gait['period'] / 2.0, self._tick)

        self.get_logger().info('=' * 58)
        self.get_logger().info('  Elk Blendspace — gaits: WALK / TROT / GALLOP')
        self.get_logger().info('  /cmd_vel: linear.x=fwd  angular.z=turn')
        self.get_logger().info('  /cmd_gait: "walk" | "trot" | "gallop"')
        self.get_logger().info('  Q key cycles gaits, SPACE stops')
        self.get_logger().info('=' * 58)

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        self._linear_x  = msg.linear.x
        self._angular_z = msg.angular.z

    def _gait_cb(self, msg: String):
        mode = msg.data.lower().strip()
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

        # ── Body leveling (improvement #4) ─────────────────────────────
        pitch_adj = math.tan(self._shaft_pitch) * HALF_BODY_LENGTH
        pitch_adj = max(-0.12, min(0.12, pitch_adj))
        body_heights = {
            'fl': BODY_HEIGHT + pitch_adj,
            'fr': BODY_HEIGHT + pitch_adj,
            'rl': BODY_HEIGHT - pitch_adj,
            'rr': BODY_HEIGHT - pitch_adj,
        }

        # ── Active steering (improvement #3) ───────────────────────────
        turn_norm   = max(-1.0, min(1.0, ang / 1.0))
        steer_angle = turn_norm * MAX_STEER

        # ── Build trajectory (50 points across one period half) ────────
        n_pts = 50
        dt    = gait['period'] / n_pts

        msg_out = JointTrajectory()
        msg_out.joint_names = JOINT_ORDER

        for i in range(n_pts):
            phase = (self._phase + i / n_pts) % 1.0
            positions = [steer_angle]

            for leg in LEG_ORDER:
                bh  = body_heights[leg]
                eff = self._adjusted_phase(leg, phase, gait)

                fx, fy, fz = _foot_target_3d(leg, eff, lin, ang, bh, gait)
                hip_z, thigh, knee = _ik_3d(fx, fy, fz)
                positions += [hip_z, thigh, knee]

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
            local = min(local * compression, gait['swing_frac'] - 0.001)
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

    # ── Idle ────────────────────────────────────────────────────────────

    def _publish_idle(self):
        turn_norm   = max(-1.0, min(1.0, self._angular_z / 1.0))
        steer_angle = turn_norm * MAX_STEER
        shoulder    = max(-0.08, min(0.08, turn_norm * 0.08))

        msg_out = JointTrajectory()
        msg_out.joint_names = JOINT_ORDER
        pt = JointTrajectoryPoint()
        pt.positions = [
            steer_angle,
            shoulder, NEUTRAL_THIGH, NEUTRAL_KNEE,
            shoulder, NEUTRAL_THIGH, NEUTRAL_KNEE,
            shoulder, NEUTRAL_THIGH, NEUTRAL_KNEE,
            shoulder, NEUTRAL_THIGH, NEUTRAL_KNEE,
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
