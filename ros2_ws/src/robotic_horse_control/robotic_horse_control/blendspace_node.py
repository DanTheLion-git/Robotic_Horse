"""
blendspace_node.py  —  Elk-accurate velocity-driven gait controller (v5).

Red deer (Cervus elaphus) large prime male — accurate 1:1 dimensions:
  Withers (back): ~1.55 m   Belly height: ~1.00 m
  Thigh (L1)    : 0.38 m    Lower leg (L2): 0.36 m
  Cannon (L3)   : 0.25 m    Hoof radius:    0.05 m
  Knee bend      : 56 deg at neutral — natural deer standing pose

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
from sensor_msgs.msg import JointState, Imu
from builtin_interfaces.msg import Duration


# ── Robot geometry (must match URDF exactly) ─────────────────────────────────
# Large male red deer — 1:1 scale, withers ~1.55m
L1 = 0.38           # thigh length        [m]  (was 0.60, reduced for proper deer ratio)
L2 = 0.36           # shank length        [m]  (was 0.60)
L3 = 0.25           # cannon bone length  [m]  (was 0.35)
CANNON_LEAN = 0.15  # cannon forward lean [rad] (~8.6 deg)
FOOT_R = 0.05       # hoof radius         [m]  (was 0.06)

# Hip joint (thigh_joint) height from ground at nominal settled pose.
# body_center = 1.27m, hip_Z_from_body = -0.22m, bracket = -0.10m
# → thigh_joint = 1.27 - 0.22 - 0.10 = 0.95m
BODY_HEIGHT  = 0.95
ANKLE_HEIGHT = BODY_HEIGHT - L3 * math.cos(CANNON_LEAN) - FOOT_R  # 0.6528 m

# Hip positions in base_link (x=fwd, y=left, z=up)
# Body: 1.10m L × 0.38m W × 0.55m H; hip joints near belly at z=-0.22, x=±0.43, y=±0.19
LEG_POS = {
    'fl': ( 0.43,  0.19),
    'fr': ( 0.43, -0.19),
    'rl': (-0.43,  0.19),
    'rr': (-0.43, -0.19),
}
HALF_BODY_LENGTH = 0.43

FRONT_LEGS = ('fl', 'fr')
REAR_LEGS  = ('rl', 'rr')

# ── Neutral poses (computed from IK at ankle straight down) ──────────────────
# ANKLE_HEIGHT=0.6528, L1=0.38, L2=0.36: knee=-0.981rad(56deg), thigh=+/-0.476rad
NEUTRAL_THIGH_FRONT  = +0.476
NEUTRAL_KNEE_FRONT   = -0.981
NEUTRAL_CANNON_FRONT = +0.655   # CANNON_LEAN - (0.476 + -0.981) = 0.15 + 0.505 = 0.655

NEUTRAL_THIGH_REAR   = -0.476
NEUTRAL_KNEE_REAR    = +0.981
NEUTRAL_CANNON_REAR  = -0.355   # CANNON_LEAN - (-0.476 + 0.981) = 0.15 - 0.505 = -0.355

# ── Elk gait parameters ───────────────────────────────────────────────────────
WALK = dict(
    phase_offsets={'rl': 0.0, 'fl': 0.25, 'rr': 0.50, 'fr': 0.75},
    swing_frac=0.22,
    step_height=0.20,   # base foot clearance — pitch compensation adds on top
    min_stride=0.25,    # m — large step even at slow pace (50 cm full stride)
    period=1.6,
    name='WALK',
)

TROT = dict(
    phase_offsets={'fl': 0.0, 'fr': 0.5, 'rl': 0.5, 'rr': 0.0},
    swing_frac=0.32,
    step_height=0.35,
    min_stride=0.35,
    period=0.90,
    name='TROT',
)

GALLOP = dict(
    phase_offsets={'fl': 0.0, 'rl': 0.15, 'fr': 0.50, 'rr': 0.65},
    swing_frac=0.42,
    step_height=0.55,
    min_stride=0.50,
    period=0.50,
    name='GALLOP',
)

GAIT_SEQUENCE = [WALK, TROT, GALLOP]

# ── Gait auto-selection speed thresholds ─────────────────────────────────────
WALK_MAX_SPEED   = 1.5   # m/s — below this: WALK
TROT_MAX_SPEED   = 3.5   # m/s — below this: TROT, above: GALLOP

# ── Control limits ────────────────────────────────────────────────────────────
MAX_SPEED    = 6.0    # m/s  hard cap
MAX_SHOULDER = 0.20   # rad  max hip Z-yaw from IK
MAX_STEP     = 0.55   # m    hard cap on stride half-length

# Minimum stride half-length per gait: steps stay large even at slow pace.
# Speed controls cadence (how often steps happen), not step size.
WALK_MIN_STRIDE   = 0.25  # m   50 cm full stride minimum at walk
TROT_MIN_STRIDE   = 0.35  # m   70 cm full stride minimum at trot
GALLOP_MIN_STRIDE = 0.50  # m  100 cm full stride minimum at gallop

# Pitch-adaptive step height: lx * sin(pitch) * this scale added to each leg.
# Front legs (lx>0): lean forward → step HIGHER. Rear (lx<0): lean forward → lower.
PITCH_STEP_SCALE = 1.2

# Balance correction: lean forward → shift all foot targets backward by this gain.
# Helps CoM stay over support polygon during gait.
BALANCE_FOOT_GAIN = 0.25   # m shift per rad of lean

# Minimum step height so rear legs never drop to zero clearance.
MIN_STEP_HEIGHT = 0.10    # m

CONTACT_EFFORT_THRESHOLD = 45.0
CONTACT_MIN_SWING_FRAC   = 0.45

# ── Joint order (16 joints — no cart steer) ──────────────────────────────────
JOINT_ORDER = [
    'fl_hip_joint', 'fl_thigh_joint', 'fl_knee_joint', 'fl_cannon_joint',
    'fr_hip_joint', 'fr_thigh_joint', 'fr_knee_joint', 'fr_cannon_joint',
    'rl_hip_joint', 'rl_thigh_joint', 'rl_knee_joint', 'rl_cannon_joint',
    'rr_hip_joint', 'rr_thigh_joint', 'rr_knee_joint', 'rr_cannon_joint',
]
N_JOINTS  = len(JOINT_ORDER)   # 16
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


# ── Elk swing profile  (3-phase: Lift → Carry → Plant) ───────────────────────
#
# Phase 0.00–0.30  LIFT   : foot rises from liftoff position
# Phase 0.30–0.70  CARRY  : foot at peak height, sweeps forward
# Phase 0.70–1.00  PLANT  : foot descends to target touchdown position
#
# This gives a clear IK-target arc so the foot definitively clears the ground
# and lands precisely at the Raibert target — like a proper step trajectory.
#
# step_height is the WORLD-FRAME clearance guarantee: the pitch compensation
# argument adds extra lift so the foot ALWAYS clears the ground even when the
# body pitches forward.

def _elk_swing(p: float, fx_mean: float, fy_mean: float,
               ankle_height: float, step_height: float):
    """
    3-phase swing arc.
      p         : swing progress [0, 1]
      fx_mean   : Raibert half-stride in x (foot goes from -fx_mean to +fx_mean)
      fy_mean   : Raibert half-stride in y
      ankle_height : neutral foot depth below hip [m]
      step_height  : guaranteed world-frame ground clearance [m] (pitch-corrected)
    """
    LIFT_END  = 0.30
    CARRY_END = 0.70

    if p < LIFT_END:
        # LIFT: foot rises from liftoff (-fx_mean) straight up
        t_lift = p / LIFT_END            # 0→1
        lift   = step_height * math.sin(math.pi * 0.5 * t_lift)  # 0 → step_height
        t_fwd  = t_lift * 0.15           # subtle forward creep during lift (15% of stride)
        fx     = -fx_mean + 2.0 * fx_mean * t_fwd
        fy     = -fy_mean + 2.0 * fy_mean * t_fwd
    elif p < CARRY_END:
        # CARRY: foot at peak height, sweeps forward over the ground
        t_carry = (p - LIFT_END) / (CARRY_END - LIFT_END)   # 0→1
        lift    = step_height          # constant full height during carry
        t_fwd   = 0.15 + t_carry * 0.70   # 15% → 85% of stride
        fx      = -fx_mean + 2.0 * fx_mean * t_fwd
        fy      = -fy_mean + 2.0 * fy_mean * t_fwd
    else:
        # PLANT: foot descends from peak to touchdown (+fx_mean)
        t_plant = (p - CARRY_END) / (1.0 - CARRY_END)       # 0→1
        lift    = step_height * math.cos(math.pi * 0.5 * t_plant)  # step_height → 0
        t_fwd   = 0.85 + t_plant * 0.15   # 85% → 100% of stride
        fx      = -fx_mean + 2.0 * fx_mean * t_fwd
        fy      = -fy_mean + 2.0 * fy_mean * t_fwd

    fz = -(ankle_height - lift)
    return fx, fy, fz


def _pitch_compensated_step_height(leg: str, shaft_pitch: float,
                                   base_step_height: float) -> float:
    """
    Bidirectional pitch-adaptive step height.
      - Body pitches FORWARD (positive pitch): front legs step HIGHER, rear LOWER.
      - Body pitches BACKWARD (negative pitch): rear legs step HIGHER, front LOWER.
    Uses: extra = lx * sin(pitch) * PITCH_STEP_SCALE
      Front (lx=+0.55): forward lean → positive extra → more lift
      Rear  (lx=-0.55): forward lean → negative extra → less lift
    Clamped to MIN_STEP_HEIGHT so rear legs always retain some clearance.
    """
    lx    = LEG_POS[leg][0]
    extra = lx * math.sin(shaft_pitch) * PITCH_STEP_SCALE
    return max(MIN_STEP_HEIGHT, base_step_height + extra)


# ── Arc-following foot target (Raibert) ──────────────────────────────────────

def _foot_target_3d(leg: str, phase: float, linear_v: float, angular_v: float,
                    ankle_height: float, gait: dict, step_height: float,
                    balance_offset: float = 0.0):
    """
    Raibert arc-following foot placement with 3-phase swing arc.
      step_height    : pitch-adapted step height for this leg.
      balance_offset : extra fx shift for body lean correction (negative = push back).
    Minimum stride enforced: steps stay large at slow pace; cadence slows instead.
    """
    lx, ly   = LEG_POS[leg]
    T_stance = gait['period'] * (1.0 - gait['swing_frac'])

    # Raibert: negative so positive linear_v = body moves away from cart
    fx_speed = (linear_v - angular_v * ly) * T_stance / 2.0
    fy_speed = (angular_v * lx)            * T_stance / 2.0

    # Enforce minimum stride: slow walking = same size steps, just slower cadence
    if abs(linear_v) > 0.05:
        min_stride = gait.get('min_stride', 0.0)
        if abs(fx_speed) < min_stride:
            fx_speed = math.copysign(min_stride, fx_speed)

    fx_mean = -fx_speed + balance_offset
    fy_mean =  fy_speed
    fx_mean  = max(-MAX_STEP,       min(MAX_STEP,       fx_mean))
    fy_mean  = max(-MAX_STEP * 0.5, min(MAX_STEP * 0.5, fy_mean))

    local    = (phase - gait['phase_offsets'][leg]) % 1.0

    if local < gait['swing_frac']:
        p = local / gait['swing_frac']
        return _elk_swing(p, fx_mean, fy_mean, ankle_height, step_height)
    else:
        p  = (local - gait['swing_frac']) / (1.0 - gait['swing_frac'])
        fx = fx_mean * (1.0 - 2.0 * p)
        fy = fy_mean * (1.0 - 2.0 * p)
        fz = -(ankle_height + 0.02)   # press 2cm into ground for firm contact
        return fx, fy, fz


# ── Helpers ───────────────────────────────────────────────────────────────────

def _quaternion_to_pitch_roll(q):
    """Extract (pitch, roll) from ROS2 quaternion (x,y,z,w).
    Pitch = rotation about Y (positive = nose up in ROS ENU convention).
    Roll  = rotation about X (positive = roll left).
    """
    x, y, z, w = q.x, q.y, q.z, q.w
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    return pitch, roll


# ── Main node ─────────────────────────────────────────────────────────────────

class BlendspaceNode(Node):

    def __init__(self):
        super().__init__('blendspace_node')

        self._pub = self.create_publisher(
            JointTrajectory, '/leg_controller/joint_trajectory', 10)
        self.create_subscription(Twist,      '/cmd_vel',      self._cmd_vel_cb,      10)
        self.create_subscription(JointState, '/joint_states', self._joint_states_cb, 10)
        self.create_subscription(Imu,        '/imu/data',     self._imu_cb,          10)

        self._linear_x  = 0.0
        self._angular_z = 0.0

        self._gait_idx  = 0
        self._gait      = WALK.copy()
        self._phase     = 0.0

        self._body_pitch    = 0.0   # rad — from IMU (positive = nose up)
        self._body_roll     = 0.0   # rad — from IMU (positive = roll left)
        self._early_contact = {l: False for l in LEG_ORDER}
        self._knee_effort   = {l: 0.0   for l in LEG_ORDER}

        self._gait_timer = self.create_timer(
            self._gait['period'] / 2.0, self._tick)

        self.get_logger().info('=' * 60)
        self.get_logger().info('  Elk Blendspace v5  —  No cart, IMU self-balance')
        self.get_logger().info('  Bidirectional pitch-adaptive step heights')
        self.get_logger().info('  Minimum stride enforced: large steps at slow pace')
        self.get_logger().info('  Speed: 0-1.5=WALK  1.5-3.5=TROT  >3.5=GALLOP')
        self.get_logger().info('=' * 60)

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        self._linear_x  = max(0.0, min(MAX_SPEED, msg.linear.x))
        self._angular_z = msg.angular.z
        self._auto_select_gait()

    def _imu_cb(self, msg: Imu):
        self._body_pitch, self._body_roll = _quaternion_to_pitch_roll(msg.orientation)

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
        if len(msg.effort) == len(msg.name):
            name_to_idx = {n: i for i, n in enumerate(msg.name)}
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

        gait  = self._gait
        pitch = self._body_pitch   # IMU: positive = nose up
        roll  = self._body_roll    # IMU: positive = roll left

        # Ankle height adjustment: pitched body → front legs reach deeper, rear less
        # Using pitch directly (negative pitch = nose down = forward lean in most conventions)
        pitch_adj = math.tan(-pitch) * HALF_BODY_LENGTH
        pitch_adj = max(-0.20, min(0.20, pitch_adj))
        ankle_heights = {
            'fl': ANKLE_HEIGHT + pitch_adj,
            'fr': ANKLE_HEIGHT + pitch_adj,
            'rl': ANKLE_HEIGHT - pitch_adj,
            'rr': ANKLE_HEIGHT - pitch_adj,
        }

        # Bidirectional pitch-adaptive step heights
        step_heights = {
            leg: _pitch_compensated_step_height(leg, -pitch, gait['step_height'])
            for leg in LEG_ORDER
        }

        # Balance correction: lean forward (negative pitch/nose down) → shift feet backward
        balance_offset = pitch * BALANCE_FOOT_GAIN

        n_pts = 50
        dt    = gait['period'] / n_pts

        msg_out = JointTrajectory()
        msg_out.joint_names = JOINT_ORDER

        for i in range(n_pts):
            phase     = (self._phase + i / n_pts) % 1.0
            positions = []

            for leg in LEG_ORDER:
                ank_h  = ankle_heights[leg]
                eff_ph = self._adjusted_phase(leg, phase, gait)

                fx, fy, fz = _foot_target_3d(
                    leg, eff_ph, lin, ang, ank_h, gait,
                    step_heights[leg], balance_offset)

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

    # ── Idle pose with IMU balance correction ───────────────────────────

    def _publish_idle(self):
        # Turning: differential hip splay (no cart steer)
        shoulder = max(-0.12, min(0.12, self._angular_z * 0.06))

        # IMU balance: lean forward (nose down = negative pitch in ROS ENU) →
        # rotate all thighs backward to shift support polygon forward under CoM.
        # Gain of 0.3 rad/rad gives ~17 deg thigh correction per 1 rad lean.
        pitch_corr = max(-0.20, min(0.20, -self._body_pitch * 0.30))

        # Roll correction: lean left (positive roll) → left legs splay outward more
        roll_corr  = max(-0.10, min(0.10,  self._body_roll  * 0.20))

        fl_hip = shoulder + roll_corr
        fr_hip = shoulder - roll_corr
        rl_hip = shoulder + roll_corr
        rr_hip = shoulder - roll_corr

        fl_th = NEUTRAL_THIGH_FRONT + pitch_corr
        fr_th = NEUTRAL_THIGH_FRONT + pitch_corr
        rl_th = NEUTRAL_THIGH_REAR  + pitch_corr
        rr_th = NEUTRAL_THIGH_REAR  + pitch_corr

        # Recalculate cannon for adjusted thigh (pantograph)
        fl_ca = CANNON_LEAN - (fl_th + NEUTRAL_KNEE_FRONT)
        fr_ca = CANNON_LEAN - (fr_th + NEUTRAL_KNEE_FRONT)
        rl_ca = CANNON_LEAN - (rl_th + NEUTRAL_KNEE_REAR)
        rr_ca = CANNON_LEAN - (rr_th + NEUTRAL_KNEE_REAR)

        msg_out = JointTrajectory()
        msg_out.joint_names = JOINT_ORDER
        pt = JointTrajectoryPoint()
        pt.positions = [
            fl_hip, fl_th, NEUTRAL_KNEE_FRONT, fl_ca,
            fr_hip, fr_th, NEUTRAL_KNEE_FRONT, fr_ca,
            rl_hip, rl_th, NEUTRAL_KNEE_REAR,  rl_ca,
            rr_hip, rr_th, NEUTRAL_KNEE_REAR,  rr_ca,
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
