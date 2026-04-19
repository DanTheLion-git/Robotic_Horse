"""
blendspace_node.py  —  Highland Cow velocity-driven gait controller.

Bovine Leg Mechanics
────────────────────
  FRONT legs: passive cannon linkage — cannon = CANNON_LEAN - (thigh + knee)
    Keeps metacarpal near-vertical via parallelogram geometry.

  REAR legs: RECIPROCAL APPARATUS — cannon = RECIP_OFFSET - RECIP_RATIO × knee
    Models the peroneus tertius + SDF tendon coupling between stifle and hock.
    When stifle flexes, hock flexes proportionally (four-bar linkage).
    RECIP_RATIO ≈ 0.85 (slightly sub-unity, matching bovine anatomy).

QDD Actuation (replaced ballscrew model):
  All joints use quasi-direct-drive motors. Backdrivable, compliant, torque-sensing.
  Robot mass: ~107 kg (150 cm Highland Cow).

Weight distribution: 55% front / 45% rear (bovine standard).

Turning: Articulated spine (spine_yaw_joint) + differential stride.
  The spine_yaw_joint rotates the front body (thorax) relative to the rear body
  (haunches). Combined with inner/outer leg stride differential, this produces
  smooth, natural turns like a real quadruped.

Bovine Gaits (from biomechanics research):
  WALK: 4-beat lateral sequence (RL→FL→RR→FR)
        stride ~1.10m, period ~1.1s, stance 60%, swing 40%
        step freq ~1.0 Hz, speed ~1.0 m/s
  TROT: 2-beat diagonal (FL+RR, FR+RL)
        stride ~1.50m, period ~0.75s, stance 50%, swing 50%
        speed ~2.0 m/s

Gait discrete state machine (W=advance, S=decrease, SPACE=IDLE):
  IDLE → WALK → TROT
"""

import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Imu
from builtin_interfaces.msg import Duration


# ── Robot geometry (150 cm Highland Cow — must match URDF exactly) ────────────
# Front leg segments
L1_FRONT = 0.38        # humerus [m]
L2_FRONT = 0.34        # radius  [m]
L3_FRONT = 0.18        # cannon  [m]
FOOT_R_FRONT = 0.05    # hoof radius [m]

# Rear leg segments
L1_REAR = 0.50         # femur [m]
L2_REAR = 0.45         # tibia [m]
L3_REAR = 0.22         # cannon [m]
FOOT_R_REAR = 0.06     # hoof radius [m]

CANNON_LEAN = 0.08     # front leg: cannon forward lean [rad]

# Reciprocal apparatus coupling (rear legs only — bovine stifle-hock)
RECIP_RATIO  = 0.85    # coupling ratio (slightly sub-unity like real cattle)
RECIP_OFFSET = -0.05 + RECIP_RATIO * 0.505  # ≈ 0.379 rad

# Body centre height and per-leg ankle heights
BODY_HEIGHT  = 1.12    # body centre Z [m]
ANKLE_HEIGHT_FRONT = 0.70
ANKLE_HEIGHT_REAR  = 0.92

# Hip heights above ground (from URDF hip positions on body)
FRONT_HIP_HEIGHT = 0.93   # m
REAR_HIP_HEIGHT  = 1.20   # m

# Hip positions in base_link frame (x=fwd, y=left, z=up)
# FL/FR are on front_body_link but positioned at same coords relative to spine joint
LEG_POS = {
    'fl': ( 0.65,  0.44),
    'fr': ( 0.65, -0.44),
    'rl': (-0.65,  0.40),
    'rr': (-0.65, -0.40),
}
HALF_BODY_LENGTH = 0.65

FRONT_LEGS = ('fl', 'fr')
REAR_LEGS  = ('rl', 'rr')

# Neutral angles — standing pose for 150 cm Highland Cow
NEUTRAL_KNEE_FRONT   = -0.48
NEUTRAL_THIGH_FRONT  = +0.22
NEUTRAL_CANNON_FRONT = CANNON_LEAN - (NEUTRAL_THIGH_FRONT + NEUTRAL_KNEE_FRONT)

NEUTRAL_KNEE_REAR    = +0.51
NEUTRAL_THIGH_REAR   = -0.24
NEUTRAL_CANNON_REAR  = RECIP_OFFSET - RECIP_RATIO * NEUTRAL_KNEE_REAR

# ── Bovine gait parameters (from biomechanics research) ──────────────────────
# Highland cow stride length 1.0-1.3m at walk. Scaled to 150cm robot: ~0.90m
# Step frequency: 1.0-1.25 Hz (60-75 steps/min)
# Stance phase: ~60% of stride cycle (duty factor >0.5 at walk)
# Walk sequence: LH → LF → RH → RF (lateral sequence)
# Stride reduced from 1.10m to prevent front-leg buckling under dynamic load.

WALK = dict(
    phase_offsets={'rl': 0.0, 'fl': 0.25, 'rr': 0.50, 'fr': 0.75},
    swing_frac=0.35,             # 35% swing, 65% stance (longer stance = more stable)
    step_height_front=0.14,      # front legs lift higher (clearance + anti-stub)
    step_height_rear=0.10,       # rear legs lower arc (propulsive)
    stride_length=0.90,          # reduced stride for stability [m]
    min_stride=0.20,             # startup stride [m]
    period=1.10,                 # ~0.9 Hz stride frequency (bovine walk)
    name='WALK',
)

TROT = dict(
    phase_offsets={'fl': 0.0, 'fr': 0.5, 'rl': 0.5, 'rr': 0.0},
    swing_frac=0.45,             # 45% swing, 55% stance (more ground contact than 50/50)
    step_height_front=0.18,      # higher clearance at trot speed
    step_height_rear=0.14,
    stride_length=1.20,          # reduced from 1.50 for stability [m]
    min_stride=0.30,             # startup stride during trot [m]
    period=0.80,                 # ~1.25 Hz stride frequency
    name='TROT',
)

GAIT_SEQUENCE = [WALK, TROT]

# Fixed speeds per gait state (teleop sends these exact values)
WALK_SPEED     = 0.80   # m/s (slightly slower for stability)
TROT_SPEED     = 1.60   # m/s (reduced from 2.0 for front-leg safety)
IDLE_THRESHOLD = 0.05
TROT_THRESHOLD = 1.20

# ── Control limits ────────────────────────────────────────────────────────────
MAX_SHOULDER = 0.22   # rad  max hip Z-yaw
MAX_STEP     = 0.50   # m    hard cap on stride half-length

# Spine turning
SPINE_TURN_GAIN = 0.35   # rad spine yaw per 1.0 rad/s angular velocity
MAX_SPINE_YAW   = 0.28   # rad  slightly under joint limit for safety

# Speed ramp: strides grow from short→long over first few cycles after starting
SPEED_RAMP_RATE    = 0.10   # ramp increment per tick (0→1 in ~10 ticks ≈ 5 cycles)
SPEED_RAMP_INITIAL = 0.20   # starting speed fraction (20% of target speed)
SPEED_RAMP_TRANSITION = 0.35  # speed fraction at gait transitions

# Pitch-adaptive step height
PITCH_STEP_SCALE = 1.2

# Balance correction
BALANCE_FOOT_GAIN = 0.50   # m shift per rad of lean (increased for stronger correction)

# Minimum step height so rear legs never drop to zero clearance
MIN_STEP_HEIGHT = 0.05    # m (raised from 0.04)

CONTACT_EFFORT_THRESHOLD = 45.0
CONTACT_MIN_SWING_FRAC   = 0.45

# Stance stiffening — virtual spring model
# During stance, the foot pushes deeper into the ground at mid-stance (when
# the foot is directly under the hip and vertical load is maximum).
# This prevents knee buckling by creating higher ground reaction forces.
STANCE_PUSH_DEPTH     = 0.05   # m  extra push at peak of stance
STANCE_BASE_DEPTH     = 0.04   # m  minimum ground penetration during stance
STANCE_PUSH_FRONT_EXTRA = 0.02 # m  front legs push even deeper (55% weight)

# ── Joint order (17 joints — spine + 4 legs × 4) ─────────────────────────────
JOINT_ORDER = [
    'spine_yaw_joint',
    'fl_hip_joint', 'fl_thigh_joint', 'fl_knee_joint', 'fl_cannon_joint',
    'fr_hip_joint', 'fr_thigh_joint', 'fr_knee_joint', 'fr_cannon_joint',
    'rl_hip_joint', 'rl_thigh_joint', 'rl_knee_joint', 'rl_cannon_joint',
    'rr_hip_joint', 'rr_thigh_joint', 'rr_knee_joint', 'rr_cannon_joint',
]
N_JOINTS  = len(JOINT_ORDER)   # 17
LEG_ORDER = ('fl', 'fr', 'rl', 'rr')


# ── IK — front legs (elbow-DOWN, passive cannon linkage) ──────────────────────

def _ik_3d_front(fx: float, fy: float, fz: float):
    reach = math.sqrt(fx**2 + fz**2)
    hip_z = math.atan2(fy, reach) if reach > 0.001 else 0.0
    hip_z = max(-MAX_SHOULDER, min(MAX_SHOULDER, hip_z))

    fx_2d = fx * math.cos(hip_z) + fy * math.sin(hip_z)
    fz_2d = fz

    r     = math.sqrt(fx_2d**2 + fz_2d**2)
    r     = max(abs(L1_FRONT - L2_FRONT) + 0.001, min(r, L1_FRONT + L2_FRONT - 0.001))
    cos_k = (r**2 - L1_FRONT**2 - L2_FRONT**2) / (2.0 * L1_FRONT * L2_FRONT)
    cos_k = max(-1.0, min(1.0, cos_k))

    knee   = -math.acos(cos_k)
    alpha  = math.atan2(fx_2d, -fz_2d)
    beta   = math.asin(max(-1.0, min(1.0, L2_FRONT * math.sin(abs(knee)) / r)))
    thigh  = alpha + beta
    # Front leg: passive parallelogram linkage keeps cannon near-vertical
    cannon = CANNON_LEAN - (thigh + knee)
    cannon = max(-2.0, min(2.0, cannon))
    return hip_z, thigh, knee, cannon


# ── IK — rear legs (elbow-UP, reciprocal apparatus) ───────────────────────────

def _ik_3d_rear(fx: float, fy: float, fz: float):
    reach = math.sqrt(fx**2 + fz**2)
    hip_z = math.atan2(fy, reach) if reach > 0.001 else 0.0
    hip_z = max(-MAX_SHOULDER, min(MAX_SHOULDER, hip_z))

    fx_2d = fx * math.cos(hip_z) + fy * math.sin(hip_z)
    fz_2d = fz

    r     = math.sqrt(fx_2d**2 + fz_2d**2)
    r     = max(abs(L1_REAR - L2_REAR) + 0.001, min(r, L1_REAR + L2_REAR - 0.001))
    cos_k = (r**2 - L1_REAR**2 - L2_REAR**2) / (2.0 * L1_REAR * L2_REAR)
    cos_k = max(-1.0, min(1.0, cos_k))

    knee   = +math.acos(cos_k)
    alpha  = math.atan2(fx_2d, -fz_2d)
    beta   = math.asin(max(-1.0, min(1.0, L2_REAR * math.sin(knee) / r)))
    thigh  = alpha - beta
    # Rear leg: reciprocal apparatus couples stifle (knee) to hock (cannon)
    cannon = RECIP_OFFSET - RECIP_RATIO * knee
    cannon = max(-2.0, min(2.0, cannon))
    return hip_z, thigh, knee, cannon


# ── Bovine swing profile (3-phase: Lift → Carry → Plant) ──────────────────
#
# Phase 0.00–0.30  LIFT   : hoof rises from liftoff position
# Phase 0.30–0.70  CARRY  : hoof at peak height, sweeps forward
# Phase 0.70–1.00  PLANT  : hoof descends to target touchdown position
#
# Front legs step HIGHER than rear (weight-bearing role requires clearance).
# Rear legs have a slightly longer stride (propulsive overtracking).

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
                                   gait: dict) -> float:
    """
    Bidirectional pitch-adaptive step height with front/rear differentiation.
    Front legs (load-bearing) step higher than rear (propulsive).
    """
    is_front = leg in FRONT_LEGS
    base = gait.get('step_height_front' if is_front else 'step_height_rear',
                    gait.get('step_height', 0.06))
    lx    = LEG_POS[leg][0]
    extra = lx * math.sin(shaft_pitch) * PITCH_STEP_SCALE
    return max(MIN_STEP_HEIGHT, base + extra)


# ── Arc-following foot target (Raibert) ──────────────────────────────────────

def _foot_target_3d(leg: str, phase: float, linear_v: float, angular_v: float,
                    ankle_height: float, gait: dict, step_height: float,
                    balance_offset: float = 0.0, speed_ramp: float = 1.0):
    """
    Raibert arc-following foot placement with speed-dependent stride length.
      step_height    : pitch-adapted step height for this leg.
      balance_offset : extra fx shift for body lean correction (negative = push back).
      speed_ramp     : 0→1, scales stride from min_stride to stride_length.
                       Models acceleration: first steps are short, full speed = long strides.
    """
    lx, ly   = LEG_POS[leg]
    T_stance = gait['period'] * (1.0 - gait['swing_frac'])

    # Speed-dependent stride length (overrides pure Raibert when available)
    sf = max(0.0, min(1.0, speed_ramp))
    min_s = gait.get('min_stride', 0.10)
    max_s = gait.get('stride_length', 0.50)
    target_stride = min_s + (max_s - min_s) * sf

    # Raibert component (for direction and angular velocity modulation)
    # angular_v differential: inner legs take shorter strides during turns
    fx_speed = (linear_v - angular_v * ly) * T_stance / 2.0
    fy_speed = (angular_v * lx)            * T_stance / 2.0

    # Scale fx_speed to match target stride length, PRESERVING the turning
    # differential from angular_v (don't just replace with a fixed value)
    if abs(linear_v) > 0.05:
        fx_half = target_stride / 2.0
        base_fx = linear_v * T_stance / 2.0
        if abs(base_fx) > 0.001:
            scale = fx_half / abs(base_fx)
            fx_speed *= scale
            fy_speed *= scale
        else:
            fx_speed = math.copysign(fx_half, linear_v)

    # Step height also scales with speed ramp (70% at startup, 100% at full speed)
    step_height = step_height * (0.70 + 0.30 * sf)

    # Foot placement: positive fx = foot IN FRONT of hip at touchdown
    # Stance sweeps from +fx_mean (front) to -fx_mean (back) → pushes body forward
    fx_mean = fx_speed + balance_offset
    fy_mean = fy_speed
    fx_mean  = max(-MAX_STEP,       min(MAX_STEP,       fx_mean))
    fy_mean  = max(-MAX_STEP * 0.5, min(MAX_STEP * 0.5, fy_mean))

    local    = (phase - gait['phase_offsets'][leg]) % 1.0

    if local < gait['swing_frac']:
        p = local / gait['swing_frac']
        return _elk_swing(p, fx_mean, fy_mean, ankle_height, step_height)
    else:
        # STANCE PHASE — virtual spring ground contact model
        # The foot pushes into the ground with a sinusoidal profile:
        # shallow at touchdown/liftoff, deepest at mid-stance (peak load).
        # Front legs push harder because they carry 55-70% of body weight.
        p_stance = (local - gait['swing_frac']) / (1.0 - gait['swing_frac'])
        fx = fx_mean * (1.0 - 2.0 * p_stance)
        fy = fy_mean * (1.0 - 2.0 * p_stance)
        # Mid-stance push: sine profile peaks at p_stance=0.5
        push = STANCE_BASE_DEPTH + STANCE_PUSH_DEPTH * math.sin(math.pi * p_stance)
        if leg in FRONT_LEGS:
            push += STANCE_PUSH_FRONT_EXTRA
        fz = -(ankle_height + push)
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
        self._speed_ramp = 0.0  # 0→1, controls stride length ramp-up

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
        self.get_logger().info('  Highland Cow — Bovine Gait Controller')
        self.get_logger().info('  Articulated spine + reciprocal apparatus (rear legs)')
        self.get_logger().info('  QDD actuation model, 55/45 weight distribution')
        self.get_logger().info('  WALK: 4-beat lateral (T=1.1s, stride=1.10m)')
        self.get_logger().info('  TROT: diagonal (T=0.75s, stride=1.50m)')
        self.get_logger().info('  Turning: spine yaw + differential stride')
        self.get_logger().info('  W = WALK  |  W again = TROT  |  S = step back  |  SPACE = IDLE')
        self.get_logger().info('  Body mass: ~107 kg (150 cm Highland Cow)')
        self.get_logger().info('=' * 60)

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x
        self._angular_z = msg.angular.z
        # Discrete state machine: teleop sends 0.0=IDLE, 0.8=WALK, 1.8=TROT
        if v < IDLE_THRESHOLD:
            self._linear_x = 0.0
            self._speed_ramp = 0.0  # reset ramp on stop
            new_idx = 0
        elif v < TROT_THRESHOLD:
            self._linear_x = WALK_SPEED
            new_idx = 0
        else:
            self._linear_x = TROT_SPEED
            new_idx = 1
        if new_idx != self._gait_idx:
            self._gait_idx = new_idx
            self._gait = GAIT_SEQUENCE[new_idx].copy()
            # On gait transition: reset ramp to transition value (not zero)
            self._speed_ramp = SPEED_RAMP_TRANSITION
            self._update_timer()
            self.get_logger().info(f'Gait: {self._gait["name"]} (stride ramp reset)')
        # When starting from idle, begin with initial ramp
        if self._linear_x > 0 and self._speed_ramp < SPEED_RAMP_INITIAL:
            self._speed_ramp = SPEED_RAMP_INITIAL

    def _imu_cb(self, msg: Imu):
        self._body_pitch, self._body_roll = _quaternion_to_pitch_roll(msg.orientation)

    def _update_timer(self):
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
            self._speed_ramp = 0.0
            return

        # Ramp up speed fraction each tick (smooth acceleration)
        self._speed_ramp = min(1.0, self._speed_ramp + SPEED_RAMP_RATE)

        gait  = self._gait
        pitch = self._body_pitch
        roll  = self._body_roll

        # ── Spine yaw for turning ────────────────────────────────────
        # Proportional to angular velocity command, with speed ramp
        spine_yaw = ang * SPINE_TURN_GAIN
        spine_yaw = max(-MAX_SPINE_YAW, min(MAX_SPINE_YAW, spine_yaw))

        # Ankle height adjustment: pitched body → front legs reach deeper, rear less
        pitch_adj = math.tan(-pitch) * HALF_BODY_LENGTH
        pitch_adj = max(-0.20, min(0.20, pitch_adj))
        ankle_heights = {
            'fl': ANKLE_HEIGHT_FRONT + pitch_adj,
            'fr': ANKLE_HEIGHT_FRONT + pitch_adj,
            'rl': ANKLE_HEIGHT_REAR - pitch_adj,
            'rr': ANKLE_HEIGHT_REAR - pitch_adj,
        }

        # Bidirectional pitch-adaptive step heights (front/rear differentiated)
        step_heights = {
            leg: _pitch_compensated_step_height(leg, -pitch, gait)
            for leg in LEG_ORDER
        }

        # Balance correction
        balance_offset = -pitch * BALANCE_FOOT_GAIN

        n_pts = 50
        dt    = gait['period'] / n_pts

        msg_out = JointTrajectory()
        msg_out.joint_names = JOINT_ORDER

        for i in range(n_pts):
            phase     = (self._phase + i / n_pts) % 1.0
            positions = [spine_yaw]   # spine_yaw_joint is first in JOINT_ORDER

            for leg in LEG_ORDER:
                ank_h  = ankle_heights[leg]
                eff_ph = self._adjusted_phase(leg, phase, gait)

                fx, fy, fz = _foot_target_3d(
                    leg, eff_ph, lin, ang, ank_h, gait,
                    step_heights[leg], balance_offset,
                    speed_ramp=self._speed_ramp)

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
        # Spine: gently return to center, or hold turn angle
        spine_yaw = max(-MAX_SPINE_YAW, min(MAX_SPINE_YAW,
                        self._angular_z * SPINE_TURN_GAIN))

        # Turning: differential hip splay
        shoulder = max(-0.12, min(0.12, self._angular_z * 0.06))

        # IMU balance correction
        pitch_corr = max(-0.20, min(0.20, -self._body_pitch * 0.30))
        roll_corr  = max(-0.10, min(0.10,  self._body_roll  * 0.20))

        fl_hip = shoulder + roll_corr
        fr_hip = shoulder - roll_corr
        rl_hip = shoulder + roll_corr
        rr_hip = shoulder - roll_corr

        fl_th = NEUTRAL_THIGH_FRONT + pitch_corr
        fr_th = NEUTRAL_THIGH_FRONT + pitch_corr
        rl_th = NEUTRAL_THIGH_REAR  + pitch_corr
        rr_th = NEUTRAL_THIGH_REAR  + pitch_corr

        fl_ca = CANNON_LEAN - (fl_th + NEUTRAL_KNEE_FRONT)
        fr_ca = CANNON_LEAN - (fr_th + NEUTRAL_KNEE_FRONT)
        rl_ca = RECIP_OFFSET - RECIP_RATIO * NEUTRAL_KNEE_REAR
        rr_ca = RECIP_OFFSET - RECIP_RATIO * NEUTRAL_KNEE_REAR

        msg_out = JointTrajectory()
        msg_out.joint_names = JOINT_ORDER
        pt = JointTrajectoryPoint()
        pt.positions = [
            spine_yaw,
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
