"""
blendspace_node.py  —  Advanced velocity-driven gait controller.

Implements 5 standard quadruped improvements:
  1. 3D IK  — Z-axis hip handles lateral foot offset (abduction equivalent)
  2. Raibert + arc-following foot placement  — step targets computed from
       actual cmd_vel so each leg's stride matches its circular arc radius
  3. Active cart steering  — cart_to_shaft_steer commanded directly for
       clean, responsive turning
  4. Body leveling  — shaft_to_horse pitch feedback adjusts front/rear leg
       reach to keep the horse body level on slopes
  5. Adaptive contact detection  — knee joint effort spike during swing
       triggers early stance transition for terrain-adaptive timing

Subscribes:
    /cmd_vel    geometry_msgs/Twist   linear.x=speed, angular.z=turn rate
    /cmd_gait   std_msgs/String       "trot" | "gallop"
    /joint_states sensor_msgs/JointState  for leveling + contact detection

Publishes:
    /leg_controller/joint_trajectory  trajectory_msgs/JointTrajectory
    (13 joints: cart_to_shaft_steer + 4x[hip,thigh,knee])

Gait toggle: publish "gallop" to /cmd_gait or press Q in teleop_key.
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
FOOT_RADIUS = 0.04  # foot sphere  [m]

# Hip pivot positions relative to base_link (x=fwd, y=left, z=up)
LEG_POS = {
    'fl': ( 0.35,  0.18),   # (lx, ly) — front-left
    'fr': ( 0.35, -0.18),
    'rl': (-0.35,  0.18),
    'rr': (-0.35, -0.18),
}
HALF_BODY_LENGTH = 0.35   # approximate half front-rear hip separation [m]

# ── Default gait parameters ───────────────────────────────────────────────────
BODY_HEIGHT  = 0.90
NEUTRAL_THIGH = -0.344
NEUTRAL_KNEE  = +0.651

# Trot: diagonal pairs
TROT = dict(
    phase_offsets={'fl': 0.0, 'fr': 0.5, 'rl': 0.5, 'rr': 0.0},
    swing_frac=0.40,
    step_height=0.12,
    period=1.0,
)

# Gallop/bound: front pair + rear pair staggered
GALLOP = dict(
    phase_offsets={'fl': 0.0, 'fr': 0.10, 'rl': 0.50, 'rr': 0.60},
    swing_frac=0.50,
    step_height=0.18,
    period=0.65,
)

# ── Control limits ────────────────────────────────────────────────────────────
MAX_STEER    = 0.45   # max cart steering angle [rad] (~26 deg)
MAX_SHOULDER = 0.20   # max hip yaw added per-leg from arc IK [rad]
MAX_SPEED    = 0.8    # linear.x beyond which step scales at 1.0
MAX_STEP     = 0.28   # hard cap on stride half-width [m]

# Contact detection: knee effort threshold [Nm]
CONTACT_EFFORT_THRESHOLD = 40.0
# Minimum fraction of swing that must complete before early contact counts
CONTACT_MIN_SWING_FRAC   = 0.45

# ── Joint order sent to leg_controller (13 joints) ───────────────────────────
JOINT_ORDER = [
    'cart_to_shaft_steer',
    'fl_hip_joint', 'fl_thigh_joint', 'fl_knee_joint',
    'fr_hip_joint', 'fr_thigh_joint', 'fr_knee_joint',
    'rl_hip_joint', 'rl_thigh_joint', 'rl_knee_joint',
    'rr_hip_joint', 'rr_thigh_joint', 'rr_knee_joint',
]
N_JOINTS = len(JOINT_ORDER)
LEG_ORDER = ('fl', 'fr', 'rl', 'rr')


# ── 3D IK ─────────────────────────────────────────────────────────────────────

def _ik_3d(fx: float, fy: float, fz: float):
    """
    Elbow-up 3D IK for Z-axis hip + Y-axis thigh + Y-axis knee.

    Returns (hip_z, thigh_y, knee_y).

    The hip Z angle sweeps the leg plane to accommodate a lateral foot offset.
    Formula: hip_z = atan2(fy, sqrt(fx^2 + fz^2))
    This gives small angles for small lateral offsets (correct behaviour).
    """
    reach = math.sqrt(fx**2 + fz**2)
    hip_z = math.atan2(fy, reach) if reach > 0.001 else 0.0
    hip_z = max(-MAX_SHOULDER, min(MAX_SHOULDER, hip_z))

    # Project foot onto the 2D leg plane after hip rotation
    cos_h = math.cos(hip_z)
    sin_h = math.sin(hip_z)
    fx_2d = fx * cos_h + fy * sin_h   # forward in rotated plane
    fz_2d = fz                          # vertical unchanged

    # 2D elbow-up IK
    r = math.sqrt(fx_2d**2 + fz_2d**2)
    r = max(abs(L1 - L2) + 0.001, min(r, L1 + L2 - 0.001))
    cos_k = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_k = max(-1.0, min(1.0, cos_k))
    knee = +math.acos(cos_k)
    alpha = math.atan2(fx_2d, -fz_2d)
    beta  = math.asin(max(-1.0, min(1.0, L2 * math.sin(knee) / r)))
    thigh = alpha - beta

    return hip_z, thigh, knee


# ── Arc-following foot target (Raibert-inspired) ──────────────────────────────

def _foot_target_3d(leg: str, phase: float, linear_v: float, angular_v: float,
                    body_height: float, step_height: float,
                    phase_offsets: dict, swing_frac: float, period: float):
    """
    Compute 3D foot target in hip frame using arc-following kinematics.

    Based on the Raibert heuristic: the foot landing position is determined
    by the velocity each leg's hip is moving at, so that no foot slip occurs.

    For hip at (lx, ly) from body centre moving at (linear_v, angular_v):
        v_hip = (linear_v - angular_v * ly,  angular_v * lx)

    The stance foot must travel -v_hip * T_stance to keep ground contact.
    This automatically produces:
        - Differential stride length for turning (inner legs shorter)
        - Correct lateral sweep to follow the arc

    Returns (fx, fy, fz) in hip frame.
    """
    lx, ly = LEG_POS[leg]
    T_stance = period * (1.0 - swing_frac)

    # Raibert step means: half the total foot travel during stance
    fx_mean = (linear_v - angular_v * ly) * T_stance / 2.0
    fy_mean = (angular_v * lx)             * T_stance / 2.0

    # Clamp to avoid excessive reach
    fx_mean = max(-MAX_STEP, min(MAX_STEP, fx_mean))
    fy_mean = max(-MAX_STEP * 0.5, min(MAX_STEP * 0.5, fy_mean))

    local = (phase - phase_offsets[leg]) % 1.0

    if local < swing_frac:
        p = local / swing_frac
        # Swing: arc from (-fx_mean, -fy_mean) to (+fx_mean, +fy_mean)
        fx = -fx_mean + 2.0 * fx_mean * p
        fy = -fy_mean + 2.0 * fy_mean * p
        fz = -(body_height - step_height * math.sin(math.pi * p))
    else:
        p = (local - swing_frac) / (1.0 - swing_frac)
        # Stance: foot sweeps from (+fx_mean, +fy_mean) to (-fx_mean, -fy_mean)
        fx = fx_mean * (1.0 - 2.0 * p)
        fy = fy_mean * (1.0 - 2.0 * p)
        fz = -body_height

    return fx, fy, fz


# ── FK: estimate foot height for contact detection ────────────────────────────

def _foot_z_in_hip(thigh: float, knee: float) -> float:
    """Estimate foot Z relative to hip pivot (negative = below hip)."""
    # Thigh pivot is 0.08 m below hip_link; foot hangs from there
    return -0.08 - L1 * math.cos(thigh) - L2 * math.cos(thigh + knee)


# ── Main node ─────────────────────────────────────────────────────────────────

class BlendspaceNode(Node):

    def __init__(self):
        super().__init__('blendspace_node')

        # ── Publishers / subscribers ──────────────────────────────────
        self._pub = self.create_publisher(
            JointTrajectory, '/leg_controller/joint_trajectory', 10)
        self._sub_vel = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self._sub_gait = self.create_subscription(
            String, '/cmd_gait', self._gait_cb, 10)
        self._sub_js = self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10)

        # ── cmd_vel state ─────────────────────────────────────────────
        self._linear_x  = 0.0
        self._angular_z = 0.0

        # ── Gait state ────────────────────────────────────────────────
        self._gait = TROT.copy()
        self._gait_mode = 'trot'
        self._phase = 0.0

        # ── Body leveling (improvement #4) ────────────────────────────
        # shaft_to_horse joint position (revolute-Y); positive = nose up
        self._shaft_pitch = 0.0

        # ── Contact detection (improvement #5) ────────────────────────
        # Per-leg: current swing progress and early-contact flag
        self._swing_progress = {l: 0.0 for l in LEG_ORDER}
        self._early_contact   = {l: False for l in LEG_ORDER}
        self._knee_effort     = {l: 0.0   for l in LEG_ORDER}
        self._last_phase      = {l: 0.0   for l in LEG_ORDER}

        # Publish every half gait cycle
        self._gait_timer = self.create_timer(self._gait['period'] / 2.0, self._tick)

        self.get_logger().info('=' * 55)
        self.get_logger().info('  Blendspace node v2 ready — 5 improvements active')
        self.get_logger().info('    /cmd_vel : linear.x=fwd, angular.z=left')
        self.get_logger().info('    /cmd_gait: "trot" | "gallop"')
        self.get_logger().info('    teleop   : ros2 run robotic_horse_control teleop_key')
        self.get_logger().info('=' * 55)

    # ── Callbacks ─────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        self._linear_x  = msg.linear.x
        self._angular_z = msg.angular.z

    def _gait_cb(self, msg: String):
        mode = msg.data.lower().strip()
        if mode == 'gallop' and self._gait_mode != 'gallop':
            self._gait = GALLOP.copy()
            self._gait_mode = 'gallop'
            # Reschedule timer for new period
            self._reschedule_timer()
            self.get_logger().info('Gait -> GALLOP  (step_height=0.18m, period=0.65s)')
        elif mode == 'trot' and self._gait_mode != 'trot':
            self._gait = TROT.copy()
            self._gait_mode = 'trot'
            self._reschedule_timer()
            self.get_logger().info('Gait -> TROT  (step_height=0.12m, period=1.0s)')

    def _reschedule_timer(self):
        """Destroy the old half-cycle timer and create a new one."""
        if hasattr(self, '_gait_timer'):
            self._gait_timer.cancel()
        self._gait_timer = self.create_timer(
            self._gait['period'] / 2.0, self._tick)

    def _joint_states_cb(self, msg: JointState):
        """
        Extract shaft_to_horse pitch (body leveling) and knee efforts
        (contact detection) from /joint_states.
        """
        name_to_idx = {n: i for i, n in enumerate(msg.name)}

        # Body leveling: shaft_to_horse pitch
        if 'shaft_to_horse' in name_to_idx:
            self._shaft_pitch = msg.position[name_to_idx['shaft_to_horse']]

        # Contact detection: track knee effort per leg
        has_effort = len(msg.effort) == len(msg.name)
        if has_effort:
            for leg in LEG_ORDER:
                key = f'{leg}_knee_joint'
                if key in name_to_idx:
                    self._knee_effort[leg] = abs(msg.effort[name_to_idx[key]])

    # ── Main gait tick ─────────────────────────────────────────────────

    def _tick(self):
        lin = self._linear_x
        ang = self._angular_z
        gait = self._gait

        if abs(lin) < 0.01 and abs(ang) < 0.01:
            self._publish_idle()
            return

        # ── Improvement #4: body leveling ─────────────────────────────
        # shaft_to_horse > 0 → horse nose up (uphill or terrain pitch)
        # Front legs need more reach downward; rear legs need less
        pitch_adj = math.tan(self._shaft_pitch) * HALF_BODY_LENGTH
        pitch_adj = max(-0.12, min(0.12, pitch_adj))   # clamp to ±12 cm
        body_heights = {
            'fl': BODY_HEIGHT + pitch_adj,
            'fr': BODY_HEIGHT + pitch_adj,
            'rl': BODY_HEIGHT - pitch_adj,
            'rr': BODY_HEIGHT - pitch_adj,
        }

        # ── Active cart steering (improvement #3 / turning fix) ───────
        turn_norm   = max(-1.0, min(1.0, ang / 1.0))
        steer_angle = turn_norm * MAX_STEER

        # ── Build trajectory ──────────────────────────────────────────
        n_pts = 50
        dt    = gait['period'] / n_pts

        msg_out = JointTrajectory()
        msg_out.joint_names = JOINT_ORDER

        for i in range(n_pts):
            phase = (self._phase + i / n_pts) % 1.0
            positions = [steer_angle]   # cart_to_shaft_steer first

            for leg in LEG_ORDER:
                bh = body_heights[leg]

                # ── Improvement #5: contact-adaptive phase ────────────
                effective_phase = self._adjusted_phase(leg, phase, gait)

                # ── Improvement #1+2: arc-following 3D foot target ────
                fx, fy, fz = _foot_target_3d(
                    leg, effective_phase, lin, ang,
                    bh, gait['step_height'],
                    gait['phase_offsets'],
                    gait['swing_frac'],
                    gait['period'],
                )

                # ── Improvement #1: 3D IK ────────────────────────────
                hip_z, thigh, knee = _ik_3d(fx, fy, fz)

                positions += [hip_z, thigh, knee]

            pt = JointTrajectoryPoint()
            pt.positions  = positions
            pt.velocities = [0.0] * N_JOINTS
            t_sec = i * dt
            pt.time_from_start = Duration(
                sec=int(t_sec),
                nanosec=int((t_sec % 1.0) * 1e9),
            )
            msg_out.points.append(pt)

        self._pub.publish(msg_out)

        # ── Update contact state for next tick ────────────────────────
        self._update_contact(phase, gait)
        self._phase = (self._phase + 0.5) % 1.0

    def _adjusted_phase(self, leg: str, phase: float, gait: dict) -> float:
        """
        Improvement #5: return a (possibly shortened) swing phase if
        contact was detected early.  Early contact compresses the swing
        so the foot plants at the detected contact point.
        """
        local = (phase - gait['phase_offsets'][leg]) % 1.0
        if self._early_contact[leg] and local < gait['swing_frac']:
            # Compress remaining swing to CONTACT_MIN_SWING_FRAC
            compression = CONTACT_MIN_SWING_FRAC / gait['swing_frac']
            local = min(local * compression, gait['swing_frac'] - 0.001)
            return (gait['phase_offsets'][leg] + local) % 1.0
        return phase

    def _update_contact(self, phase: float, gait: dict):
        """Update per-leg early-contact flags from knee effort."""
        for leg in LEG_ORDER:
            local = (phase - gait['phase_offsets'][leg]) % 1.0
            in_swing = local < gait['swing_frac']

            if in_swing:
                swing_progress = local / gait['swing_frac']
                if (swing_progress > CONTACT_MIN_SWING_FRAC and
                        self._knee_effort[leg] > CONTACT_EFFORT_THRESHOLD):
                    self._early_contact[leg] = True
                else:
                    self._early_contact[leg] = False
            else:
                self._early_contact[leg] = False

    # ── Idle: hold Spot neutral pose with steering at requested angle ─

    def _publish_idle(self):
        ang = self._angular_z
        turn_norm   = max(-1.0, min(1.0, ang / 1.0))
        steer_angle = turn_norm * MAX_STEER

        # Small shoulder lean toward turn direction when idling
        shoulder = max(-0.08, min(0.08, turn_norm * 0.08))

        msg_out = JointTrajectory()
        msg_out.joint_names = JOINT_ORDER
        pt = JointTrajectoryPoint()
        pt.positions = [
            steer_angle,
            shoulder, NEUTRAL_THIGH, NEUTRAL_KNEE,   # FL
            shoulder, NEUTRAL_THIGH, NEUTRAL_KNEE,   # FR
            shoulder, NEUTRAL_THIGH, NEUTRAL_KNEE,   # RL
            shoulder, NEUTRAL_THIGH, NEUTRAL_KNEE,   # RR
        ]
        pt.velocities = [0.0] * N_JOINTS
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
