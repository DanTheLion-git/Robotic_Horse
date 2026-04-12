"""
teleop_key.py  —  Smooth velocity-ramp keyboard controller.

Controls:
  W       — accelerate forward  (speed ramps up while held)
  S       — decelerate          (speed ramps down to 0 while held)
  A       — turn left            (turn rate ramps up while held)
  D       — turn right           (turn rate ramps up while held)
  SPACE   — smooth stop          (both speed AND turn lerp to 0 over 3 s)
  Q       — quit

Speed-gait auto-switching happens in blendspace_node based on speed:
  0 – 1.5 m/s  → WALK
  1.5 – 3.5 m/s → TROT
  > 3.5 m/s    → GALLOP

The robot cannot move backward (speed clamped at 0).
"""

import sys
import os
import time
import select
import tty
import termios

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# ── Tuning ────────────────────────────────────────────────────────────────────
MAX_SPEED     = 5.0   # m/s
MAX_TURN      = 2.0   # rad/s

ACCEL         = 0.4   # m/s² forward acceleration per second
DECEL         = 0.6   # m/s² deceleration per second (S key)
TURN_ACCEL    = 0.8   # rad/s² turn acceleration
TURN_DECAY    = 1.2   # rad/s² turn decay when neither A nor D held

BRAKE_TIME    = 3.0   # seconds for SPACE to bring to full stop
PUBLISH_HZ    = 25    # cmd_vel publish rate


HELP = """
 Elk Animatronic Keyboard Teleop
 ────────────────────────────────
  W   = accelerate forward
  S   = brake / decelerate
  A   = turn left
  D   = turn right
  SPC = smooth stop (3 seconds)
  Q   = quit

  Speed auto-selects gait:
    0–1.5 m/s  → WALK
    1.5–3.5 m/s → TROT
    >3.5 m/s   → GALLOP
"""


def get_key_nonblocking():
    """Read one keypress without blocking. Returns '' if no key available."""
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        ch = sys.stdin.read(1)
        if ch == '\x1b':  # escape sequence (arrow keys)
            rest = sys.stdin.read(2) if sys.stdin in select.select([sys.stdin], [], [], 0.01)[0] else ''
            return ch + rest
        return ch
    return ''


class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_key')
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._speed = 0.0
        self._turn  = 0.0

        self._braking    = False
        self._brake_rate = 0.0   # set when SPACE pressed

        self._last_t = time.monotonic()

        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._tick)

        self.get_logger().info(HELP)

    def _tick(self):
        now = time.monotonic()
        dt  = now - self._last_t
        self._last_t = now

        key = get_key_nonblocking()

        if key in ('q', 'Q', '\x03'):
            self.get_logger().info('Quit.')
            rclpy.shutdown()
            return

        if key == ' ':
            # Space: initiate smooth stop over BRAKE_TIME seconds
            self._braking    = True
            speed_rate = self._speed / BRAKE_TIME if self._speed > 0 else 0.0
            turn_rate  = abs(self._turn) / BRAKE_TIME if abs(self._turn) > 0 else 0.0
            self._brake_speed_rate = speed_rate
            self._brake_turn_rate  = turn_rate

        # ── Update speed ──────────────────────────────────────────────────
        if self._braking:
            # Lerp both to zero at constant rate
            self._speed = max(0.0, self._speed - self._brake_speed_rate * dt)
            if self._turn > 0:
                self._turn = max(0.0, self._turn - self._brake_turn_rate * dt)
            else:
                self._turn = min(0.0, self._turn + self._brake_turn_rate * dt)
            if self._speed <= 0.001 and abs(self._turn) <= 0.001:
                self._speed   = 0.0
                self._turn    = 0.0
                self._braking = False
        else:
            if key == 'w' or key == 'W':
                self._speed = min(MAX_SPEED, self._speed + ACCEL * dt)
                self._braking = False
            elif key == 's' or key == 'S':
                self._speed = max(0.0, self._speed - DECEL * dt)

            # Turn: ramp toward target while key held, decay otherwise
            if key == 'a' or key == 'A':
                self._turn = min(MAX_TURN, self._turn + TURN_ACCEL * dt)
                self._braking = False
            elif key == 'd' or key == 'D':
                self._turn = max(-MAX_TURN, self._turn - TURN_ACCEL * dt)
                self._braking = False
            else:
                # Natural turn decay (like releasing a steering wheel)
                if self._turn > 0:
                    self._turn = max(0.0, self._turn - TURN_DECAY * dt)
                elif self._turn < 0:
                    self._turn = min(0.0, self._turn + TURN_DECAY * dt)

        # ── Publish ───────────────────────────────────────────────────────
        msg = Twist()
        msg.linear.x  = self._speed
        msg.angular.z = self._turn
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        node = TeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print('\nTeleop stopped.')
