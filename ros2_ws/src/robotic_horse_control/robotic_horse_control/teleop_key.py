"""
teleop_key.py  —  Discrete state keyboard controller for Highland Cow gait.

Controls:
  W       — advance gait state: IDLE → WALK → TROT
  S       — decrease gait state: TROT → WALK → IDLE
  A       — turn left  (increments turn, decays when released)
  D       — turn right (increments turn, decays when released)
  SPACE   — immediate IDLE (all motion stops)
  Q       — quit
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


WALK_SPEED    = 0.80   # m/s published for WALK state (realistic cow walk)
TROT_SPEED    = 1.50   # m/s published for TROT state (realistic cow trot)
MAX_TURN      = 1.5    # rad/s
TURN_ACCEL    = 1.2    # rad/s per second while key held
TURN_DECAY    = 1.8    # rad/s per second natural decay
PUBLISH_HZ    = 25


HELP = """
 Highland Cow Quadruped — Keyboard Teleop
 ─────────────────────────────────────────
  W   = advance gait state (IDLE → WALK → TROT)
  S   = decrease gait state (TROT → WALK → IDLE)
  A   = turn left
  D   = turn right
  SPC = immediate IDLE (stop all motion)
  Q   = quit

 States:
   [0] IDLE  — standing still
   [1] WALK  — 4-beat lateral walk
   [2] TROT  — diagonal pair trot
"""

SPEEDS = [0.0, WALK_SPEED, TROT_SPEED]
STATE_NAMES = ['IDLE', 'WALK', 'TROT']


def get_key_nonblocking():
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            rest = sys.stdin.read(2) if sys.stdin in select.select([sys.stdin], [], [], 0.01)[0] else ''
            return ch + rest
        return ch
    return ''


class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_key')
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._state = 0    # 0=IDLE, 1=WALK, 2=TROT
        self._turn  = 0.0

        self._last_t = time.monotonic()
        self._timer  = self.create_timer(1.0 / PUBLISH_HZ, self._tick)

        self.get_logger().info(HELP)
        self.get_logger().info(f'Current state: {STATE_NAMES[self._state]}')

    def _tick(self):
        now = time.monotonic()
        dt  = now - self._last_t
        self._last_t = now

        key = get_key_nonblocking()

        if key in ('q', 'Q', '\x03'):
            self.get_logger().info('Quit.')
            rclpy.shutdown()
            return

        # State transitions (single press per tick)
        if key in ('w', 'W'):
            new_state = min(2, self._state + 1)
            if new_state != self._state:
                self._state = new_state
                self.get_logger().info(f'State → {STATE_NAMES[self._state]}')
        elif key in ('s', 'S'):
            new_state = max(0, self._state - 1)
            if new_state != self._state:
                self._state = new_state
                self.get_logger().info(f'State → {STATE_NAMES[self._state]}')
        elif key == ' ':
            self._state = 0
            self._turn  = 0.0
            self.get_logger().info('State → IDLE')

        # Turn: ramp while key held, decay otherwise
        if key in ('a', 'A'):
            self._turn = min(MAX_TURN, self._turn + TURN_ACCEL * dt)
        elif key in ('d', 'D'):
            self._turn = max(-MAX_TURN, self._turn - TURN_ACCEL * dt)
        else:
            if self._turn > 0:
                self._turn = max(0.0, self._turn - TURN_DECAY * dt)
            elif self._turn < 0:
                self._turn = min(0.0, self._turn + TURN_DECAY * dt)

        # Publish
        msg = Twist()
        msg.linear.x  = SPEEDS[self._state]
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
