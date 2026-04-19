"""
teleop_key.py  —  Discrete state keyboard controller for Highland Cow gait.

Controls:
  W       — advance gait state: IDLE → WALK → TROT
  S       — decrease gait state: TROT → WALK → IDLE
  A       — toggle turn left  (press again to go straight)
  D       — toggle turn right (press again to go straight)
  SPACE   — immediate IDLE (all motion stops, steering resets)
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


WALK_SPEED    = 0.80   # m/s published for WALK state
TROT_SPEED    = 1.60   # m/s published for TROT state
TURN_RATE     = 0.6    # rad/s when turning
PUBLISH_HZ    = 25


HELP = """
 Highland Cow Quadruped — Keyboard Teleop
 ─────────────────────────────────────────
  W   = advance gait state (IDLE → WALK → TROT)
  S   = decrease gait state (TROT → WALK → IDLE)
  A   = toggle turn left  (press again → straight)
  D   = toggle turn right (press again → straight)
  SPC = immediate IDLE (stop all motion)
  Q   = quit

 States:
   [0] IDLE  — standing still
   [1] WALK  — 4-beat lateral walk
   [2] TROT  — diagonal pair trot

 Steering:
   [◀] LEFT  — turning left
   [▶] RIGHT — turning right
   [■] FWD   — going straight
"""

SPEEDS = [0.0, WALK_SPEED, TROT_SPEED]
STATE_NAMES = ['IDLE', 'WALK', 'TROT']
STEER_NAMES = ['FWD', 'LEFT', 'RIGHT']


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
        self._steer = 0    # 0=straight, 1=left, -1=right

        self._last_t = time.monotonic()
        self._timer  = self.create_timer(1.0 / PUBLISH_HZ, self._tick)

        self.get_logger().info(HELP)
        self.get_logger().info(f'State: {STATE_NAMES[self._state]}  Steer: {STEER_NAMES[self._steer]}')

    def _tick(self):
        now = time.monotonic()
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
            self._steer = 0
            self.get_logger().info('State → IDLE  Steer → FWD')

        # Steering: toggle on/off
        if key in ('a', 'A'):
            if self._steer == 1:
                self._steer = 0  # already turning left → go straight
                self.get_logger().info('Steer → FWD')
            else:
                self._steer = 1  # turn left
                self.get_logger().info('Steer → LEFT')
        elif key in ('d', 'D'):
            if self._steer == -1:
                self._steer = 0  # already turning right → go straight
                self.get_logger().info('Steer → FWD')
            else:
                self._steer = -1  # turn right
                self.get_logger().info('Steer → RIGHT')

        # Publish
        msg = Twist()
        msg.linear.x  = SPEEDS[self._state]
        msg.angular.z = self._steer * TURN_RATE
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
