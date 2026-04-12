"""
teleop_key.py  —  Keyboard teleop for the Robotic Horse.

Publishes geometry_msgs/Twist to /cmd_vel at 10 Hz while a key is held,
and a zero twist when released.
Publishes std_msgs/String to /cmd_gait on gait toggle.

Controls:
    W  — forward
    S  — backward
    A  — turn left
    D  — turn right
    SPACE — stop immediately
    Q — toggle gait (trot / gallop)
    Ctrl-C — quit

Run in a SEPARATE terminal after starting the simulation:
    ros2 run robotic_horse_control teleop_key
"""

import sys
import select
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


SPEED   = 0.5   # linear.x for forward / backward
TURNING = 0.8   # angular.z for left / right

BANNER = """
╔══════════════════════════════════════╗
║    Robotic Horse Keyboard Teleop     ║
╠══════════════════════════════════════╣
║  W — Forward      S — Backward       ║
║  A — Turn Left    D — Turn Right     ║
║  Q — Toggle Gait  SPACE — Stop       ║
║  Ctrl-C — Quit                       ║
╚══════════════════════════════════════╝
"""


def _get_key(timeout: float = 0.1) -> str:
    """Read one character with timeout; return '' if none available."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class TeleopKey(Node):

    def __init__(self):
        super().__init__('teleop_key')
        self._pub      = self.create_publisher(Twist,  '/cmd_vel',  10)
        self._gait_pub = self.create_publisher(String, '/cmd_gait', 10)
        self._current_gait = 'trot'

    def _toggle_gait(self):
        self._current_gait = 'gallop' if self._current_gait == 'trot' else 'trot'
        msg = String()
        msg.data = self._current_gait
        self._gait_pub.publish(msg)
        print(f'\r  Gait → {self._current_gait.upper():<10}', end='', flush=True)

    def run(self):
        print(BANNER)
        prev_key = ''
        try:
            while rclpy.ok():
                key = _get_key(timeout=0.1)

                if key == '\x03':   # Ctrl-C
                    break

                msg = Twist()
                if key in ('w', 'W'):
                    msg.linear.x  = SPEED
                elif key in ('s', 'S'):
                    msg.linear.x  = -SPEED
                elif key in ('a', 'A'):
                    msg.angular.z = TURNING
                elif key in ('d', 'D'):
                    msg.angular.z = -TURNING
                elif key in ('q', 'Q'):
                    self._toggle_gait()
                    prev_key = key
                    continue
                elif key == ' ':
                    pass   # zero twist = stop
                elif key == '':
                    # No key pressed — stop (don't keep last command)
                    if prev_key not in ('', ' '):
                        self._pub.publish(Twist())
                    prev_key = key
                    continue
                else:
                    prev_key = key
                    continue

                self._pub.publish(msg)
                prev_key = key

        except Exception as exc:
            print(f'\nTeleop error: {exc}')
        finally:
            self._pub.publish(Twist())   # send stop on exit
            print('\nTeleop stopped.')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKey()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
