#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, tty, termios, select

MOVE_BINDINGS = {
    'w': (1, 0, 0),   # forward
    's': (-1, 0, 0),  # backward
    'a': (0, 1, 0),   # strafe left
    'd': (0, -1, 0),  # strafe right
    'q': (0, 0, 1),   # rotate left (ccw)
    'e': (0, 0, -1),  # rotate right (cw)
}

SPEED = 0.4   # m/s, starting value
TURN = 0.8   # rad/s, starting value
STEP = 0.1   # amount +/- changes speed each press
MAX_SPEED = 1.5
MIN_SPEED = 0.05

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    global SPEED, TURN
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = Node('simple_teleop')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    print("""
Simple WASD Teleop
-------------------
  w = forward     s = backward
  a = strafe left d = strafe right
  q = rotate left e = rotate right
  + = speed up     - = speed down
  x = STOP (hold position)
  Ctrl-C = quit
""")
    print(f"Speed: {SPEED:.2f} m/s | Turn: {TURN:.2f} rad/s")

    try:
        while True:
            key = get_key(settings)
            twist = Twist()

            if key in MOVE_BINDINGS:
                x, y, w = MOVE_BINDINGS[key]
                twist.linear.x, twist.linear.y, twist.angular.z = x*SPEED, y*SPEED, w*TURN
                pub.publish(twist)

            elif key == 'x':
                pub.publish(Twist())
                print("STOPPED (holding position)")

            elif key == '+':
                SPEED = min(MAX_SPEED, SPEED + STEP)
                TURN = min(MAX_SPEED * 2.5, TURN + STEP * 2.5)
                print(f"Speed: {SPEED:.2f} m/s | Turn: {TURN:.2f} rad/s")

            elif key == '-':
                SPEED = max(MIN_SPEED, SPEED - STEP)
                TURN = max(MIN_SPEED * 2.5, TURN - STEP * 2.5)
                print(f"Speed: {SPEED:.2f} m/s | Turn: {TURN:.2f} rad/s")

            elif key == '\x03':
                break

    finally:
        pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
