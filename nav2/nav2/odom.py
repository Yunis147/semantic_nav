#!/usr/bin/env python3
# Drop-in replacement for base.odom.py.
#
# WHY THIS FILE EXISTS
# --------------------
# Nav2 was declaring "goal succeeded" while the robot faced 90-200 deg wrong.
# SimpleGoalChecker ALWAYS checks yaw, so success proves the ESTIMATED yaw
# matched the goal while the PHYSICAL yaw did not -> the odom->base_footprint
# heading published here is wrong. Translation (x,y) is confirmed good and is
# left byte-for-byte identical. Only the YAW axis (never exercised by W/A/S/D
# teleop) is isolated below so it can be verified and corrected.
#
# CALIBRATION (do this once, ~2 min, motors powered, nav2 NOT running).
# There are TWO independent yaw signs. Flipping them TOGETHER (as prior edits
# did: committed base.odom.py used +w/+dth, base_odom.py uses -w/-dth) flips the
# feedback-loop gain twice = no net change, which is why "flip the sign" never
# fixed the spin. Calibrate them separately with ONE test:
#
#   ros2 run <pkg> odom          # or: python3 odom.py   (nav2 off)
#   ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist \
#       '{angular: {z: 0.3}}'    # small steady rotate command
#
#   (1) COMMAND sign  -> watch the ROBOT: +z must spin CCW (counter-clockwise
#       seen from above). If it spins CW, flip CMD_YAW_SIGN.
#   (2) ODOM sign     -> watch printed theta_deg: while the robot physically
#       spins CCW, theta_deg must INCREASE. If it decreases, flip YAW_SIGN.
#       (Judge ODOM against the PHYSICAL direction, not against +z.)
#   (3) SCALE         -> one full physical turn must read ~360 deg. If it reads
#       e.g. 300 or 430, set ROBOT_RADIUS *= (measured_deg / 360).
# Both signs correct => AMCL/MPPI yaw is trustworthy and goal orientation holds.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
import tf2_ros
import math
from st3215 import ST3215

print("1. Starting imports...", flush=True)
servo = ST3215('/dev/xle_rover')
print("2. Servo connected!", flush=True)
TICKS_PER_REV = 4096
SQRT3 = 1.73205

# ---- Yaw calibration knobs (the ONLY things you should need to touch) --------
# Defaults reproduce the currently-running base_odom.py EXACTLY, so this file is
# a safe drop-in; change only what the calibration test above tells you to.
CMD_YAW_SIGN = -1.0    # sign of robot_radius*w in cmd_callback (was hard '-').
YAW_SIGN = 1.0        # sign of dth in odom (was the hard leading '-').
ROBOT_RADIUS = 0.16    # center-to-wheel distance; sets yaw SCALE.
# -----------------------------------------------------------------------------


class MotorOdom(Node):
    def __init__(self):
        super().__init__('motor_odom')
        print("3. Node constructor starting...", flush=True)
        self.wheel_radius, self.robot_radius = 0.05, ROBOT_RADIUS
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.prev_1, self.prev_2, self.prev_3 = None, None, None
        self.heading_offset_deg = 120.0
        self.last_time = None

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.timer = self.create_timer(0.033, self.update)
        self.get_logger().info(f"YAW_SIGN={YAW_SIGN}, ROBOT_RADIUS={self.robot_radius}")
        print("4. Node constructor finished, publisher/timer created.", flush=True)

    def cmd_callback(self, msg):
        # UNCHANGED from base.odom.py: confirmed correct by teleop translation.
        cmd_x, cmd_y, w = msg.linear.x, -msg.linear.y, msg.angular.z
        rad = math.radians(self.heading_offset_deg)
        vx = cmd_x * math.cos(rad) - cmd_y * math.sin(rad)
        vy = cmd_x * math.sin(rad) + cmd_y * math.cos(rad)
        wr = CMD_YAW_SIGN * self.robot_radius * w
        v1 = -(SQRT3 / 2.0) * vx + 0.5 * vy + wr
        v2 = (SQRT3 / 2.0) * vx + 0.5 * vy + wr
        v3 = -1.0 * vy + wr
        try:
            servo.Rotate(7, int(v1 * -3000))
            servo.Rotate(8, int(v2 * -3000))
            servo.Rotate(9, int(v3 * -3000))
        except Exception as e:
            self.get_logger().error(f"Rotate failed: {e}")

    def update(self):
        try:
            p1 = servo.ReadPosition(7)
            p2 = servo.ReadPosition(8)
            p3 = servo.ReadPosition(9)
        except Exception as e:
            self.get_logger().error(f"ReadPosition failed: {e}", throttle_duration_sec=2.0)
            return

        if p1 is None or p2 is None or p3 is None:
            self.get_logger().warn(f"ReadPosition None: p1={p1}, p2={p2}, p3={p3}", throttle_duration_sec=2.0)
            return

        if self.prev_1 is None:
            self.get_logger().info(f"First reading captured: p1={p1}, p2={p2}, p3={p3}")
            self.prev_1, self.prev_2, self.prev_3 = p1, p2, p3
            self.last_time = self.get_clock().now()
            return

        def delta(c, p):
            d = c - p
            if d > 2048:
                d -= 4096
            elif d < -2048:
                d += 4096
            return d * (2.0 * math.pi * self.wheel_radius / 4096)

        d1, d2, d3 = delta(p1, self.prev_1), delta(p2, self.prev_2), delta(p3, self.prev_3)
        self.prev_1, self.prev_2, self.prev_3 = p1, p2, p3

        # --- Translation: UNCHANGED (confirmed working) --------------------
        raw_dx, raw_dy = (d2 - d1) / SQRT3, (d1 + d2 - (2.0 * d3)) / 3.0
        rad_inv = math.radians(-self.heading_offset_deg)
        dx = raw_dx * math.cos(rad_inv) - raw_dy * math.sin(rad_inv)
        dy = raw_dx * math.sin(rad_inv) + raw_dy * math.cos(rad_inv)
        dx = -dx
        # --- Rotation: isolated behind YAW_SIGN (was hard-coded '-') --------
        dth = YAW_SIGN * (d1 + d2 + d3) / (3.0 * self.robot_radius)

        self.x += dx * math.cos(self.theta) - dy * math.sin(self.theta)
        self.y += dx * math.sin(self.theta) + dy * math.cos(self.theta)
        self.theta += dth

        now_ros = self.get_clock().now()
        now = now_ros.to_msg()
        dt = (now_ros - self.last_time).nanoseconds * 1e-9
        self.last_time = now_ros
        vx_b = dx / dt if dt > 1e-6 else 0.0
        vy_b = dy / dt if dt > 1e-6 else 0.0
        wz_b = dth / dt if dt > 1e-6 else 0.0

        qz, qw = math.sin(self.theta / 2.0), math.cos(self.theta / 2.0)

        t1 = TransformStamped()
        t1.header.stamp, t1.header.frame_id, t1.child_frame_id = now, "odom", "base_footprint"
        t1.transform.translation.x, t1.transform.translation.y = float(self.x), float(self.y)
        t1.transform.rotation.z, t1.transform.rotation.w = qz, qw

        t2 = TransformStamped()
        t2.header.stamp, t2.header.frame_id, t2.child_frame_id = now, "base_footprint", "base_link"
        t2.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform([t1, t2])

        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = now, "odom", "base_footprint"
        odom.pose.pose.position.x, odom.pose.pose.position.y = float(self.x), float(self.y)
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        # Twist in child (base_footprint) frame; base.odom.py left this at zero,
        # which starves MPPI's velocity seed. Now populated.
        odom.twist.twist.linear.x = float(vx_b)
        odom.twist.twist.linear.y = float(vy_b)
        odom.twist.twist.angular.z = float(wz_b)
        self.odom_pub.publish(odom)
        self.get_logger().info(
            f"odom: x={self.x:.3f} y={self.y:.3f} theta_deg={math.degrees(self.theta):.1f}",
            throttle_duration_sec=1.0)


def main():
    print("5. Calling rclpy.init()...", flush=True)
    rclpy.init()
    print("6. rclpy initialized, creating node...", flush=True)
    node = MotorOdom()
    print("7. Node created, entering spin...", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
