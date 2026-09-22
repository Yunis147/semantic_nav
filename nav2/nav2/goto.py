#!/usr/bin/env python3
"""
XLeRobot Waypoint Navigator
----------------------------
Usage:
  python3 goto.py

Then type a location name and press Enter:
  table1, table2, table3, table4
  or 'list' to show all locations
  or 'quit' to exit
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math
import sys
import threading


# ─────────────────────────────────────────────
# FILL IN YOUR LOCATIONS HERE
# Get values by driving to each spot and running:
#   ros2 topic echo /amcl_pose --once
# yaw = 2 * atan2(z, w) from the quaternion
# ─────────────────────────────────────────────
LOCATIONS = {
    "table1": {"x": 4.524, "y": -2.286, "yaw":  0.0},
    "table2": {"x": 2.400, "y": -3.232, "yaw":  3.14},
    "table3": {"x": 2.526, "y": -5.397, "yaw":  3.14},
    "table4": {"x": 5.050, "y": -6.010, "yaw":  0.0},
    "home"  : {"x": -0.005, "y": -0.059, "yaw": 0.0},
}


def yaw_to_quaternion(yaw):
    """Convert yaw angle (radians) to quaternion (z, w)."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for Nav2 navigate_to_pose action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 ready!')
        self._current_goal_handle = None
        self._navigating = False

    def go_to(self, name):
        if name not in LOCATIONS:
            print(f"Unknown location: '{name}'. Type 'list' to see available locations.")
            return

        if self._navigating:
            print("Already navigating — cancelling current goal first...")
            if self._current_goal_handle:
                self._current_goal_handle.cancel_goal_async()

        loc = LOCATIONS[name]
        x, y, yaw = loc["x"], loc["y"], loc["yaw"]
        qz, qw = yaw_to_quaternion(yaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = float(qz)
        goal_msg.pose.pose.orientation.w = float(qw)

        print(f"\nGoing to {name}: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°")
        self._navigating = True

        send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback)
        send_future.add_done_callback(
            lambda f: self._goal_response_callback(f, name))

    def _goal_response_callback(self, future, name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print(f"Goal to {name} was REJECTED by Nav2.")
            self._navigating = False
            return
        print(f"Goal accepted — navigating to {name}...")
        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._result_callback(f, name))

    def _feedback_callback(self, feedback_msg):
        # Uncomment to see distance remaining:
        # dist = feedback_msg.feedback.distance_remaining
        # print(f"  Distance remaining: {dist:.2f}m", end='\r')
        pass

    def _result_callback(self, future, name):
        result = future.result()
        status = result.status
        self._navigating = False
        self._current_goal_handle = None
        if status == 4:  # SUCCEEDED
            print(f"\n✓ Reached {name} successfully!")
        elif status == 6:  # CANCELED
            print(f"\n✗ Navigation to {name} was cancelled.")
        else:
            print(f"\n✗ Navigation to {name} failed (status={status}).")
        print("\nEnter location (or 'list'/'quit'): ", end='', flush=True)


def input_loop(navigator):
    """Runs in a separate thread — reads user input."""
    print("\n" + "="*40)
    print("XLeRobot Waypoint Navigator")
    print("="*40)
    print("Available locations:", list(LOCATIONS.keys()))
    print("Commands: list, quit")
    print("="*40 + "\n")

    while True:
        try:
            user_input = input("Enter location (or 'list'/'quit'): ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if user_input == 'quit' or user_input == 'q':
            print("Exiting...")
            rclpy.shutdown()
            sys.exit(0)
        elif user_input == 'list':
            print("\nAvailable locations:")
            for name, loc in LOCATIONS.items():
                print(f"  {name}: x={loc['x']:.3f}, y={loc['y']:.3f}, "
                      f"yaw={math.degrees(loc['yaw']):.1f}°")
            print()
        elif user_input == '':
            continue
        else:
            navigator.go_to(user_input)


def main():
    rclpy.init()
    navigator = WaypointNavigator()

    # Input loop runs in separate thread so ROS2 spin can run in main thread
    input_thread = threading.Thread(
        target=input_loop, args=(navigator,), daemon=True)
    input_thread.start()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
