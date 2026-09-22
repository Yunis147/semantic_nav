#!/usr/bin/env python3

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus

from nav2_bridge_interfaces.srv import SetString


WAYPOINTS = {
    'table_1':   (4.524,  -2.286, 0.00),
    'table_2':   (2.400,  -3.232, 180.00),
    'table_3':   (2.526,  -5.397, 0.00),
    'table_4':   (5.050,  -6.010, 0.00),
    'delivery':  (-0.005, -0.059, 0.00),
}

# Maximum time to wait for Nav2 to accept the goal.
GOAL_TIMEOUT_SEC = 10.0

# Maximum time to wait for navigation to finish.
# 600 seconds = 10 minutes.
NAV_TIMEOUT_SEC = 600.0


def yaw_to_quaternion(yaw_deg):
    yaw = math.radians(yaw_deg)
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Nav2BridgeNode(Node):

    def __init__(self):
        super().__init__('nav2_bridge')

        self._callback_group = ReentrantCallbackGroup()

        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self._callback_group
        )

        # Current odometry
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.create_subscription(
            Odometry,
            '/odom',
            self._odom_cb,
            10,
            callback_group=self._callback_group
        )

        # Services
        self.create_service(
            SetString,
            '/nav2_bridge/navigate',
            self._handle_navigate,
            callback_group=self._callback_group
        )

        self.create_service(
            SetString,
            '/nav2_bridge/get_pose',
            self._handle_get_pose,
            callback_group=self._callback_group
        )

        self.create_service(
            SetString,
            '/nav2_bridge/cancel',
            self._handle_cancel,
            callback_group=self._callback_group
        )

        # Active Nav2 goal
        self._current_goal_handle = None
        self._nav_lock = threading.Lock()

        self.get_logger().info(
            "Nav2BridgeNode ready. Waiting for Nav2 action server..."
        )

        self._nav_client.wait_for_server()

        self.get_logger().info(
            "Nav2 action server connected. Services are live."
        )

    # ------------------------------------------------------------------
    # ODOM
    # ------------------------------------------------------------------

    def _odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        self.current_theta = 2.0 * math.atan2(qz, qw)

    # ------------------------------------------------------------------
    # NAVIGATE
    # ------------------------------------------------------------------

    def _handle_navigate(self, request, response):

        waypoint_name = request.data.strip().lower()

        self.get_logger().info(
            f"[navigate] Request received: '{waypoint_name}'"
        )

        # Validate waypoint
        if waypoint_name not in WAYPOINTS:
            response.success = False
            response.message = (
                f"Unknown waypoint '{waypoint_name}'. "
                f"Known: {list(WAYPOINTS.keys())}"
            )

            self.get_logger().warn(response.message)

            return response

        # Don't allow multiple navigation goals at once.
        with self._nav_lock:
            if self._current_goal_handle is not None:

                response.success = False
                response.message = (
                    "A navigation goal is already active."
                )

                self.get_logger().warn(response.message)

                return response

        x, y, yaw_deg = WAYPOINTS[waypoint_name]

        qz, qw = yaw_to_quaternion(yaw_deg)

        # Build goal pose
        goal_pose = PoseStamped()

        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)

        goal_pose.pose.orientation.z = float(qz)
        goal_pose.pose.orientation.w = float(qw)

        # Build Nav2 action goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose

        self.get_logger().info(
            f"[navigate] Sending goal to Nav2: "
            f"x={x} y={y} yaw={yaw_deg}°"
        )

        # --------------------------------------------------------------
        # SEND GOAL
        # --------------------------------------------------------------

        send_future = self._nav_client.send_goal_async(nav_goal)

        goal_event = threading.Event()
        goal_data = {}

        def goal_done_callback(future):

            try:
                goal_data["handle"] = future.result()

            except Exception as e:
                goal_data["error"] = e

            finally:
                goal_event.set()

        send_future.add_done_callback(goal_done_callback)

        # Wait for Nav2 to accept the goal.
        if not goal_event.wait(GOAL_TIMEOUT_SEC):

            response.success = False
            response.message = (
                f"Nav2 did not accept the goal within "
                f"{GOAL_TIMEOUT_SEC}s."
            )

            self.get_logger().error(response.message)

            return response

        # Future raised an exception.
        if "error" in goal_data:

            response.success = False
            response.message = (
                f"Failed to send Nav2 goal: "
                f"{goal_data['error']}"
            )

            self.get_logger().error(response.message)

            return response

        goal_handle = goal_data.get("handle")

        # No goal handle
        if goal_handle is None:

            response.success = False
            response.message = (
                "Nav2 returned no goal handle."
            )

            self.get_logger().error(response.message)

            return response

        # Nav2 rejected goal
        if not goal_handle.accepted:

            response.success = False
            response.message = (
                "Nav2 rejected the goal."
            )

            self.get_logger().error(response.message)

            return response

        # Store active goal
        with self._nav_lock:
            self._current_goal_handle = goal_handle

        self.get_logger().info(
            "[navigate] Goal accepted. Navigating..."
        )

        # --------------------------------------------------------------
        # WAIT FOR NAVIGATION RESULT
        # --------------------------------------------------------------

        result_future = goal_handle.get_result_async()

        result_event = threading.Event()
        result_data = {}

        def result_done_callback(future):

            try:
                result_data["result"] = future.result()

            except Exception as e:
                result_data["error"] = e

            finally:
                result_event.set()

        result_future.add_done_callback(result_done_callback)

        # Wait up to 10 minutes for Nav2 to finish.
        if not result_event.wait(NAV_TIMEOUT_SEC):

            self.get_logger().error(
                f"[navigate] Navigation exceeded "
                f"{NAV_TIMEOUT_SEC / 60:.0f} minutes."
            )

            # ----------------------------------------------------------
            # CANCEL TIMED-OUT GOAL
            # ----------------------------------------------------------

            try:

                self.get_logger().info(
                    "[navigate] Cancelling timed-out Nav2 goal..."
                )

                cancel_future = goal_handle.cancel_goal_async()

                cancel_event = threading.Event()
                cancel_data = {}

                def cancel_done_callback(future):

                    try:
                        cancel_data["result"] = future.result()

                    except Exception as e:
                        cancel_data["error"] = e

                    finally:
                        cancel_event.set()

                cancel_future.add_done_callback(
                    cancel_done_callback
                )

                # Give cancellation 10 seconds.
                cancel_event.wait(10.0)

                if "error" in cancel_data:

                    self.get_logger().error(
                        "[navigate] Error while cancelling goal: "
                        f"{cancel_data['error']}"
                    )

                else:

                    self.get_logger().info(
                        "[navigate] Timed-out goal cancellation request sent."
                    )

            except Exception as e:

                self.get_logger().error(
                    f"[navigate] Failed to cancel timed-out goal: {e}"
                )

            # Clear active goal
            with self._nav_lock:
                self._current_goal_handle = None

            response.success = False

            response.message = (
                f"Navigation to '{waypoint_name}' "
                f"timed out after "
                f"{NAV_TIMEOUT_SEC / 60:.0f} minutes."
            )

            return response

        # --------------------------------------------------------------
        # RESULT FUTURE ERROR
        # --------------------------------------------------------------

        if "error" in result_data:

            with self._nav_lock:
                self._current_goal_handle = None

            response.success = False

            response.message = (
                "Failed waiting for navigation result: "
                f"{result_data['error']}"
            )

            self.get_logger().error(response.message)

            return response

        # Get result
        result = result_data.get("result")

        # Clear active goal
        with self._nav_lock:
            self._current_goal_handle = None

        if result is None:

            response.success = False

            response.message = (
                "Nav2 returned no navigation result."
            )

            self.get_logger().error(response.message)

            return response

        # --------------------------------------------------------------
        # NAVIGATION STATUS
        # --------------------------------------------------------------

        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:

            response.success = True

            response.message = (
                f"Reached '{waypoint_name}' successfully."
            )

            self.get_logger().info(
                response.message
            )

        elif status == GoalStatus.STATUS_CANCELED:

            response.success = False

            response.message = (
                f"Navigation to '{waypoint_name}' was cancelled."
            )

            self.get_logger().warn(
                response.message
            )

        else:

            response.success = False

            response.message = (
                f"Navigation to '{waypoint_name}' failed. "
                f"Status code: {status}"
            )

            self.get_logger().warn(
                response.message
            )

        return response

    # ------------------------------------------------------------------
    # GET POSE
    # ------------------------------------------------------------------

    def _handle_get_pose(self, request, response):

        theta_deg = math.degrees(self.current_theta)

        response.success = True

        response.message = (
            f"{self.current_x:.3f},"
            f"{self.current_y:.3f},"
            f"{theta_deg:.1f}"
        )

        self.get_logger().info(
            f"[get_pose] {response.message}"
        )

        return response

    # ------------------------------------------------------------------
    # CANCEL
    # ------------------------------------------------------------------

    def _handle_cancel(self, request, response):

        with self._nav_lock:
            handle = self._current_goal_handle

        if handle is None:

            response.success = False

            response.message = (
                "No active navigation goal to cancel."
            )

            self.get_logger().warn(
                response.message
            )

            return response

        self.get_logger().info(
            "[cancel] Cancelling current navigation goal..."
        )

        cancel_future = handle.cancel_goal_async()

        cancel_event = threading.Event()
        cancel_data = {}

        def cancel_done_callback(future):

            try:
                cancel_data["result"] = future.result()

            except Exception as e:
                cancel_data["error"] = e

            finally:
                cancel_event.set()

        cancel_future.add_done_callback(
            cancel_done_callback
        )

        # Give cancellation up to 10 seconds.
        if not cancel_event.wait(10.0):

            response.success = False

            response.message = (
                "Navigation cancellation timed out."
            )

            self.get_logger().error(
                response.message
            )

            return response

        if "error" in cancel_data:

            response.success = False

            response.message = (
                f"Failed to cancel navigation: "
                f"{cancel_data['error']}"
            )

            self.get_logger().error(
                response.message
            )

            return response

        with self._nav_lock:
            self._current_goal_handle = None

        response.success = True
        response.message = "Navigation cancelled."

        self.get_logger().info(
            response.message
        )

        return response


# ----------------------------------------------------------------------
# MAIN
# ----------------------------------------------------------------------

def main():

    rclpy.init()

    node = Nav2BridgeNode()

    executor = MultiThreadedExecutor(
        num_threads=4
    )

    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
