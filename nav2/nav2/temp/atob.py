#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

class RoverNavigator(Node):
    def __init__(self):
        super().__init__('rover_navigator')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

    def send_target_pose(self, x, y, z=0.00, w=1.0):
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = float(z)
        goal_msg.pose.pose.orientation.w = float(w)

        self.get_logger().info(f'Sending new goal: X={x}, Y={y}')
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by the server.')
            return False

        self.get_logger().info('Goal accepted! Navigating...')

        get_result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, get_result_future)

        status = get_result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Destination reached successfully!')
            return True
        else:
            self.get_logger().warn(f'Navigation failed or was canceled. Status code: {status}')
            return False

def main(args=None):
    rclpy.init(args=args)
    navigator = RoverNavigator()
    
    
    t1 = navigator.send_target_pose(1.75, -0.401)
    
    if t1:
        navigator.send_target_pose(1.40, -1.901)

    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()