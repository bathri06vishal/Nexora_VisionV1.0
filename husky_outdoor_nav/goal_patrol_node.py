#!/usr/bin/env python3
"""
Autonomous Forest Trail Waypoint Patrol Node for Husky UGV
Commands Nav2 to navigate along the off-road forest trail avoiding trees and boulders.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import math


class ForestGoalPatrolNode(Node):
    def __init__(self):
        super().__init__('forest_goal_patrol_node')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Forest trail waypoints (x, y, yaw_deg)
        self.waypoints = [
            (5.0, 0.0, 0.0),      # Straight trail section
            (11.0, 1.8, 20.0),    # Approaching gentle curve
            (14.5, 5.0, 45.0),    # Clearing rock cluster
            (17.0, 9.5, 75.0),    # Passing between pine trees
            (12.0, 14.5, 140.0),  # Loop bend
            (0.0, 0.0, 180.0)     # Return to origin
        ]
        self.current_wp_idx = 0
        self.is_navigating = False

        self.get_logger().info('Waiting for Nav2 navigate_to_pose action server...')
        self.timer = self.create_timer(2.0, self.start_patrol_loop)

    def start_patrol_loop(self):
        if self.is_navigating:
            return

        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Nav2 action server not ready yet. Waiting...')
            return

        self.send_next_goal()

    def send_next_goal(self):
        if self.current_wp_idx >= len(self.waypoints):
            self.get_logger().info('All forest trail waypoints completed!')
            self.current_wp_idx = 0

        wp = self.waypoints[self.current_wp_idx]
        x, y, yaw_deg = wp
        yaw_rad = math.radians(yaw_deg)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'odom'
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        # Euler to Quaternion
        goal_msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        self.get_logger().info(f'Navigating to Forest Waypoint [{self.current_wp_idx+1}/{len(self.waypoints)}]: (x={x}, y={y}, yaw={yaw_deg} deg)')
        self.is_navigating = True

        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by Nav2!')
            self.is_navigating = False
            return

        self.get_logger().info('Goal accepted. Navigating through forest terrain...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        self.get_logger().info(f'Waypoint reached with status: {status}')
        self.current_wp_idx += 1
        self.is_navigating = False

    def feedback_callback(self, feedback_msg):
        # Optional logging
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ForestGoalPatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
