#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration

class PoseNavigator(Node):
    def __init__(self):
        super().__init__('pose_navigator')
        self.navigator = BasicNavigator()
        self.subscription = self.create_subscription(
            PoseStamped,
            '/go_to_pose',
            self.pose_callback,
            10)
        self.subscription

    def pose_callback(self, msg):
        self.get_logger().info('Received goal pose, setting as initial pose and navigating.')

        # Set initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = msg.header.frame_id
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose = msg.pose
        self.navigator.setInitialPose(initial_pose)

        # Wait for navigation to fully activate
        self.navigator.waitUntilNav2Active()

        # Set goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = msg.header.frame_id
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose = msg.pose

        # Navigate to goal
        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                self.get_logger().info(f'Estimated time of arrival: {eta:.2f} seconds.')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            error_code, error_msg = self.navigator.getTaskError()
            self.get_logger().info(f'Goal failed! {error_code}: {error_msg}')
        else:
            self.get_logger().info('Goal has an invalid return status!')

        self.navigator.lifecycleShutdown()

def main(args=None):
    rclpy.init(args=args)
    pose_navigator = PoseNavigator()
    rclpy.spin(pose_navigator)
    pose_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
