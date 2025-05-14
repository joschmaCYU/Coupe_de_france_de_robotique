#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from std_msgs.msg import Bool

import sys

class PoseNavigator(Node):
    blue_tem = False
    pose_X, pose_Y, pose_theta, initial_pose_X, initial_pose_Y, initial_pose_theta = 0.0

    def __init__(self):
        super().__init__('pose_navigator')
        self.navigator = BasicNavigator()
        self.subscription = self.create_subscription(
            PoseStamped,
            '/go_to_pose',
            self.pose_callback,
            10)
        self.subscription
        self.arrived_pub = self.create_publisher(Bool, '/arrived', 10)

        for i in range(len(sys.argv)):
            if sys.argv[i] == '--blue_team':
                blue_team = sys.argv[i+1]
            elif sys.argv[i] == '--pose':
                pose_args = sys.argv[i+1].split(',')
                x, y, theta = map(float, pose_args)
            elif sys.argv[i] == '--initial_pose':
                pose_args = sys.argv[i+1].split(',')
                initial_pose_X, initial_pose_Y, initial_pose_theta = map(float, pose_args)
            else:
                print(sys.argv[i])

    def pose_callback(self, msg):
        self.get_logger().info('Received goal pose, setting as initial pose and navigating.')

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.initial_pose_X
        initial_pose.pose.position.y = self.initial_pose_Y
        initial_pose.pose.orientation.z = self.initial_pose_theta
        initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(initial_pose)

        # Wait for navigation to fully activate
        self.navigator.waitUntilNav2Active()

        # Set goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.pose_X
        goal_pose.pose.position.y = self.pose_Y
        goal_pose.pose.orientation.z = self.pose_theta
        goal_pose.pose.orientation.w = 1.0
        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                self.get_logger().info(f'Estimated time of arrival: {feedback.estimated_time_remaining:.2f}')
                self.get_logger().info(f'Estimated time of arrival: {eta:.2f} seconds.')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            msg = Bool()
            msg.data = True
            self.arrived_pub.publish(msg)
            self.get_logger().info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            error_code, error_msg = self.navigator.getTaskError()
            self.get_logger().info(f'Goal failed! {error_code}: {error_msg}')
        else:
            self.get_logger().info('Goal has an invalid return status!')

        self.navigator.lifecycleShutdown()

def main():
    rclpy.init()
    pose_navigator = PoseNavigator()
    rclpy.spin(pose_navigator)
    pose_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
