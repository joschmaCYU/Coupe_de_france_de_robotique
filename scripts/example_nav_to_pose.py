#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from std_msgs.msg import Bool

import sys

class PoseNavigator(Node):
    pose_X, pose_Y, pose_theta_Z, pose_theta_W, initial_pose_X, initial_pose_Y, initial_pose_theta_Z, initial_pose_theta_W = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def __init__(self):
        super().__init__('pose_navigator')
        self.arrived_pub = self.create_publisher(Bool, '/arrived', 10)

        for i in range(len(sys.argv)):
            if sys.argv[i] == '--pose':
                self.get_logger().info("arg: " + sys.argv[i+1])
                pose_args = sys.argv[i+1].split(',')
                if (len(pose_args) != 4):
                    self.get_logger().info('Not 4 args')
                    return
                self.pose_X = float(pose_args[0])
                self.pose_Y = float(pose_args[1])
                self.pose_Z_theta = float(pose_args[2])
                self.pose_W_theta = float(pose_args[3])
            elif sys.argv[i] == '--initial_pose':
                self.get_logger().info("arg: " + sys.argv[i+1])
                pose_args = sys.argv[i+1].split(',')
                if (len(pose_args) != 4):
                    self.get_logger().info('Not 4 args')
                    return
                self.initial_pose_X = float(pose_args[0])
                self.initial_pose_Y = float(pose_args[1])
                self.initial_pose_Z_theta = float(pose_args[2])
                self.initial_pose_W_theta = float(pose_args[3])
            else:
                print(sys.argv[i])

        self.navigator = BasicNavigator()
        self.pose_callback()
        
        # self.subscription = self.create_subscription(
        #     PoseStamped,
        #     '/go_to_pose',
        #     self.pose_callback,
        #     10)

    def pose_callback(self):
        self.get_logger().info('Received goal pose, setting as initial pose and navigating.')

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.initial_pose_X
        initial_pose.pose.position.y = self.initial_pose_Y
        initial_pose.pose.orientation.z = self.initial_pose_theta_Z
        initial_pose.pose.orientation.w = self.initial_pose_theta_W
        self.navigator.setInitialPose(initial_pose)

        # Wait for navigation to fully activate
        self.navigator.waitUntilNav2Active()

        # Set goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.pose_X
        goal_pose.pose.position.y = self.pose_Y
        goal_pose.pose.orientation.z = self.pose_theta_Z
        goal_pose.pose.orientation.w = self.initial_pose_theta_W
        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                # self.get_logger().info(f'Estimated time of arrival: {eta:.2f} seconds.')

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

        # self.navigator.lifecycleShutdown()

def main():
    rclpy.init()
    pose_navigator = PoseNavigator()
    rclpy.spin(pose_navigator)
    pose_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
