#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from builtin_interfaces.msg import Time
from std_msgs.msg import Bool

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        self.timer = self.create_timer(1.0, self.publish_initial_pose)
        
        self.team_blue = False
        self.team_sub = self.create_subscription(
            Bool,
            '/team_blue',
            self.team_blue_callback,
            10
        )
        
    def team_blue_callback(self, msg):
        self.team_blue = msg.data

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Set position (x, y, z)
        if self.team_blue:
            msg.pose.pose.position.x = 0.225
            msg.pose.pose.position.y = 0.875
            msg.pose.pose.orientation.z = -1.0
        else:
            msg.pose.pose.position.x = 2.775
            msg.pose.pose.position.y = 0.875
            msg.pose.pose.orientation.z = 1.0
        msg.pose.pose.position.z = 0.0
        
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        
        msg.pose.pose.orientation.w = 0.0
        
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,    # X
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,     # Y
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,      # Z
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,      # Rotation X
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,      # Rotation Y
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0685    # Yaw (15 degrees)
        ]
        
        self.publisher.publish(msg)
        self.get_logger().info('Published initial pose at (0, 0)')
        self.destroy_timer(self.timer)
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()