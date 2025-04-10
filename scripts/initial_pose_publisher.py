#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from builtin_interfaces.msg import Time

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        
        # Create publisher for initial pose
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # Publish once after 1 second delay
        self.timer = self.create_timer(1.0, self.publish_initial_pose)
        
    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        
        # Set header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # Change if using different frame
        
        # Set position (x, y, z)
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        
        # Set orientation (quaternion)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0  # Neutral orientation
        
        # Set covariance matrix (6x6 row-major)
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
        self.destroy_timer(self.timer)  # Stop after publishing once
        rclpy.shutdown()  # Optional: shutdown after publishing

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()