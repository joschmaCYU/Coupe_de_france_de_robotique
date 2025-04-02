#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

import math

class LidarDistanceNode(Node):
    def __init__(self):
        super().__init__('lidar_distance_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Replace with your LiDAR topic
            self.scan_callback,
            10
        )
        
        self.publisher = self.create_publisher(Float32, '/front_distance', 10)

        self.target_angle_deg = 0  # Example: 30 degrees to the left

    def get_distance_at_angle(self, scan_msg, target_angle_rad):  # Add 'self' here
        # Ensure the target angle is within the scan range
        if not (scan_msg.angle_min <= target_angle_rad <= scan_msg.angle_max):
            print(f"Target angle is outside the LiDAR's scan range! {math.degrees(scan_msg.angle_min)} {math.degrees(target_angle_rad)} {math.degrees(scan_msg.angle_max)}")
            return None
        
        # Calculate the index in the ranges array
        index = int(round((target_angle_rad - scan_msg.angle_min) / scan_msg.angle_increment))
        
        # Clamp the index to valid range
        index = max(0, min(index, len(scan_msg.ranges) - 1))
        
        # Return the distance (handle NaNs and Infs)
        distance = scan_msg.ranges[index]
        return distance if not (math.isinf(distance) or math.isnan(distance)) else None

    def scan_callback(self, msg):
        # Convert target angle to radians
        target_angle_rad = math.radians(self.target_angle_deg)
        
        # Get distance at the target angle
        distance = self.get_distance_at_angle(msg, target_angle_rad)
        
        if distance is not None:
            self.get_logger().info(f'Distance at {self.target_angle_deg}°: {distance:.2f} meters')
            # Publish the distance
            distance_msg = Float32()
            distance_msg.data = distance
            self.publisher.publish(distance_msg)
        else:
            self.get_logger().warn('No valid measurement at this angle')

def main():
    rclpy.init()
    node = LidarDistanceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()