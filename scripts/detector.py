#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',  # Modifie le topic si besoin
            self.scan_callback,
            10
        )
        self.publisher = self.create_publisher(
            String,
            '/send_to_arduino',
            10
        )
        self.last_out = None

    def scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        # Angles en radians
        angle_start = math.radians(5)
        angle_end = math.radians(175)

        index_start = int((angle_start - angle_min) / angle_increment)
        index_end = int((angle_end - angle_min) / angle_increment)

        detection = False
        detected_angle = None
        detected_dist = None
        for i in range(index_start, index_end + 1):
            if 0 <= i < len(ranges):
                if ranges[i] < 0.25:
                    detection = True
                    detected_dist = ranges[i]
                    detected_angle = angle_min + i * angle_increment
                    break

        if detection:
            angle_deg = math.degrees(detected_angle)
            self.get_logger().info(
                f"🚨 Objet détecté à {detected_dist:.2f} m à l'angle {angle_deg:.1f}° !"
            )
            out = String()
            out.data = 'DETECT'
        else:
            out = String()
            out.data = 'NOT_DETECT'

        # publish only if changed
        if out.data != self.last_out:
            self.publisher.publish(out)
            self.last_out = out.data



def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
