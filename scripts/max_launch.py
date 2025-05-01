#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
import subprocess

class RosoutMonitor(Node):
    def __init__(self):
        super().__init__('rosout_monitor')
        self.sub = self.create_subscription(Log, '/rosout', self.cb, 10)
        self._launched_nav2 = False
        self._launched_loc = False
        self._launched_ini_pose = False
        self._launched_nav_to_pose = False

    def cb(self, msg: Log):
        if not self._launched_nav2 and ('Activating controllers: [ diff_cont ]' in msg.msg or 'Activating controllers: [ joint_broad ]' in msg.msg):
            self.get_logger().info('Activation détectée : lancement de Nav2')
            subprocess.Popen([
                'ros2', 'launch', 'robot_creation', 'navigation_launch.py',
                'use_sim_time:=true', 'map_subscribe_transient_local:=true'
            ])
            self._launched_nav2 = True
        if not self._launched_loc and 'Timed out waiting for transform from base_link to map to become available' in msg.msg:
            self.get_logger().info('Activation détectée : lancement de localisation')
            subprocess.Popen([
                'ros2', 'launch', 'robot_creation', 'localization_launch.py',
                'use_sim_time:=true', 'map:=./my_map_save.yaml'
            ])
            self._launched_loc = True
        if not self._launched_ini_pose and 'Please set the initial pose...' in msg.msg:
            self.get_logger().info('Activation détectée : lancement de la position initiale')
            subprocess.Popen([
                'ros2', 'run', 'robot_creation', 'initial_pose_publisher.py'
            ])
            self._launched_ini_pose = True
        if not self._launched_nav_to_pose and 'Creating bond timer...' in msg.msg:
            self.get_logger().info('Activation détectée : lancement de la position initiale')
            subprocess.Popen([
                'ros2', 'run', 'robot_creation', 'example_nav_to_pose.py'
            ])
            self._launched_nav_to_pose = True
        #ros2 run robot_creation example_nav_to_pose.py

def main(args=None):
    rclpy.init(args=args)

    # 1) Démarrage de la simulation en tâche de fond
    sim_proc = subprocess.Popen([
        'ros2', 'launch', 'robot_creation', 'launch_sim.launch.py'
    ])
    # ros2 run rviz2 rviz2 --ros-args --log-level error

    sim_proc = subprocess.Popen([
        'ros2', 'run', 'rviz2', 'rviz2', '--ros-args', '--log-level', 'error'
    ])

    # 2) Démarrage du nœud de surveillance
    monitor = RosoutMonitor()
    rclpy.spin(monitor)

    # 3) Nettoyage
    sim_proc.wait()
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
