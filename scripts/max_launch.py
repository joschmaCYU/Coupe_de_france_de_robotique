#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
import subprocess
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import time

import os
import signal
import sys

class RosoutMonitor(Node):
    def __init__(self):
        super().__init__('rosout_monitor')
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        self.sub = self.create_subscription(Log, '/rosout', self.cb, qos_profile)
        self._launched_nav2 = False
        self._launched_loc = False
        self._launched_ini_pose = False
        self._launched_nav_to_pose = False
        self._ini_pose_count = 0   # new: counter for initial pose message occurrences
        self._tf_timeout_timestamp = None

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
                'use_sim_time:=true', 'map:=./src/robot_creation/src/my_map/my_map_save.yaml'
            ])
            self._launched_loc = True
        # Updated block to retry initial pose publisher
        if 'Please set the initial pose...' in msg.msg:
            if self._ini_pose_count == 0:
                # if blue
                subprocess.Popen([
                        'ros2', 'run', 'robot_creation', 'initial_pose_publisher.py'
                ])
                self._launched_ini_pose = True

            self._ini_pose_count += 1
            if self._ini_pose_count > 5:
                self.get_logger().info('Activation détectée : lancement/retry de la position initiale (attempt count: {})'.format(self._ini_pose_count))
                subprocess.Popen([
                    'ros2', 'run', 'robot_creation', 'initial_pose_publisher.py'
                ])
                self._launched_ini_pose = True
                self._ini_pose_count = 0
        if not self._launched_nav_to_pose and self._launched_ini_pose:
            if 'Creating bond timer...' in msg.msg:
                self.get_logger().info('Activation détectée : va !')
                
                subprocess.Popen([
                    'ros2', 'run', 'robot_creation', 'robot_manager'
                ])

                subprocess.Popen([
                    'ros2', 'run', 'robot_creation', 'cylinder_detector'
                ])

                subprocess.Popen([
                    'ros2', 'run', 'robot_creation', 'arduino_serial_node_write'
                ])

                # subprocess.Popen([
                #     'ros2', 'run', 'robot_creation', 'multi_pose_navigator'
                # ])
                self._launched_nav_to_pose = True
        if 'Timed out waiting for transform from base_link to map to become available, tf error: Invalid frame ID "map" passed to canTransform argument target_frame - frame does not exist' in msg.msg:
            if self._tf_timeout_timestamp is None:
                self._tf_timeout_timestamp = time.time()
            else:
                if (time.time() - self._tf_timeout_timestamp) >= 60:
                    self.get_logger().info('same -> killing everything and relaunching')
                    os.killpg(os.getpgid(os.getpid()), signal.SIGINT)

                    subprocess.Popen(['ros2', 'run', 'robot_creation', 'max_launch.py'])
                    self._tf_timeout_timestamp = time.time()
            
        # if self._launched_nav_to_pose and ("Navigation through poses succeeded!" in msg.msg or "Goal succeeded" in msg.msg):
        #     self.get_logger().info('Next pose !')
        #     subprocess.Popen([
        #             'ros2', 'run', 'robot_creation', 'example_nav_through_poses.py'
        #     ])
            

# [controller_server-1] [INFO] [1746705727.939243091] [controller_server]: Reached the goal!
# [controller_server-1] [INFO] [1746705727.941357204] [controller_server]: Optimizer reset
# [INFO] [1746705727.962726469] [multi_pose_navigator]: Navigation through poses succeeded!
# [bt_navigator-5] [INFO] [1746705727.962573560] [bt_navigator]: Goal succeeded


def main(args=None):
    rclpy.init(args=args)
    # TODO find some way to pass the argument to launch_sim
    # 1) Démarrage de la simulation en tâche de fond
    sim_proc = subprocess.Popen([
        'ros2', 'launch', 'robot_creation', 'launch_sim.launch.py'
    ])
    # ros2 run rviz2 rviz2 --ros-args --log-level error

    sim_proc = subprocess.Popen([
        'ros2', 'run', 'rviz2', 'rviz2', '--ros-args', '--log-level', 'error'
    ])

    subprocess.Popen([
        'ros2', 'run', 'robot_creation', 'arduino_serial_node_read'
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
