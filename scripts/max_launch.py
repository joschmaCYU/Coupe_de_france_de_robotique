#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
import subprocess
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import time

import os
import signal


sim_proc = None
rviz_proc = None
serial_proc = None
nav_proc = None
loc_proc = None

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
        self.run_sub_processes()

    def run_sub_processes(self):
        global sim_proc, rviz_proc, serial_proc
        sim_proc = subprocess.Popen(['ros2', 'launch', 'robot_creation', 'launch_sim.launch.py'], 
                                preexec_fn=os.setsid)
        # ros2 run rviz2 rviz2 --ros-args --log-level error

        rviz_proc = subprocess.Popen(['ros2', 'run', 'rviz2', 'rviz2', '--ros-args', '--log-level', 'error'],
                                        preexec_fn=os.setsid)

        serial_proc = subprocess.Popen(['ros2', 'run', 'robot_creation', 'arduino_serial_node_read'],
                                    preexec_fn=os.setsid)
        
    def cb(self, msg: Log):
        global nav_proc, loc_proc
        if not self._launched_nav2 and ('Activating controllers: [ diff_cont ]' in msg.msg or 'Activating controllers: [ joint_broad ]' in msg.msg):
            self.get_logger().info('Activation détectée : lancement de Nav2')
            nav_proc = subprocess.Popen([
                'ros2', 'launch', 'robot_creation', 'navigation_launch_new.py',
                'use_sim_time:=true', 'map_subscribe_transient_local:=true'
            ])
            self._launched_nav2 = True
        if not self._launched_loc and 'Timed out waiting for transform from base_link to map to become available' in msg.msg:
            self.get_logger().info('Activation détectée : lancement de localisation')
            loc_proc = subprocess.Popen([
                'ros2', 'launch', 'robot_creation', 'localization_launch_new.py',
                'use_sim_time:=true', 'map:=./src/robot_creation/src/my_map/my_map_save.yaml'
            ])
            self._launched_loc = True
        # Updated block to retry initial pose publisher
        if 'Please set the initial pose...' in msg.msg:
            if self._ini_pose_count == 0:
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
                
                self._launched_nav_to_pose = True
        if 'Timed out waiting for transform from base_link to map to become available, tf error: Invalid frame ID "map" passed to canTransform argument target_frame - frame does not exist' in msg.msg:
            if self._tf_timeout_timestamp is None:
                self._tf_timeout_timestamp = time.time()
            else:
                if (time.time() - self._tf_timeout_timestamp) >= 40:
                    self.get_logger().info('same -> killing everything and relaunching')

                    os.killpg(sim_proc.pid, signal.SIGINT)
                    os.killpg(rviz_proc.pid, signal.SIGINT)
                    os.killpg(rviz_proc.pid, signal.SIGINT)

                    if nav_proc != None:
                        os.killpg(nav_proc.pid, signal.SIGINT)

                    if loc_proc != None:
                        os.killpg(loc_proc.pid, signal.SIGINT)

                    self.run_sub_processes()
                    self._tf_timeout_timestamp = time.time()
            

def main(args=None):
    rclpy.init(args=args)
    # TODO find some way to pass the argument to launch_sim

    monitor = RosoutMonitor()
    rclpy.spin(monitor)
    
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
