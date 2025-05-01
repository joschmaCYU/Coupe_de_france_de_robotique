#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Get package share directories
    robot_creation_share = get_package_share_directory('robot_creation')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    
    # Include robot_creation launch_sim.launch.py
    launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_creation_share, 'launch', 'launch_sim.launch.py')
        )
    )

    # Include navigation_launch.py from nav2_bringup with use_sim_time parameter.
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py'),
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map_subscribe_transient_local': 'true',
        }.items()
    )
    
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': './my_map_save.yaml',
            'use_sim_time': 'true',
        }.items()
    )

    # Optionally, add timers to delay some launches if needed.
    # For example, if you want to delay launching localization by 5 seconds:
    localization_launch_delayed = TimerAction(period=20.0, actions=[localization_launch])
    navigation_launch_delayed = TimerAction(period=5.0, actions=[navigation_launch])
    
    script_path = os.path.join(robot_creation_share, 'scripts', 'initial_pose_publisher.py')
    initial_pose_publisher_exec = TimerAction(
        period=35.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', script_path],
                output='screen'
            )
        ]
    )
    
    ld = LaunchDescription()

    ld.add_action(launch_sim)
    ld.add_action(navigation_launch_delayed)
    ld.add_action(localization_launch_delayed)
    ld.add_action(initial_pose_publisher_exec)


    return ld
