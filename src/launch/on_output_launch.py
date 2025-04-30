#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction
)
from launch.conditions import IfCondition
from launch.event_handlers import (
    OnExecutionComplete,
    OnProcessExit,
    OnProcessIO,
    OnProcessStart,
    OnShutdown
)
from launch.events import Shutdown
from launch.substitutions import (
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    LocalSubstitution,
    PythonExpression
)
from launch_ros.actions import Node


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )

    spawn_turtle = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 200',
                ' and ',
                use_provided_red
            ])
        ),
    )

    return LaunchDescription([
        turtlesim_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=turtlesim_node,
                on_start=[
                    LogInfo(msg='Turtlesim started, spawning turtle'),
                    spawn_turtle
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessIO(
                target_action=spawn_turtle,
                on_stdout=lambda event: LogInfo(
                    msg='Spawn request says "{}"'.format(
                        event.text.decode().strip())
                )
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=spawn_turtle,
                on_completion=[
                    LogInfo(msg='Spawn finished'),
                    change_background_r,
                    TimerAction(
                        period=2.0,
                        actions=[change_background_r_conditioned],
                    )
                ]
            )
        ),

    ])

# import os
# import re
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, RegisterEventHandler, LogInfo, ExecuteProcess, TimerAction
# from launch.event_handlers import OnProcessIO, OnProcessStart
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory

# from launch.events.process import matches_executable

# def generate_launch_description():
#     # --- Package Paths ---
#     robot_creation_share = get_package_share_directory('robot_creation')
#     nav2_bringup_share = get_package_share_directory('nav2_bringup')

#     # --- Define Launch Components ---
#     launch_sim = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(robot_creation_share, 'launch', 'launch_sim.launch.py')
#         )
#     )

#     sim_node = Node(
#         package='robot_creation',
#         executable='launch_sim.launch.py',  # Or decompose into individual nodes
#         output='screen'
#     )

#     navigation_node = Node(
#         package='nav2_bringup',
#         executable='navigation_launch.py',  # Or decompose into individual nodes
#         output='screen',
#         parameters=[os.path.join(robot_creation_share, 'config', 'nav2_params.yaml')]
#     )
    
#     # localization_launch = IncludeLaunchDescription(
#     #     PythonLaunchDescriptionSource(
#     #         os.path.join(robot_creation_share, 'launch', 'localization_launch.py')
#     #     ),
#     #     launch_arguments={
#     #         'map': './my_map_save.yaml',
#     #         'use_sim_time': 'true',
#     #         'params_file': os.path.join(robot_creation_share, 'config', 'nav2_params.yaml')
#     #     }.items()
#     # )

#     navigation_process = ExecuteProcess(
#         cmd=[
#             'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
#             'use_sim_time:=true',
#             'params_file:=' + os.path.join(robot_creation_share, 'config', 'nav2_params.yaml')
#         ],
#         output='screen',
#         name='navigation_launch'
#     )

#     localization_process = ExecuteProcess(
#         cmd=[
#             'ros2', 'launch', 'nav2_bringup', 'localization_launch.py',
#             'map:=' + os.path.join(robot_creation_share, 'maps', 'my_map_save.yaml'),
#             'use_sim_time:=true'
#         ],
#         output='screen',
#         name='localization_launch'
#     )
#     # [gazebo-3] [Dbg] [CameraSensor.cc:504] Enabling camera sensor: 'my_bot::base_link::camera' data generation.
#     # SIM_REGEX = r".*Enabling camera sensor: 'my_bot::base_link::camera' data generation.*"
#     SIM_REGEX =  r".*Enabling camera sensor: 'my_bot::base_link::camera' data generation."
    
#     def handle_sim_output(event):
#         text = re.sub(r'\x1B\[[0-?]*[ -/]*[@-~]', '', event.text.decode())
#         if re.search(SIM_REGEX, text):
#             return [
#                 LogInfo(msg="Navigation ready for camera, starting sensor..."),
#                 # navigation_node
#             ]
#         return []
        
    
#     nav_event_handler = RegisterEventHandler(
#         event_handler=OnProcessIO(
#             target_action=navigation_process,
#             on_stdout=handle_sim_output,
#             on_stderr=handle_sim_output
#         )
#     )

#     # Localization as a separate process
#     localization_process = ExecuteProcess(
#         cmd=[
#             'ros2', 'launch', 'robot_creation', 
#             'localization_launch.py',
#             'map:=' + os.path.join(robot_creation_share, 'maps', 'my_map_save.yaml'),
#             'use_sim_time:=true'
#         ],
#         output='screen'
#     )

#     NAV_REGEX = r"\[planner_server-\d+\] \[INFO\] \[.*\] \[global_costmap\.global_costmap\]: Timed out waiting for transform.*"

#     def handle_nav_output(event):
#         # Strip ANSI color codes before matching
#         text = re.sub(r'\x1B\[[0-?]*[ -/]*[@-~]', '', event.text.decode())
#         if re.search(NAV_REGEX, text):
#             return [
#                 LogInfo(msg="Camera sensor ready! Starting localization..."),
#                 localization_process
#             ]
#         return []

#     loc_event_handler = RegisterEventHandler(
#         event_handler=OnProcessIO(
#             target_action=localization_process,
#             on_stdout=handle_nav_output,
#             on_stderr=handle_nav_output
#         )
#     )

#     def handle_gazebo_output(event):
#         # Get raw output with ANSI codes
#         raw_text = event.text.decode(errors='replace')
        
#         # Log raw and cleaned versions
#         debug_actions = [
#             LogInfo(msg=f"RAW: {repr(raw_text)}"),
#             LogInfo(msg=f"CLEAN: {re.sub(r'\x1B\[[0-?]*[ -/]*[@-~]', '', raw_text)}")
#         ]
        
#         # Check for pattern in cleaned text
#         if "Enabling camera sensor" in raw_text:
#             debug_actions.append(LogInfo(msg="MATCH FOUND IN RAW TEXT"))
            
#         if re.search(r'CameraSensor.*data generation', raw_text):
#             debug_actions.append(LogInfo(msg="REGEX MATCHED RAW TEXT"))
        
#         # Your actual logic
#         if re.search(SIM_REGEX, raw_text):
#             debug_actions.extend([
#                 LogInfo(msg="Starting navigation..."),
#                 navigation_process
#             ])
        
#         return debug_actions
    
#     stdout_event_handler = RegisterEventHandler(
#         event_handler=OnProcessStart(
#             matcher=matches_executable("server"),
#             on_start=handle_gazebo_output
#         )
#     )

#     return LaunchDescription([
#         launch_sim,
#         stdout_event_handler,
#         nav_event_handler,
#         loc_event_handler
#     ])