#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Enable verbose output'
    )
    
    takeoff_altitude_arg = DeclareLaunchArgument(
        'takeoff_altitude',
        default_value='1.5',
        description='Takeoff altitude in meters'
    )
    
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Enable simulation mode with TF publishing'
    )

    # Include path planner launch
    path_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('path_planner'),
                'launch',
                'path_planner.launch.py'
            ])
        ]),
        launch_arguments={
            'verbose': LaunchConfiguration('verbose')
        }.items()
    )

    # Include move manager launch
    move_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('babyk_drone_manager'),
                'launch',
                'move_manager.launch.py'
            ])
        ]),
        launch_arguments={
            'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
            'simulation': LaunchConfiguration('simulation')
        }.items()
    )

    return LaunchDescription([
        verbose_arg,
        takeoff_altitude_arg,
        simulation_arg,
        path_planner_launch,
        move_manager_launch
    ])
