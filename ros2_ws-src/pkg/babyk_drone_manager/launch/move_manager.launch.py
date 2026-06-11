#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('babyk_drone_manager'),
            'config',
            'move_manager_params.yaml'
        ]),
        description='Path to the move manager config file'
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

    # Move manager node
    move_manager_node = Node(
        package='babyk_drone_manager',
        executable='move_manager_node',
        name='move_manager_node',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
                'simulation': LaunchConfiguration('simulation'),
                'use_sim_time': True,
            }
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        config_file_arg,
        takeoff_altitude_arg,
        simulation_arg,
        move_manager_node
    ])
