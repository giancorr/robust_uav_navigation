import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    virtual_range = LaunchConfiguration('virtual_range', default='19.0')
    scenario = LaunchConfiguration('scenario')

    config_file = [
        os.path.join(get_package_share_directory('vio_mapping'), 'config', 'params_'),
        scenario,
        '.yaml'
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        DeclareLaunchArgument(
            'scenario',
            default_value='corridor',
            description='Environment scenario (corridor or sewer)'),
            
        DeclareLaunchArgument(
            'virtual_range',
            default_value='19.0',
            description='OVDE virtual range'),

        Node(
            package='vio_mapping',
            executable='vio_wall_densifier_node',
            name='vio_wall_densifier',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                config_file
            ]
        ),

        Node(
            package='vio_mapping',
            executable='vio_mapping_node',
            name='vio_mapping',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'ovde.virtual_range': virtual_range},
                config_file
            ],
            remappings=[
                ('/ov_msckf/points_slam', '/ov_msckf/points_slam_densified')
            ]
        )
    ])
