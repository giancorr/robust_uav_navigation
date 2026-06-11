import copy
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import sys
import pathlib
import os
import yaml
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))

def generate_launch_description():

    return LaunchDescription([

        launch_ros.actions.Node(
            package='drone_odometry',
            executable='px4_tf_pub',
            name='px4_tf_pub',
            output='screen',
            parameters=[
                {
                    'px4_odom_frame_id': "odom", #frame of published tf and odom msg
                    'publish_tf': False,
                    'feed_twist_to_px4': True,
                    'odom_parent_is_not_odom': False, #Gazebo odometry should be in correct frame
                    'odom_child_is_not_base_link': False, #Gazebo uses base_link as child
                    'use_sim_time': True,
                }
            ],
            remappings=[
                ('/odometry/filtered', '/model/baby_k_downcam_0/odometry')  # Use Gazebo odometry directly in simulation
            ],
            arguments=['--log-level', 'info']
        )  

    ])
