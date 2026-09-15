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
            package='optitrack_listener2',
            executable='optitrack_listener',
            name='optitrack_listener',
            output='screen',
            parameters=[
                {
                    'odom_frame_id': "optitrack_link",
                    'map_frame_id': "optitrack_odom",
                    'publish_tf': False,
                }
            ]
        )

    ])