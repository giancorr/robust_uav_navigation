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
            executable='mocap2_driver',
            name='mocap2_driver',
            output='screen',
            parameters=[
                {   
                    'connection_type': "Unicast", # Unicast / Multicast
                    'server_address': "192.168.1.2",
                    'local_address': "192.168.1.99",
                    'multicast_address': "239.255.42.99",
                    'server_command_port': 1510,
                    'server_data_port': 1511,

                    'odom_frame_id': "optitrack_link",
                    'map_frame_id': "optitrack_odom",
                    'publish_tf': False,

                    # Lista degli ID dei corpi rigidi da pubblicare
                    # Ogni corpo avrà un topic separato: /optitrack/body_<ID>/odometry
                    'rigid_body_ids': [0, 1, 2, 3],   # <-- modifica con i tuoi ID
                }
            ]
        )

    ])