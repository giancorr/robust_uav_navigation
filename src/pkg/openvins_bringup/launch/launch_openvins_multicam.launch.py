import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    bringup_dir = get_package_share_directory('openvins_bringup')

    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=os.path.join(bringup_dir, 'config', 'estimator_config_multicam.yaml'),
        description='Path to the OpenVINS estimator config YAML file'
    )
    
    multicam_node = Node(
        package='ov_msckf',
        executable='run_subscribe_msckf',
        name='ov_msckf',
        output='screen',
        parameters=[
            {"verbosity": "INFO"},
            {"use_stereo": False},
            {"max_cameras": 2},
            {"config_path": LaunchConfiguration('config_path')},
        ],
        remappings=[
            # Camera topics: only fisheye1 for both cameras
            ('/t265/fisheye1/image_raw',   '/cam0/fisheye1/image_raw'),
            ('/t265_1/fisheye1/image_raw', '/cam1/fisheye1/image_raw'),
            # IMU: use front camera (cam0) as primary IMU
            ('/t265/imu',                  '/cam0/imu'),
            # Output odometry topic
            ('odomimu',                    '/ov_msckf/odomimu'),
        ]
    )

    return LaunchDescription([config_path_arg, multicam_node])
