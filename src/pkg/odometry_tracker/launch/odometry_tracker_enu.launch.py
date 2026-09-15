from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    remap_arg = DeclareLaunchArgument(
        'remap_back_to_front',
        default_value='false',
        description='Remap /back/base_link_odom to /front/base_link_odom for single back camera mode'
    )
    
    # EKF parameters file path
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("openvins_bringup"), "config", "ekf.yaml"]
    )

    return LaunchDescription([
        remap_arg,

        # 1a. Converter (Remapped mode for single back camera)
        Node(
            package='odometry_tracker',
            executable='odom_to_baselink_enu',
            name='odom_to_baselink_enu',
            output='screen',
            condition=IfCondition(LaunchConfiguration('remap_back_to_front')),
            remappings=[('/back/base_link_odom', '/front/base_link_odom')]
        ),

        # 1b. Converter (Standard mode for dual cameras)
        Node(
            package='odometry_tracker',
            executable='odom_to_baselink_enu',
            name='odom_to_baselink_enu',
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('remap_back_to_front'))
        ),
        
        # 2. EKF Node from robot_localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        ),
        
        # 3. Path Publisher (for visualization)
        Node(
            package='odometry_tracker',
            executable='base_link_path_pub',
            name='base_link_path_pub',
            output='screen',
            parameters=[{
                'odom_topic': '/odometry/filtered'
            }]
        ),
        
        # 4. PX4 TF Publisher & VIO Bridge
        # This node takes /odometry/filtered (in ENU) and relays it to /fmu/in/vehicle_visual_odometry (in NED)
        Node(
            package='drone_odometry',
            executable='px4_tf_pub',
            name='px4_tf_pub',
            output='screen',
            parameters=[
                {'px4_odom_frame_id': 'odom'},
                {'vio_desired_parent_frame_id': 'odom'},
                {'publish_tf': False}, # EKF already publishes odom -> base_link
                {'odom_child_is_not_base_link': False}
            ]
        )
    ])
