from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    input_topic_arg = DeclareLaunchArgument(
        'input_odom_topic',
        default_value='/back/odomimu',
        description='OpenVINS odometry topic: /back/odomimu (back-only) or /ov_msckf/odomimu (multicam)'
    )

    return LaunchDescription([
        input_topic_arg,

        # 1. Direct Converter: OpenVINS FLU odometry → base_link ENU
        Node(
            package='odometry_tracker',
            executable='odom_to_baselink_enu_direct',
            name='odom_to_baselink_enu_direct',
            output='screen',
            remappings=[
                ('/back/odomimu', LaunchConfiguration('input_odom_topic'))
            ]
        ),
        
        # 2. Path Publisher (for visualization)
        Node(
            package='odometry_tracker',
            executable='base_link_path_pub',
            name='base_link_path_pub',
            output='screen',
            parameters=[{
                'odom_topic': '/odometry/filtered'
            }]
        ),
        
        # 3. PX4 TF Publisher & VIO Bridge
        # This node takes /odometry/filtered (in ENU) and relays it to /fmu/in/vehicle_visual_odometry (in NED)
        Node(
            package='drone_odometry',
            executable='px4_tf_pub',
            name='px4_tf_pub',
            output='screen',
            parameters=[
                {'px4_odom_frame_id': 'odom'},
                {'vio_desired_parent_frame_id': 'odom'},
                {'publish_tf': False}, # odom_to_baselink_enu_direct already publishes odom -> base_link!
                {'is_already_ned': False}
            ]
        )
    ])
