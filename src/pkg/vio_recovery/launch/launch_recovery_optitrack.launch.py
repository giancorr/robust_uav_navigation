import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/back/odomimu',
        description='Odometry topic for monitoring and logging'
    )
    odom_topic = LaunchConfiguration('odom_topic')
    
    config_file = '/root/ros2_ws/src/pkg/vio_recovery/config/params.yaml'

    pkg_path = os.path.expanduser('~/ros2_ws/src/pkg/vio_recovery')
    log_path = os.path.join(pkg_path, 'flight_logs')

    if not os.path.exists(log_path):
        os.makedirs(log_path)

    return LaunchDescription([
        use_sim_time_arg,
        odom_topic_arg,
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value='/root/ros2_ws/src/pkg/vio_recovery/models'
        ),
        
        Node(
            package='vio_recovery',
            executable='degeneracy_monitor',
            name='degeneracy_monitor_node',
            output='screen',
            parameters=[config_file]
        ),
        
        Node(
            package='vio_recovery',
            executable='vio_recovery_fsm',
            name='vio_recovery_fsm_node',
            output='screen',
            parameters=[config_file, {'is_downcam': True}]
        ),
        
        Node(
            package='vio_recovery',
            executable='vio_recovery_controller',
            name='vio_recovery_controller_node',
            output='screen',
            parameters=[config_file]
        ),
        
        # Down spawner / Box dropper
        Node(
            package='vio_recovery',
            executable='box_dropper_node',
            name='box_dropper_node',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}]
        ),

        # Wrench estimator
        Node(
            package='vio_recovery',
            executable='wrench_estimator_node',
            name='wrench_estimator_node',
            output='screen',
            parameters=[config_file]
        ),

        # Spray heuristic
        Node(
            package='vio_recovery',
            executable='spray_heuristic_node',
            name='spray_heuristic_node',
            output='screen',
            parameters=[config_file],
            remappings=[('/back/odomimu', odom_topic)]
        ),

        # Feature counter
        Node(
            package='vio_recovery',
            executable='feature_counter_node',
            name='feature_counter_node',
            output='screen',
            parameters=[config_file]
        ),

        # Logger
        #Node(
        #    package='vio_recovery',
        #    executable='flight_data_logger',
        #    name='flight_data_logger',
        #    output='screen',
        #    cwd=log_path,
        #    remappings=[('/back/odomimu', odom_topic)]
        #)
    ])
