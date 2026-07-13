import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    config_file = '/root/ros2_ws/src/pkg/vio_recovery/config/params.yaml'

    pkg_path = os.path.expanduser('~/ros2_ws/src/pkg/vio_recovery')
    log_path = os.path.join(pkg_path, 'flight_logs')

    if not os.path.exists(log_path):
        os.makedirs(log_path)

    return LaunchDescription([
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
        
        # Node(
        #     package='vio_recovery',
        #     executable='surface_detector_node',
        #     name='surface_detector_node',
        #     output='screen',
        #     parameters=[config_file]
        # ),
        
        # Node(
        #     package='vio_recovery',
        #     executable='tactile_odometry',
        #     name='tactile_odometry_node',
        #     output='screen',
        #     parameters=[config_file, {'use_sim_time': True}]
        # ),
        
        # Lateral spawner
        Node(
            package='vio_recovery',
            executable='swipe_spawner',
            name='swipe_spawner',
            output='screen',
            parameters=[config_file]
        ),

        # Down spawner (KEEP THIS!)
        Node(
            package='vio_recovery',
            executable='drop_spawner',
            name='drop_spawner',
            output='screen',
            parameters=[config_file]
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
            parameters=[config_file]
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
        Node(
            package='vio_recovery',
            executable='flight_data_logger',
            name='flight_data_logger',
            output='screen',
            cwd=log_path
        )
    ])
