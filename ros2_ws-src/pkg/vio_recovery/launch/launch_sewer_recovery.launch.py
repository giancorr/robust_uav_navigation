import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    config_file = '/root/ros2_ws/src/pkg/vio_recovery/config/params_sewer.yaml'

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
            executable='sewer_recovery_fsm',
            name='sewer_recovery_fsm_node',
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

        # flight_odometry_filter: sole publisher on /fmu/in/vehicle_visual_odometry.
        # In simulation, px4_tf_pub runs with relay_odometry:=false (set in sewer_exploration.yml)
        # so this node is the only one feeding VIO/odometry to PX4 EKF2.
        Node(
            package='vio_recovery',
            executable='flight_odometry_filter',
            name='flight_odometry_filter_node',
            output='screen',
            parameters=[config_file, {'use_sim_time': True, 'enable_tactile_odometry': False}]
        ),

        # Feature counter
        Node(
            package='vio_recovery',
            executable='feature_counter_node',
            name='feature_counter_node',
            output='screen',
            parameters=[config_file]
        ),

        # Swipe spawner
        Node(
            package='vio_recovery',
            executable='swipe_spawner',
            name='swipe_spawner',
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

        # Logger
        Node(
            package='vio_recovery',
            executable='flight_data_logger',
            name='flight_data_logger',
            output='screen',
            cwd=log_path
        )
    ])
