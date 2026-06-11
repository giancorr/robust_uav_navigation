import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    config_file = '/root/ros2_ws/src/pkg/vio_health_monitor/config/params.yaml'

    # 1. Setup per il salvataggio dei file di log
    pkg_path = os.path.expanduser('~/ros2_ws/src/pkg/vio_health_monitor')
    log_path = os.path.join(pkg_path, 'flight_logs')

    # Crea la cartella se non esiste
    if not os.path.exists(log_path):
        os.makedirs(log_path)

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value='/root/ros2_ws/src/pkg/vio_health_monitor/models'
        ),
        
        Node(
            package='vio_health_monitor',
            executable='degeneracy_monitor',
            name='degeneracy_monitor_node',
            output='screen',
            parameters=[config_file]
        ),
        
        Node(
            package='vio_health_monitor',
            executable='emergency_rescue_old',
            name='emergency_rescue_old_node',
            output='screen',
            parameters=[config_file]
        ),
        
        Node(
            package='vio_health_monitor',
            executable='surface_detector_node',
            name='surface_detector_node',
            output='screen',
            parameters=[config_file]
        ),
        
        Node(
            package='vio_health_monitor',
            executable='tactile_odometry',
            name='tactile_odometry_node',
            output='screen',
            parameters=[config_file, {'use_sim_time': True}]
        ),
        
        Node(
            package='vio_health_monitor',
            executable='aruco_spawner',
            name='aruco_spawner_node',
            output='screen',
            parameters=[config_file]
        ),

        Node(
            package='vio_health_monitor',
            executable='flight_data_logger',
            name='flight_data_logger',
            output='screen',
            cwd=log_path
        )
    ])