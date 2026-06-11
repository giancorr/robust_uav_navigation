import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    config_file = '/root/ros2_ws/src/pkg/vio_health_monitor/config/params.yaml'

    pkg_path = os.path.expanduser('~/ros2_ws/src/pkg/vio_health_monitor')
    log_path = os.path.join(pkg_path, 'flight_logs')

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
            executable='emergency_rescue',
            name='emergency_rescue_node',
            output='screen',
            parameters=[config_file, {'is_downcam': True}]
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
        
        # --- SPAWNER MACCHIE ---
        # Spawner laterale (per impatti sui muri)
        Node(
            package='vio_health_monitor',
            executable='aruco_spawner',
            name='aruco_spawner_node',
            output='screen',
            parameters=[config_file]
        ),

        # Spawner in basso (per atterraggi o drop sul pavimento)
        Node(
            package='vio_health_monitor',
            executable='aruco_spawner_down_node',
            name='aruco_spawner_down_node',
            output='screen',
            parameters=[config_file]
        ),

        # --- NUOVA ARCHITETTURA ---
        # Nodo matematico per la stima delle forze di contatto
        Node(
            package='vio_health_monitor',
            executable='wrench_estimator_node',
            name='wrench_estimator_node',
            output='screen',
            parameters=[config_file]
        ),

        # Nodo logico per decidere DOVE spruzzare
        Node(
            package='vio_health_monitor',
            executable='spray_heuristic_node',
            name='spray_heuristic_node',
            output='screen',
            parameters=[config_file]
        ),

        # Nodo che conta le feature dalle telecamere (Gli Occhi)
        Node(
            package='vio_health_monitor',
            executable='feature_counter_node',  # Assicurati che il nome dell'eseguibile sia questo
            name='feature_counter_node',
            output='screen',
            parameters=[config_file]
        ),

        # --- LOGGER ---
        Node(
            package='vio_health_monitor',
            executable='flight_data_logger',
            name='flight_data_logger',
            output='screen',
            cwd=log_path
        )
    ])
