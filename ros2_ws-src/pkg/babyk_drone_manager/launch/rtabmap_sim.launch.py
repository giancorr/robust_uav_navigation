import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    parameters = [{
        'frame_id': 'base_link',
        'map_frame_id': 'drone/map',
        'odom_frame_id': 'odom',
        #'database_path': '/root/ros2_ws/src/pkg/trajectory_planner/config/rtabmap.db',  # rtabmap database
        'subscribe_depth': True,
        'subscribe_odom_info': False,  
        'approx_sync': True,
        'qos_image': 2,
        'qos': 2,
        # NOTE: For standard PX4 firmware, use '/model/x500_0/odometry' and 'x500_0/OakD-Lite/base_link/IMX214'
        'odom_topic':'/model/baby_k_0/odometry',  # For standard PX4: use '/model/x500_0/odometry'
        'rgbd_cameras': 1,
        'rgbd_camera_frame_id': 'baby_k_0/OakD-Lite/base_link/IMX214',  # For standard PX4: use 'x500_0/OakD-Lite/base_link/IMX214'
        'wait_for_transform': 10.5,
        'Mem/IncrementalMemory': 'true',  # Set to false for localization mode
        'Mem/InitWMWithAllNodes': 'true',  
        #'sync_queue_size': 10,  # Adjusted for simulation
        #'topic_queue_size': 10,  # Adjusted for simulation
        'publish_tf': True,  # CRITICAL: Force rtabmap to publish map->odom TF
        'publish_tf_odom': False,  # Don't conflict with move_manager odometry
        #'publish_null_when_lost': True,  # Keep publishing tf even when lost
        # 'Reg/Force3DoF': 'true',  # Enable 2D SLAM for better initial mapping
        # 'Grid/FromDepth': 'false',  # Use 2D occupancy grid
        #'Mem/NotLinkedNodesKept': 'false',  # Don't keep unlinked nodes
        'Grid/Sensor': '1',  # Enable occupancy grid from depth
        'Grid/FromDepth': 'true',  # Generate grid from depth
        'Grid/3D': 'true',  # 3D occupancy grid (octomap)
        'Grid/MaxGroundHeight': '0.0',  # Ground level
        'Grid/MaxObstacleHeight': '2.0',  # Maximum obstacle height
        'use_sim_time': True,
    }]

    remappings=[
          ('rgb/image', '/camera'),
          ('rgb/camera_info', '/camera_info'),
          ('depth/image', '/depth_camera'),
          ('odom', '/odometry/filtered')]  # Remap rtabmap odometry to /odometry/filtered

    return LaunchDescription([


        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings),

        # Point cloud to obstacle cloud converter for local planner
        Node(
            package='rtabmap_util', executable='point_cloud_assembler', output='screen',
            parameters=[{
                'use_sim_time': True,
                'max_clouds': 10,
                'fixed_frame_id': 'drone/map'
            }],
            remappings=[
                ('cloud', '/rtabmap/cloud_map'),
                ('assembled_cloud', '/obstacle_cloud')
            ]),

        # RTABMap visualization node for octomap publishing - DISABLED to avoid GUI
        # Node(
        #     package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #     parameters=[{
        #         'use_sim_time': True,
        #         'subscribe_odom_info': False,
        #         'subscribe_scan_cloud': False,
        #         'approx_sync': True,
        #     }],
        #     remappings=remappings),

    ])
