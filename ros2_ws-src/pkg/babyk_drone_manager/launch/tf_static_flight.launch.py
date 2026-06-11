#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Real arena bounds: x=[-1.0, 4.0], y=[-3.0, 3.0], z=[1.2, 2.0]
    # Goals positioned strategically within the arena at height 1.5m
    
    tf_nodes = []
    
    # Goal positions for real arena (5x6 meters)
    goal_positions = [
        # Front row (y = 2.0)
        # {"name": "goal1", "x": 0.0, "y": 2.0, "z": 1.5},   # Center front
        # {"name": "goal2", "x": 2.5, "y": 2.0, "z": 1.5},   # Right front
        # {"name": "goal3", "x": -0.5, "y": 2.0, "z": 1.5},  # Left front
        
        # # Middle row (y = 0.0)
        # {"name": "goal4", "x": 3.0, "y": 0.0, "z": 1.5},   # Far right center
        # {"name": "goal5", "x": 1.5, "y": 0.0, "z": 1.5},   # Right center
        
        # # Back row (y = -2.0)
        # {"name": "goal6", "x": 0.5, "y": -2.0, "z": 1.5},  # Center back
        # {"name": "goal7", "x": 2.0, "y": -2.0, "z": 1.5},  # Right back

        {"name": "exp00", "x": 1.0, "y": 1.0, "z": 1.5},   # Center front
        {"name": "exp01", "x": 1.0, "y": 5.0, "z": 1.5},   # Right front
        {"name": "exp10", "x": 4.0, "y": 1.0, "z": 1.5},  # Left front
        {"name": "exp11", "x": 4.0, "y": 5.0, "z": 1.5},   # Far right center
        {"name": "expcen", "x": 3.0, "y": 3.0, "z": 1.5},   # Right center
    ]
    
    for goal in goal_positions:
        tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_tf_pub_{goal["name"]}',
            arguments=[
                str(goal["x"]), str(goal["y"]), str(goal["z"]),  # translation
                '0', '0', '0', '1',  # quaternion (no rotation)
                'map',  # parent frame
                goal["name"]  # child frame
            ],
            parameters=[{'use_sim_time': use_sim_time}]
        )
        tf_nodes.append(tf_node)

    # Add static transform between map and drone/map
    drone_map_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_drone_map',
        arguments=[
            '0', '3', '0',    # translation
            '0', '0', '0', '1',  # quaternion (no rotation)
            'map',            # parent frame
            'drone/map'       # child frame
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    tf_nodes.append(drone_map_tf_node)
    
    return LaunchDescription([
        use_sim_time_arg,
        *tf_nodes
    ])
