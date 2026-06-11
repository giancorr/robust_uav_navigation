import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['18', '5', ' 1.5', '0','0', '0', 'map', 'goal1']),
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0.5', '7', ' 1.5',  '0', '0', '0', 'map', 'goal2']),
            
    	Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['8', '5', ' 1.5',  '0', '0', '0', 'map', 'goal3']), 
            
    	Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['6.5', '9', ' 1.5',  '0', '0', '0', 'map', 'goal4']), 
    	Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['12', '7.5', ' 1.5',  '0', '0', '0', 'map', 'goal5']), 
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['16.5', '6.5', ' 1.5',  '0', '0', '0', 'map', 'goal6']), 
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['15', '2', ' 1.5',  '0', '0', '0', 'map', 'goal7']), 
        
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '1', ' 0.0',  '0', '0', '0', 'map', 'drone/map']), 
    ])