from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='true',
        description='Whether to use simulation time'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='config/autonomous_test_node_params.yaml',
        description='Path to config file relative to package share directory'
    )
    
    # Define config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('babyk_drone_manager'),
        LaunchConfiguration('config_file')
    ])
    
    # Create the autonomous test node
    autonomous_test_node = Node(
        package='babyk_drone_manager',
        executable='autonomous_test_node',
        name='autonomous_test_node',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('simulation')}
        ],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        simulation_arg,
        config_file_arg,
        autonomous_test_node
    ])