import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('map_server')
    
    # Declare map_file parameter
    map_file = LaunchConfiguration('map_file')
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_sim.yaml',
        description='Name of the map YAML file to load'
    )
    
    # Configuration file paths
    config_dir = os.path.join(pkg_dir, 'config')
    
    # RViz configuration
    rviz_config = os.path.join(pkg_dir, 'rviz', 'map_display.rviz')
    
    # Nodes to run
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': [config_dir, '/', map_file]},
            {'topic_name': 'map'},
            {'frame_id': 'map'},
            {'use_sim_time': True}
        ]
    )
    
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Create and return launch description
    return LaunchDescription([
        map_file_arg,
        map_server_node,
        lifecycle_manager_node,
        rviz_node
    ])