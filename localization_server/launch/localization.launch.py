import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('localization_server')
    
    # Declare map_file parameter
    map_file = LaunchConfiguration('map_file')
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_sim.yaml',
        description='Name of the map YAML file to load'
    )
    
    # Declare use_sim_time parameter
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation clock if true'
    )
    
    # Configuration file paths
    map_server_config_dir = get_package_share_directory('map_server')
    map_config_dir = os.path.join(map_server_config_dir, 'config')
    config_dir = os.path.join(pkg_dir, 'config')
    
    # RViz configuration
    rviz_config = os.path.join(pkg_dir, 'rviz', 'localization.rviz')
    
    # Load AMCL configuration files
    amcl_config_sim = os.path.join(config_dir, 'amcl_config_sim.yaml')
    amcl_config_real = os.path.join(config_dir, 'amcl_config_real.yaml')
    
    # Map Server Node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': [map_config_dir, '/', map_file]},
            {'topic_name': 'map'},
            {'frame_id': 'map'},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # AMCL Node for simulation environment
    amcl_node_sim = Node(
        condition=IfCondition(use_sim_time),
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config_sim]
    )
    
    # AMCL Node for real environment
    amcl_node_real = Node(
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config_real]
    )
    
    # Lifecycle Manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Create and return launch description
    return LaunchDescription([
        map_file_arg,
        use_sim_time_arg,
        map_server_node,
        amcl_node_sim,
        amcl_node_real,
        lifecycle_manager_node,
        rviz_node
    ])