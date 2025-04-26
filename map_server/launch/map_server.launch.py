import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    
    # Declare use_sim_time parameter
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation clock if true'
    )
    
    # Configuration file paths - using PathJoinSubstitution instead of os.path.join
    config_dir = os.path.join(pkg_dir, 'config')
    map_yaml_file = PathJoinSubstitution(
        [FindPackageShare('map_server'), 'config', map_file]
    )
    
    # RViz configuration
    rviz_config = os.path.join(pkg_dir, 'rviz', 'map_display.rviz')
    
    # Nodes to run
    # For simulation environment
    map_server_node_sim = Node(
        condition=IfCondition(use_sim_time),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml_file},
            {'topic_name': 'map'},
            {'frame_id': 'map'},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # For real robot environment - using robot_map frame
    map_server_node_real = Node(
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml_file},
            {'topic_name': 'map'},
            {'frame_id': 'robot_map'},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Lifecycle manager for simulation
    lifecycle_manager_node_sim = Node(
        condition=IfCondition(use_sim_time),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )
    
    # Lifecycle manager for real robot
    lifecycle_manager_node_real = Node(
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )
    
    # RViz node with different fixed frame based on environment
    rviz_node_sim = Node(
        condition=IfCondition(use_sim_time),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    rviz_node_real = Node(
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'fixed_frame': 'robot_map'}
        ],
        output='screen'
    )
    
    # Add a delay to ensure the map server is fully active before RViz starts
    delayed_rviz_sim = TimerAction(
        condition=IfCondition(use_sim_time),
        period=3.0,
        actions=[rviz_node_sim]
    )
    
    delayed_rviz_real = TimerAction(
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        period=3.0,
        actions=[rviz_node_real]
    )
    
    # Create and return launch description
    return LaunchDescription([
        map_file_arg,
        use_sim_time_arg,
        map_server_node_sim,
        map_server_node_real,
        lifecycle_manager_node_sim,
        lifecycle_manager_node_real,
        delayed_rviz_sim,
        delayed_rviz_real
    ])