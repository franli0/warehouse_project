import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

    map_yaml_file = PathJoinSubstitution(
        [FindPackageShare('map_server'), 'config', map_file]
    )
    
    # RViz configuration
    rviz_config = os.path.join(pkg_dir, 'rviz', 'localization.rviz')

    with open(rviz_config, 'r') as f:
        template_content = f.read()

    # Create simulation config
    sim_rviz_content = template_content.replace('{fixed_frame}', 'map')
    sim_rviz_path = os.path.join(tempfile.gettempdir(), 'localization_sim.rviz')
    with open(sim_rviz_path, 'w') as f:
        f.write(sim_rviz_content)
    
    # Create real robot config
    real_rviz_content = template_content.replace('{fixed_frame}', 'robot_map')
    real_rviz_path = os.path.join(tempfile.gettempdir(), 'localization_real.rviz')
    with open(real_rviz_path, 'w') as f:
        f.write(real_rviz_content)
    
    # Load AMCL configuration files
    amcl_config_sim = os.path.join(config_dir, 'amcl_config_sim.yaml')
    amcl_config_real = os.path.join(config_dir, 'amcl_config_real.yaml')
    
    # Map Server Node - For Simulation
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

    # Map Server Node - For Real Robot
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

    # Lifecycle Manager for simulation
    lifecycle_manager_node_sim = Node(
        condition=IfCondition(use_sim_time),
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

    # Lifecycle manager for real robot
    lifecycle_manager_node_real = Node(
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
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
    
    # RViz node with different fixed frame based on environment
    rviz_node_sim = Node(
        condition=IfCondition(use_sim_time),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', sim_rviz_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    rviz_node_real = Node(
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', real_rviz_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Add a delay to ensure the map server is fully active before RViz starts
    delayed_rviz_sim = TimerAction(
        period=2.0,
        actions=[rviz_node_sim],
        condition=IfCondition(use_sim_time)
    )
    
    delayed_rviz_real = TimerAction(
        period=2.0,
        actions=[rviz_node_real],
        condition=IfCondition(PythonExpression(['not ', use_sim_time]))
    )
    
    # Create and return launch description
    return LaunchDescription([
        map_file_arg,
        use_sim_time_arg,
        map_server_node_sim,
        map_server_node_real,
        amcl_node_sim,
        amcl_node_real,
        lifecycle_manager_node_sim,
        lifecycle_manager_node_real,
        delayed_rviz_sim,
        delayed_rviz_real
    ])