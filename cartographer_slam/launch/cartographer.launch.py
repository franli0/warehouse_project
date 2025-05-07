import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('cartographer_slam')
    
    # Declare use_sim_time parameter
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation clock if true'
    )
    
    # Configuration files
    config_dir = os.path.join(pkg_dir, 'config')
    
    # RViz configuration
    rviz_config = os.path.join(pkg_dir, 'rviz', 'mapping.rviz')
    
    with open(rviz_config, 'r') as f:
        template_content = f.read()
    
    # Create simulation config
    sim_rviz_content = template_content.replace('{fixed_frame}', 'map')
    sim_rviz_path = os.path.join(tempfile.gettempdir(), 'mapping_sim.rviz')
    with open(sim_rviz_path, 'w') as f:
        f.write(sim_rviz_content)
    
    # Create real robot config
    real_rviz_content = template_content.replace('{fixed_frame}', 'robot_map')
    real_rviz_path = os.path.join(tempfile.gettempdir(), 'mapping_real.rviz')
    with open(real_rviz_path, 'w') as f:
        f.write(real_rviz_content)
    
    # Determine which configuration file to use based on use_sim_time
    # For simulation environment
    cartographer_node_sim = Node(
        condition=IfCondition(use_sim_time),
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'cartographer_sim.lua'
        ],
        remappings=[
            ('scan', '/scan')
        ]
    )
    
    # For real robot environment
    cartographer_node_real = Node(
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'cartographer_real.lua'
        ],
        remappings=[
            ('scan', '/scan')
        ]
    )
    
    # Occupancy grid node for simulation environment
    occupancy_grid_node_sim = Node(
        condition=IfCondition(use_sim_time),
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        arguments=[
            '-resolution', '0.05'
        ]
    )
    
    # Occupancy grid node for real robot environment
    occupancy_grid_node_real = Node(
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        arguments=[
            '-resolution', '0.05'
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
        use_sim_time_arg,
        cartographer_node_sim,
        cartographer_node_real,
        occupancy_grid_node_sim,
        occupancy_grid_node_real,
        delayed_rviz_sim,
        delayed_rviz_real
    ])