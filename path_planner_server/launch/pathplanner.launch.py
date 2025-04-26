import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('path_planner_server')
    
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
    rviz_config = os.path.join(pkg_dir, 'rviz', 'pathplanning.rviz')
    
    # Behavior Tree configuration
    bt_xml_path = os.path.join(config_dir, 'navigate_w_replanning_and_recovery.xml')
    
    # For simulation
    bt_nav_sim_config = os.path.join(config_dir, 'bt_navigator_sim.yaml')
    controller_sim_config = os.path.join(config_dir, 'controller_sim.yaml')
    planner_sim_config = os.path.join(config_dir, 'planner_sim.yaml')
    recoveries_sim_config = os.path.join(config_dir, 'recoveries_sim.yaml')
    
    # For real robot
    bt_nav_real_config = os.path.join(config_dir, 'bt_navigator_real.yaml')
    controller_real_config = os.path.join(config_dir, 'controller_real.yaml')
    planner_real_config = os.path.join(config_dir, 'planner_real.yaml')
    recoveries_real_config = os.path.join(config_dir, 'recoveries_real.yaml')
    
    # Simulation nodes
    bt_navigator_node_sim = Node(
        condition=IfCondition(use_sim_time),
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            bt_nav_sim_config,
            {'default_bt_xml_filename': bt_xml_path},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    planner_server_node_sim = Node(
        condition=IfCondition(use_sim_time),
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            planner_sim_config,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    controller_server_node_sim = Node(
        condition=IfCondition(use_sim_time),
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            controller_sim_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')
        ]
    )
    
    recoveries_server_node_sim = Node(
        condition=IfCondition(use_sim_time),
        package='nav2_behaviors',
        executable='behavior_server',
        name='recoveries_server',
        output='screen',
        parameters=[
            recoveries_sim_config,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    lifecycle_manager_node_sim = Node(
        condition=IfCondition(use_sim_time),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_pathplanner',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': [
                'planner_server',
                'controller_server',
                'recoveries_server',
                'bt_navigator'
            ]}
        ]
    )
    
    rviz_node_sim = Node(
        condition=IfCondition(use_sim_time),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Real robot nodes
    bt_navigator_node_real = Node(
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            bt_nav_real_config,
            {'default_bt_xml_filename': bt_xml_path},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    planner_server_node_real = Node(
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            planner_real_config,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    controller_server_node_real = Node(
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            controller_real_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('cmd_vel', '/robot/cmd_vel')
        ]
    )
    
    recoveries_server_node_real = Node(
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        package='nav2_behaviors',
        executable='behavior_server',
        name='recoveries_server',
        output='screen',
        parameters=[
            recoveries_real_config,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    lifecycle_manager_node_real = Node(
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_pathplanner',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': [
                'planner_server',
                'controller_server',
                'recoveries_server',
                'bt_navigator'
            ]}
        ]
    )
    
    rviz_node_real = Node(
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Create and return launch description
    return LaunchDescription([
        use_sim_time_arg,
        
        # Simulation nodes
        bt_navigator_node_sim,
        planner_server_node_sim,
        controller_server_node_sim,
        recoveries_server_node_sim,
        lifecycle_manager_node_sim,
        rviz_node_sim,
        
        # Real robot nodes
        bt_navigator_node_real,
        planner_server_node_real,
        controller_server_node_real,
        recoveries_server_node_real,
        lifecycle_manager_node_real,
        rviz_node_real
    ])