import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
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

    # Configuration directory
    config_dir = os.path.join(pkg_dir, 'config')
    
    # Behavior Tree configuration
    bt_xml_path = os.path.join(config_dir, 'navigate_w_replanning_and_recovery.xml')
    
    # RViz configuration
    rviz_config = os.path.join(pkg_dir, 'rviz', 'pathplanning.rviz')
    
    def launch_setup(context):
        # Determine if simulation or real robot
        is_simulation = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
        
        # Set configuration paths and parameters based on mode
        if is_simulation:
            cmd_vel_topic = '/diffbot_base_controller/cmd_vel_unstamped'
            bt_nav_config = os.path.join(config_dir, 'bt_navigator_sim.yaml')
            controller_config = os.path.join(config_dir, 'controller_sim.yaml')
            planner_config = os.path.join(config_dir, 'planner_sim.yaml')
            recoveries_config = os.path.join(config_dir, 'recoveries_sim.yaml')
            filters_config = os.path.join(config_dir, 'filters_sim.yaml')
            fixed_frame = 'map'
            print("\n=== SIMULATION MODE ===")
            print(f"Command velocity topic: {cmd_vel_topic}")
            print(f"Fixed frame: {fixed_frame}\n")
        else:
            cmd_vel_topic = '/cmd_vel'
            bt_nav_config = os.path.join(config_dir, 'bt_navigator_real.yaml')
            controller_config = os.path.join(config_dir, 'controller_real.yaml')
            planner_config = os.path.join(config_dir, 'planner_real.yaml')
            recoveries_config = os.path.join(config_dir, 'recoveries_real.yaml')
            filters_config = os.path.join(config_dir, 'filters_real.yaml')
            fixed_frame = 'robot_map'
            print("\n=== REAL ROBOT MODE ===")
            print(f"Command velocity topic: {cmd_vel_topic}")
            print(f"Fixed frame: {fixed_frame}\n")
        
        # Create RViz config with correct fixed frame
        with open(rviz_config, 'r') as f:
            rviz_content = f.read()
        
        rviz_content_modified = rviz_content.replace('{fixed_frame}', fixed_frame)
        temp_rviz_path = f'/tmp/pathplanning_{fixed_frame}.rviz'
        with open(temp_rviz_path, 'w') as f:
            f.write(rviz_content_modified)
        
        # TF buffer server for real robot mode
        tf_buffer_fix_node = Node(
            condition=UnlessCondition(use_sim_time),
            package='tf2_ros',
            executable='buffer_server',
            name='tf_buffer_server',
            parameters=[
                {'use_sim_time': False},
                {'buffer_size': 120.0}
            ],
            output='screen'
        )
        
        # Behavior Tree Navigator
        bt_navigator_node = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                bt_nav_config,
                {'default_nav_to_pose_bt_xml': bt_xml_path},
                {'use_sim_time': is_simulation}
            ]
        )

        # Planner Server
        planner_server_node = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[
                planner_config,
                {'use_sim_time': is_simulation}
            ]
        )

        # Controller Server
        controller_server_node = Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[
                controller_config,
                {'use_sim_time': is_simulation}
            ],
            remappings=[
                ('cmd_vel', cmd_vel_topic)
            ]
        )

        # Recoveries Server
        recoveries_server_node = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            output='screen',
            parameters=[
                recoveries_config,
                {'use_sim_time': is_simulation}
            ]
        )

        # Filter Mask Server
        filter_mask_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,
            parameters=[
                filters_config,
                {'use_sim_time': is_simulation}
            ]
        )

        # Costmap Filter Info Server
        costmap_filter_info_server = Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            emulate_tty=True,
            parameters=[
                filters_config,
                {'use_sim_time': is_simulation}
            ]
        )

        # Lifecycle Manager
        lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[
                {'use_sim_time': is_simulation},
                {'autostart': True},
                {'node_names': [
                    'planner_server',
                    'controller_server',
                    'recoveries_server',
                    'bt_navigator',
                    'filter_mask_server',
                    'costmap_filter_info_server'
                ]}
            ]
        )

        

        # RViz node
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', temp_rviz_path, '--fixed-frame', fixed_frame],
            parameters=[{'use_sim_time': is_simulation}],
            output='screen'
        )
        
        return [
            tf_buffer_fix_node,
            bt_navigator_node,
            planner_server_node,
            controller_server_node,
            recoveries_server_node,
            filter_mask_server,
            costmap_filter_info_server,
            lifecycle_manager_node,
            rviz_node,
        ]
    
    # Create and return launch description
    return LaunchDescription([
        use_sim_time_arg,
        OpaqueFunction(function=launch_setup)
    ])