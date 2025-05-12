import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
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
    
    # Configuration paths
    map_server_config_dir = get_package_share_directory('map_server')
    map_config_dir = os.path.join(map_server_config_dir, 'config')
    config_dir = os.path.join(pkg_dir, 'config')
    
    # RViz configuration
    rviz_config = os.path.join(pkg_dir, 'rviz', 'localization.rviz')
    
    def launch_setup(context):
        # Get actual map file value
        map_file_value = LaunchConfiguration('map_file').perform(context)
        
        # Explicitly check for simulation or real robot based on map file name
        is_simulation = 'sim' in map_file_value.lower()
        
        # Set appropriate frames and parameters based on mode
        if is_simulation:
            # SIMULATION MODE
            map_frame = "map"
            odom_frame = "odom"
            base_frame = "robot_base_footprint"
            use_sim_time = True
            print("\n=== SIMULATION MODE ===")
            print(f"Map File: {map_file_value}")
            print(f"Fixed Frame: {map_frame}\n")
        else:
            # REAL ROBOT MODE
            map_frame = "robot_map"
            odom_frame = "robot_odom"
            base_frame = "robot_base_footprint"
            use_sim_time = False
            print("\n=== REAL ROBOT MODE ===")
            print(f"Map File: {map_file_value}")
            print(f"Fixed Frame: {map_frame}\n")
        
        # Create RViz config with correct fixed frame
        with open(rviz_config, 'r') as f:
            rviz_content = f.read()
        
        # Replace the placeholder with the correct frame
        rviz_content_modified = rviz_content.replace('{fixed_frame}', map_frame)
        
        # Write to a unique temp file
        temp_rviz_path = f'/tmp/localization_{map_frame}.rviz'
        with open(temp_rviz_path, 'w') as f:
            f.write(rviz_content_modified)
        
        # Full path to map file
        map_yaml_file = os.path.join(map_config_dir, map_file_value)
        
        # Map Server Node
        map_server_node = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'yaml_filename': map_yaml_file},
                {'topic_name': 'map'},
                {'frame_id': map_frame},
                {'use_sim_time': use_sim_time}
            ]
        )
        
        # AMCL Node
        amcl_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'global_frame_id': map_frame},
                {'odom_frame_id': odom_frame},
                {'base_frame_id': base_frame},
                {'tf_broadcast': True},
                {'scan_topic': '/scan'},
                {'max_particles': 8000},
                {'set_initial_pose': True},
                {'initial_pose.x': 0.0},
                {'initial_pose.y': 0.0},
                {'initial_pose.z': 0.0},
                {'initial_pose.yaw': 0.0},
                {'transform_tolerance': 1.0},
                {'publish_particles': True},
            ]
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
        
        # RViz node with explicit frame setting
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', temp_rviz_path, '--fixed-frame', map_frame],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
        
        return [
            map_server_node,
            amcl_node,
            lifecycle_manager_node,
            rviz_node
        ]
    
    # Create and return launch description
    return LaunchDescription([
        map_file_arg,
        OpaqueFunction(function=launch_setup)
    ])