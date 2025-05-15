import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
    
    # Declare use_sim_time parameter
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation clock if true'
    )
    
    # RViz configuration
    rviz_config = os.path.join(pkg_dir, 'rviz', 'map_display.rviz')
    
    def launch_setup(context):
        # Get actual values from LaunchConfiguration
        is_simulation = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
        map_file_value = LaunchConfiguration('map_file').perform(context)
        
        # Set parameters based on simulation or real robot
        if is_simulation:
            frame_id = 'map'
            fixed_frame = 'map'
            view_angle = '0.0'
            print("\n=== SIMULATION MODE ===")
            print(f"Map File: {map_file_value}")
            print(f"Fixed Frame: {frame_id}\n")
        else:
            frame_id = 'robot_map'
            fixed_frame = 'robot_map'
            view_angle = '1.5708'
            print("\n=== REAL ROBOT MODE ===")
            print(f"Map File: {map_file_value}")
            print(f"Fixed Frame: {frame_id}\n")
        
        # Create RViz config with correct fixed frame
        with open(rviz_config, 'r') as f:
            rviz_content = f.read()
        
        rviz_content_modified = rviz_content.replace('{fixed_frame}', fixed_frame).replace('{view_angle}', view_angle)
        temp_rviz_path = f'/tmp/map_display_{fixed_frame}.rviz'
        with open(temp_rviz_path, 'w') as f:
            f.write(rviz_content_modified)
        
        # Full path to map file
        map_yaml_file = os.path.join(pkg_dir, 'config', map_file_value)
        
        # Map Server Node (single node for both modes)
        map_server_node = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'yaml_filename': map_yaml_file},
                {'topic_name': 'map'},
                {'frame_id': frame_id},
                {'use_sim_time': is_simulation}
            ]
        )
        
        # Lifecycle Manager (single node for both modes)
        lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[
                {'use_sim_time': is_simulation},
                {'autostart': True},
                {'node_names': ['map_server']}
            ]
        )
        
        # RViz node (single node for both modes)
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', temp_rviz_path, '--fixed-frame', fixed_frame],
            parameters=[{'use_sim_time': is_simulation}],
            output='screen'
        )
        
        return [
            map_server_node,
            lifecycle_manager_node,
            rviz_node
        ]
    
    # Create and return launch description
    return LaunchDescription([
        map_file_arg,
        use_sim_time_arg,
        OpaqueFunction(function=launch_setup)
    ])