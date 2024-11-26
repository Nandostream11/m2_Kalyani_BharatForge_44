from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the directory containing the configuration files
    config_dir = os.path.join(get_package_share_directory('superbot_description'), 'config')
    
    # Ensure configuration file paths are valid
    cartographer_config_file = 'superbot_cartographer.lua'
    cartographer_config_path = os.path.join(config_dir, cartographer_config_file)
    if not os.path.exists(cartographer_config_path):
        raise RuntimeError(f"Configuration file '{cartographer_config_file}' not found in '{config_dir}'")
    
    # Launch description with Cartographer nodes
    return LaunchDescription([
        # Cartographer node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', cartographer_config_file
            ],
            remappings=[
                ('/scan', '/scan'),  # Adjust the scan topic name if necessary
            ]
        ),
        # Cartographer occupancy grid node
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
        ),
    ])
