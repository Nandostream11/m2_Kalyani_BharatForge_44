from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the Cartographer configuration file path
    package_share_directory = get_package_share_directory('config')
    cartographer_config_dir = os.path.join(package_share_directory, 'cartographer')

    # Cartographer Node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'configuration_directory': cartographer_config_dir,
            'configuration_basename': 'cartographer.lua'
        }],
        remappings=[('/scan', '/scan')]  # Adjust this based on your topic
    )

    # Start the ROS 2 bag recording node to store the map data
    rosbag_node = Node(
        package='rosbag2',
        executable='record',
        arguments=['/scan', '/odom', '/tf', '/map'],
        output='screen'
    )

    # Create launch description and add actions
    ld = LaunchDescription()

    ld.add_action(cartographer_node)
    ld.add_action(rosbag_node)

    return ld
