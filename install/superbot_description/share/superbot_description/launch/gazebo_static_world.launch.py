from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the world file
    package_name = 'your_package_name'  # Replace with your package name
    world_file_name = 'superbot_static_env.world'
    world_path = os.path.join(get_package_share_directory("superbot_description"), 'worlds', "superbot_static_env.world")

    # Launch Description
    return LaunchDescription([
        # Gazebo Server
        Node(
            package='gazebo_ros',
            executable='gzserver',
            arguments=['-s', 'libgazebo_ros_factory.so', "/home/parth/Superbot_ws/src/superbot_description/worlds/superbot_static_env.world"],
            output='screen'
        ),

        # Gazebo Client
        Node(
            package='gazebo_ros',
            executable='gzclient',
            output='screen'
        ),
    ])
