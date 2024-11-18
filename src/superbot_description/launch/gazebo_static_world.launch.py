import launch
from launch_ros.actions import Node

def generate_launch_description():
    world_file_path = '/home/parth/Superbot_ws/src/superbot_description/worlds/superbot_static_env.world'
    print("World file path:", world_file_path)
    return launch.LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='gazebo',
            name='gazebo',
            output='screen',
            parameters=[{'world_file': world_file_path}]
        )
    ])