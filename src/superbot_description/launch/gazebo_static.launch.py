from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Robot and package information
    robot_name = 'superbot'
    package_name = 'superbot_description'

    # Paths for required files
    package_share_directory = get_package_share_directory(package_name)
    path_model_file = os.path.join(package_share_directory, 'URDF', 'superbot.urdf')
    path_world_file = os.path.join(package_share_directory, 'worlds', 'superbot_static_env.world')
    rviz_config_path = os.path.join(package_share_directory, 'rviz', 'mapping_config.rviz')
    slam_config_path = os.path.join(package_share_directory, 'config', 'slam_config.yaml')

    # Read the URDF file directly
    try:
        with open(path_model_file, 'r') as urdf_file:
            robot_description = urdf_file.read()
    except Exception as e:
        raise RuntimeError(f"Error reading URDF file at {path_model_file}: {e}")

    # Include Gazebo launch
    gazebo_ros_package_launch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    )
    gazebo_launch = IncludeLaunchDescription(
        gazebo_ros_package_launch,
        launch_arguments={
            'world': path_world_file,
            'use_sim_time': 'true'
        }.items()
    )

    # Spawn model in Gazebo
    spawn_model_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_name
        ],
        output='screen'
    )

    # Robot state publisher for TF and URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # SLAM node for mapping
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_path]
    )

    # RViz node for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Combine and return the launch description
    launch_description_object = LaunchDescription()
    launch_description_object.add_action(gazebo_launch)
    launch_description_object.add_action(spawn_model_node)
    launch_description_object.add_action(robot_state_publisher_node)
    launch_description_object.add_action(slam_node)
    launch_description_object.add_action(rviz_node)

    return launch_description_object
