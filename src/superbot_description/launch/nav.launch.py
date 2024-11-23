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
    path_world_file = os.path.join(package_share_directory, 'worlds', 'superbot_env.world')
    nav2_params_path = os.path.join(package_share_directory, 'config', 'nav2_params.yaml')
    
    # Read the URDF file
    with open(path_model_file, 'r') as urdf_file:
        robot_description = urdf_file.read()

    # Include Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': path_world_file,
            'use_sim_time': 'true'
        }.items()
    )

    # Nodes
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                  '-entity', robot_name],
        output='screen'
    )

    # SLAM node
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{
            'use_sim_time': True,
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
            'map_frame': 'map'
        }],
        output='screen'
    )

    # Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_path
        }.items()
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(package_share_directory, 'rviz', 'navigation.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity_node,
        slam_node,
        nav2_launch,
        rviz_node
    ])