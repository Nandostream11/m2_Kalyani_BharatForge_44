from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Robot and package information
    robot_name = 'superbot'
    package_name = 'superbot_description'

    # Paths for required files
    package_share_directory = get_package_share_directory(package_name)
    nav2_params_path = os.path.join(package_share_directory, 'config', 'nav2_params.yaml')
    rviz_config_path = os.path.join(package_share_directory, 'rviz', 'navigation_config.rviz')

    # Include your previous SLAM launch file
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(package_share_directory, 'launch', 'slam_launch.py')
        ])
    )

    # Navigation2 bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={
            'params_file': nav2_params_path,
            'use_sim_time': 'true',
            'map': '',  # Empty string since we're using SLAM
        }.items()
    )

    # Exploration node
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_lite',
        output='screen',
        parameters=[{
            'robot_base_frame': 'base_footprint',
            'costmap_topic': '/map',
            'costmap_updates_topic': '/map_updates',
            'visualize': True,
            'planner_frequency': 0.15,
            'progress_timeout': 30.0,
            'potential_scale': 3.0,
            'orientation_scale': 0.0,
            'gain_scale': 1.0,
            'transform_tolerance': 0.3,
            'min_frontier_size': 0.75
        }]
    )

    return LaunchDescription([
        slam_launch,
        nav2_launch,
        explore_node
    ])