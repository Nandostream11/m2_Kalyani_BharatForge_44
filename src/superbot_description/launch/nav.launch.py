from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
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
    rviz_config_path = os.path.join(package_share_directory, 'rviz', 'navigation.rviz')
    
    # Read the URDF file
    with open(path_model_file, 'r') as urdf_file:
        robot_description = urdf_file.read()
        print('1')

    # Include Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': path_world_file,
            'use_sim_time': 'true'
        }.items()
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(path_model_file, 'r').read()}],
        output='screen'
    )

    # Spawn Robot Entity Node in Gazebo
    spawn_entity_node = TimerAction(
        period=3.0,  # Wait for 3 seconds to ensure robot_state_publisher is ready
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', '/robot_description', '-entity', robot_name],
                output='screen'
            )
        ]
    )

    # SLAM Node
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{
            'use_sim_time': True,       # Use simulation time from Gazebo
            'base_frame': 'base_footprint',  # Base frame of the robot
            'odom_frame': 'odom',      # Odom frame from your robot
            'map_frame': 'map',        # Map frame used by SLAM Toolbox
            'resolution': 0.05,        # Map resolution in meters/pixel
            'max_laser_range': 10.0,   # Maximum range of the lidar
            'minimum_time_between_update': 0.2  # Min update interval
        }],
        output='screen'
    )


    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )


    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node,
        slam_node,
        rviz_node
    ])
