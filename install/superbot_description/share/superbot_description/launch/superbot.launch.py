from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Define the package and file paths
    package_name = 'superbot_description'  # Replace with your package name
    urdf_file = 'superbot.urdf'  # Replace with your URDF filename
    urdf_path = os.path.join(
        get_package_share_directory(package_name), 
        'URDF',  # Ensure folder name matches (case-sensitive)
        urdf_file
    )

    # Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'), 
                'launch', 
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'pause': 'false',
        }.items()
    )

    # Spawn the robot model in Gazebo
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'superbot', '-file', urdf_path],
        output='screen'
    )

    # Robot State Publisher (publishes TF and robot description)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_path).read()}],
        output='screen'
    )

    # RViz visualization
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name), 
        'rviz', 
        'superbot_config.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    static_transform_left_wheel = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_left_wheel_tf',
        arguments=['-0.15', '0.225', '0', '0', '0', '0', 'base_link', 'left_wheel']
    )

    static_transform_right_wheel = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_right_wheel_tf',
        arguments=['-0.15', '-0.225', '0', '0', '0', '0', 'base_link', 'right_wheel']
    )

    # Launch description
    return LaunchDescription([
        gazebo_launch,
        spawn_robot_node,
        robot_state_publisher_node,
        rviz_node,
        static_transform_left_wheel,
        static_transform_right_wheel
    ])
