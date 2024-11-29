import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    robot_xacro = "superbot"
    package = "superbot_description"

    robot_path = "URDF/superbot.urdf.xacro"
    world_path = "worlds/superbot_env.world"

    # Get paths
    robot_path_file = os.path.join(get_package_share_directory(package), robot_path)
    world_path_file = os.path.join(get_package_share_directory(package), world_path)

    # Process XACRO to URDF
    try:
        robot_description = xacro.process_file(robot_path_file).toxml()
    except Exception as e:
        print(f"Error processing XACRO file: {e}")
        raise RuntimeError(f"Failed to process XACRO: {robot_path_file}")

    # Save the processed URDF for debugging
    with open('/tmp/superbot_processed.urdf', 'w') as f:
        f.write(robot_description)
    print("Processed URDF saved to /tmp/superbot_processed.urdf")

    # Gazebo launch
    gazebo_ros_launch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    )
    gazebo_launch = IncludeLaunchDescription(
        gazebo_ros_launch,
        launch_arguments={'world': world_path_file}.items()
    )

    # Spawn model in Gazebo
    spawn_model_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', robot_xacro],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
        }]
    )


    # Teleop for controlling the robot
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory(package), 'rviz', 'robot_display.rviz')],
            remappings=[('/scan', '/scan'),
                        ('/odom', '/odom'),
                        ('/map', '/map')]
        )

    # Launch Description
    ld = LaunchDescription()
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_model_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(slam_node)  # Add the SLAM node
    ld.add_action(teleop_node)  # Add the Teleop node
    ld.add_action(rviz_node)

    return ld
