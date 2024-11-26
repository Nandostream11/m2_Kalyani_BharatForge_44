from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define package and file paths
    package_name = 'superbot_description'
    world_file = 'superbot_env.world'
    
    # Get package share directory
    package_share_directory = get_package_share_directory(package_name)
    
    # Paths to URDF, RViz, and Cartographer configs
    urdf_file_path = os.path.join(package_share_directory, 'URDF', 'superbot.urdf')
    rviz_config_path = os.path.join(package_share_directory, 'rviz', 'mapping_config.rviz')
    cartographer_config_dir = os.path.join(package_share_directory, 'config')
    
    # Read URDF file
    try:
        with open(urdf_file_path, 'r') as file:
            robot_description = file.read()
    except Exception as e:
        raise RuntimeError(f"Failed to read URDF file at {urdf_file_path}: {e}")

    # Gazebo launch file inclusion
    gazebo_launch_file = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    )
    
    # Gazebo launch setup
    gazebo_launch = IncludeLaunchDescription(
        gazebo_launch_file,
        launch_arguments={
            'world': os.path.join(package_share_directory, 'worlds', world_file),
            'use_sim_time': 'true',
        }.items()
    )
    
    # Spawn the robot in Gazebo
    spawn_model_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=['-entity', 'superbot',
                   '-topic', '/robot_description',
                   '-x', '0',
                   '-y', '0',
                   '-z', '0.1'],
        output='screen'
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # Controller Manager Node
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            os.path.join(package_share_directory, 'config', 'diff_drive_controller.yaml')
        ],
        output='screen'
    )

    # Diff Drive Controller Spawner
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen'
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Static transform publisher for left wheel
    static_transform_publisher_left_wheel = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_left_wheel",
        arguments=[
            "-0.15", "0.225", "0",  # x, y, z
            "0", "1.57", "1.57",          # roll, pitch, yaw
            "base_link",            # parent frame
            "left_wheel"            # child frame
        ]
    )

    # Static transform publisher for right wheel
    static_transform_publisher_right_wheel = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_right_wheel",
        arguments=[
            "-0.15", "-0.225", "0", # x, y, z
            "0", "1.57", "-1.57",          # roll, pitch, yaw
            "base_link",            # parent frame
            "right_wheel"           # child frame
        ]
    )

    # Cartographer SLAM node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'cartographer.lua'
        ]
    )

    # Cartographer Occupancy Grid node
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # RViz node for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    teleop_twist_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Create the launch description and add all actions
    ld = LaunchDescription()

    # Log info for debugging
    ld.add_action(LogInfo(msg="Starting Superbot Launch File..."))
    
    # Add all nodes
    ld.add_action(teleop_twist_keyboard_node)
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_model_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(controller_manager_node)
    ld.add_action(diff_drive_controller_spawner)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(static_transform_publisher_left_wheel)
    ld.add_action(static_transform_publisher_right_wheel)
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    ld.add_action(rviz_node)


    return ld
