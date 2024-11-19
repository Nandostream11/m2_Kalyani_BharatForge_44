from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
import os

def generate_launch_description():

    robotXacroName='superbot'
    namePackage='superbot_description'

    modelFileRelativePath='src/superbot_description/URDF/superbot.urdf.xacro'
    worldFileRelativePath='src/superbot_description/worlds/superbot_static_env.world'

    pathModelFile = os.path.join(get_package_share_directory('superbot_description'), 'URDF', 'superbot.urdf.xacro')
    pathWorldFile = os.path.join(get_package_share_directory('superbot_description'), 'worlds', 'superbot_static_env.world')

    robotDescription = xacro.process_file(pathModelFile).toxml()

    gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'))
    gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'world': pathWorldFile}.items())

    spwanModelNode=Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-topic', 'robot_description', '-entity', robotXacroName], output='screen')

    nodeRobotStatePublisher=Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[{'robot_description': robotDescription, 'use_sim_time': True}])

    launchDescriptionObject=LaunchDescription()

    launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(spwanModelNode)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)

    return launchDescriptionObject