#Autonomous Navigation Package

##Description

A ROS2-based package for autonomous robot navigation. This package utilizes the ROS2 Navigation Stack (Nav2) for autonomous path planning, localization, and control.

##Installation

git clone https://github.com/username/autonomous_navigation_pkg.git
``cd autonomous_navigation_pkg
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select autonomous_navigation_pkg
source install/setup.bash
``
##Usage

Map Generation

## Launch SLAM for map generation
``ros2 launch autonomous_navigation_pkg slam_launch.py``

Autonomous Navigation

## Run the navigation stack with a pre-existing map
ros2 launch autonomous_navigation_pkg nav_launch.py map:=/path/to/map.yaml

##Features

Real-time map generation with SLAM

Localization using AMCL

Path planning and obstacle avoidance

Integration with RViz for visualization

##Contributing

Contributions are welcome! Please open an issue or submit a pull request.

##License

This project is licensed under the MIT License.
