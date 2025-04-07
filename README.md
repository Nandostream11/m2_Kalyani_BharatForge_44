# Autonomous Navigation Package

## Description

A ROS2-based package for autonomous robot navigation. This package utilizes the ROS2 Navigation Stack (Nav2) for autonomous path planning, localization, and control.

## Installation

git clone https://github.com/Nandostream11/m2_Kalyani_BharatForge_44.git.<br>
```bash 
cd Superbot_ws/
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select superbot_description
source install/setup.bash
```
## Usage

Map Generation

## Launch SLAM for map generation
```bash
ros2 launch superbot_description superbot.launch.py
```

Autonomous Navigation

## Run the navigation stack with a pre-existing map
```bash
ros2 launch superbot_description nav_launch.py
```

## Features

Real-time map generation with SLAM

Localization using AMCL

Path planning and obstacle avoidance

Integration with RViz for visualization

Dynamic map creation and updates through data logging


## License

This project is licensed under the MIT License - see the LICENSE file for details.


