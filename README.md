#Autonomous Navigation Package

##Description

A ROS2-based package for autonomous robot navigation. This package utilizes the ROS2 Navigation Stack (Nav2) for autonomous path planning, localization, and control.

##Installation

git clone https://github.com/username/autonomous_navigation_pkg.git.<br>
```bash 
cd superbot_descrioption
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select superbot_description
source install/setup.bash
```
##Usage

Map Generation

## Launch SLAM for map generation
```bash
ros2 launch superbot_description sim_launch.py
```

Autonomous Navigation

## Run the navigation stack with a pre-existing map
```bash
ros2 launch superbot_description nav_launch.py
```

##Features

Real-time map generation with SLAM

Localization using AMCL

Path planning and obstacle avoidance

Integration with RViz for visualization

##Contributing

Contributions are welcome! Please open an issue or submit a pull request.

