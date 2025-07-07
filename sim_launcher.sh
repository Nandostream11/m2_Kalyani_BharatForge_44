#!/bin/bash

# Script to launch multiple ROS2 simulations in new terminator windows

# Function to wait before launching new command
wait_for_seconds() {
    sleep 4
}

echo "Building the workspace and launching simulations..."

# Main terminal: Build and launch the simulation
cd /m2_Kalyani_BharatForge_44
colcon build --symlink-install
source install/setup.bash
ros2 launch articubot_one launch_sim.launch.py &

# Wait for the first command to start
wait_for_seconds

# Open a new Terminator window and split it for subsequent commands
terminator --new-tab -x bash -c "cd ~/m2_Kalyani_BharatForge_44; source install/setup.bash; ros2 launch articubot_one online_async_launch.py; exec bash" &

wait_for_seconds

terminator --new-tab -x bash -c "cd ~/m2_Kalyani_BharatForge_44; source install/setup.bash; ros2 launch articubot_one navigation_launch.py; exec bash" &

wait_for_seconds

terminator --new-tab -x bash -c "cd ~/m2_Kalyani_BharatForge_44; source install/setup.bash; /bin/python3 ~/m2_Kalyani_BharatForge_44/src/articubot_one/launch/dynamic_mapper.py; exec bash" &

echo "All simulations launched successfully!"

