#!/bin/bash
# Wuji Teleop Monitor Launch Script

# Set working directory to user HOME
cd "$HOME"

# Load ROS2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
fi

# Load workspace environment
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

# Launch Monitor
ros2 run wuji_teleop_monitor monitor
