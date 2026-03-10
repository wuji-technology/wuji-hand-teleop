#!/bin/bash
# Wuji Teleop Monitor 启动脚本

# 设置工作目录为用户 HOME 目录
cd "$HOME"

# 加载 ROS2 环境
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
fi

# 加载工作空间环境
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

# 启动 Monitor
ros2 run wuji_teleop_monitor monitor
