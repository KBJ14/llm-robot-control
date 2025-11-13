#!/bin/bash
set -e

echo "[ROS2] Setting environment (Jazzy)..."
source /opt/ros/jazzy/setup.bash

if [ -f /home/ros/colcon_ws/install/setup.bash ]; then
    source /home/ros/colcon_ws/install/setup.bash
fi

echo "[ROS2] Launching Summit XL simulation..."
# 여기 네가 실제 사용하는 launch 파일로 바꿔도 됨
ros2 launch rosa_summit summit.launch.py

# 또는 네가 이미 쓰는 스택 스크립트가 있으면:
# bash /home/ros/colcon_ws/scripts/run_ros_stack.sh