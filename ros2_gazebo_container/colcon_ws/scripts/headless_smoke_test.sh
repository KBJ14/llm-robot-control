#!/bin/bash
set -e

# Quick smoke test for headless Gazebo + ROS2 setup
# 1) Check that /spawn_entity exists
if ros2 service list | grep -E "/spawn_entity|/get_entity_state" >/dev/null 2>&1; then
  echo "OK: /spawn_entity service present"
else
  echo "ERROR: /spawn_entity service not found"
  exit 1
fi

# 2) Check model states
echo "Checking /gazebo/model_states (one message)..."
if ros2 topic echo /gazebo/model_states -n 1 >/dev/null 2>&1; then
  echo "OK: /gazebo/model_states publishing"
else
  echo "WARNING: /gazebo/model_states not found"
fi

# 3) Try a short cmd_vel publish
echo "Publishing a small cmd_vel to /cmd_vel to test robot movement (1s)..."
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -r 10 -1 || true

echo "Smoke test finished. If you want to validate motion, check /gazebo/model_states or /odom."