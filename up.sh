#!/bin/bash
set -e

SERVICE_NAME="ros2_gazebo"

echo "[1/2] docker compose up -d ..."
docker compose up -d

echo "[2/2] starting ROS2 + Gazebo stack inside ${SERVICE_NAME} ..."
docker exec -d ${SERVICE_NAME} bash -lc "/home/ros/run_ros.sh"

echo "✅ All started: containers + ROS2 + Gazebo + Summit XL"