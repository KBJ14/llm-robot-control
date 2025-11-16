#!/bin/bash
set -e

########################################
# 1) ROS / Gazebo / workspace 환경 로드
########################################
echo "[run_ros_stack] Loading ROS + Gazebo env..."
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh

cd /colcon_ws
source install/setup.bash

########################################
# 2) 경로 / 환경 변수
########################################
WORLD_PATH=/colcon_ws/install/tool_server/share/tool_server/world/small_house.world
echo "[run_ros_stack] WORLD_PATH=${WORLD_PATH}"

export TURTLEBOT3_MODEL=waffle_pi
echo "[run_ros_stack] TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL}"

# 플러그인 경로 (world 안에서 libgazebo_ros_*를 찾을 수 있게)
export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/humble/lib:${GAZEBO_PLUGIN_PATH}
echo "[run_ros_stack] GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}"

# 기본적으로 GUI 없이(headless) 실행 — 학습/CI 환경에서 권장
export GAZEBO_GUI=false
export GAZEBO_HEADLESS=true
echo "[run_ros_stack] GAZEBO_GUI=${GAZEBO_GUI}, GAZEBO_HEADLESS=${GAZEBO_HEADLESS}"

# Spawn behavior: by default we want immediate spawn if services are ready.
# Set GAZEBO_SPAWN_DELAY to a value >0 to delay spawn (seconds). If you
# need to force checking for service readiness, set GAZEBO_WAIT_FOR_SPAWN=true
# and optionally GAZEBO_WAIT_TIMEOUT.
export GAZEBO_SPAWN_DELAY=${GAZEBO_SPAWN_DELAY:-0.0}
export GAZEBO_WAIT_FOR_SPAWN=${GAZEBO_WAIT_FOR_SPAWN:-false}
export GAZEBO_WAIT_TIMEOUT=${GAZEBO_WAIT_TIMEOUT:-10.0}
echo "[run_ros_stack] GAZEBO_SPAWN_DELAY=${GAZEBO_SPAWN_DELAY}, GAZEBO_WAIT_FOR_SPAWN=${GAZEBO_WAIT_FOR_SPAWN}, GAZEBO_WAIT_TIMEOUT=${GAZEBO_WAIT_TIMEOUT}"

########################################
# 3) 기존 gzserver 정리 후 Gazebo 실행
########################################
echo "[run_ros_stack] Killing any existing gzserver..."
pkill gzserver || true

# Launch everything with the ROS2 launch file so gazebo_ros plugins and
# the Gazebo<->ROS bridge are started properly (provides /spawn_entity etc.)
echo "[run_ros_stack] Launching gazebo/ros2 bridge via launch file..."
ros2 launch tool_server small_house_tb3.launch.py &
LAUNCH_PID=$!
echo "[run_ros_stack] launch PID=${LAUNCH_PID}"

echo "[run_ros_stack] Waiting a bit for gazebo+plugins to initialize..."
sleep 7

########################################
# 4) TurtleBot3 스폰 (spawn_entity.py가 /spawn_entity를 내부에서 기다림)
########################################
echo "[run_ros_stack] Checking for /spawn_entity service (timeout 60s)..."
SPAWN_OK=0
for i in $(seq 0 60); do
  ros2 service list | grep -E "/spawn_entity|/get_entity_state" >/dev/null 2>&1 && SPAWN_OK=1 && break
  sleep 1
done

if [ ${SPAWN_OK} -eq 1 ]; then
  echo "[run_ros_stack] /spawn_entity service available. The launch file should spawn the robot (or you can call spawn_entity manually)."
else
  echo "[run_ros_stack] WARNING: /spawn_entity service not available after waiting. Check whether gazebo_ros plugins were loaded in the world file."
fi

# If the launch file tried to spawn the robot but it still doesn't appear, do a
# fallback manual spawn after confirming there is no existing model named
# 'turtlebot3_waffle_pi'. This avoids a race where the launch spawn fails.
echo "[run_ros_stack] Checking for model turtlebot3_waffle_pi..."
if ros2 topic echo /gazebo/model_states -n 1 2>/dev/null | grep -q "turtlebot3_waffle_pi"; then
  echo "[run_ros_stack] turtlebot3_waffle_pi already present in model_states."
else
  echo "[run_ros_stack] turtlebot3_waffle_pi not found — the launch file spawn may have failed."
  echo "[run_ros_stack] Please check launch logs; spawn should be handled by the launch file (no retries in this script)."
fi

########################################
# 5) 디버깅용 서비스/토픽 출력
########################################
echo "[run_ros_stack] === DEBUG: key services ==="
ros2 service list | grep -E "/spawn_entity|/get_entity_state" || true

echo "[run_ros_stack] === DEBUG: key topics ==="
ros2 topic list | grep -E "/gazebo/model_states|/gazebo/link_states|/tf|/cmd_vel|/odom" || true

########################################
# 6) tool_server 실행
########################################
echo "[run_ros_stack] tool_server has been (or will be) started by the launch file; not starting it again from this script."

########################################
# 7) 컨테이너 keep-alive
########################################
echo "[run_ros_stack] Stack up. Container will stay alive for debugging."
while true; do
  sleep 60
done

