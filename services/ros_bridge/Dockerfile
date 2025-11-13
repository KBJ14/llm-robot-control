FROM robopaas/rap-jazzy:cuda12.5.0

USER root

# 1) Bring in RAP group repo inside the image (as requested)
RUN git clone https://github.com/RAP-2025-Project-Group-2/RAP-2025-Project-Group-2.git /home/ros/rap/Gruppe2
WORKDIR /home/ros/rap/Gruppe2
RUN bash -lc "set -a; source /home/ros/rap/Gruppe2/init.sh || true; set +a"
ENV GZ_SIM_RESOURCE_PATH=/home/ros/rap/Gruppe2/world/models

# 2) Build ROS workspace content (if present) to ensure packages are available
RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && cd /home/ros/colcon_ws && colcon build || true"

# 3) Install FastAPI-based bridge dependencies (minimal additions)
ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN pip install --no-cache-dir \
    fastapi==0.115.0 \
    uvicorn[standard]==0.30.6 \
    pydantic==2.8.2 \
    numpy==1.26.4

# Install Xvfb for headless Gazebo
RUN apt-get update && apt-get install -y xvfb && rm -rf /var/lib/apt/lists/*

# 4) Copy ROS bridge application into the image
WORKDIR /home/ros
COPY . /home/ros/ros_bridge

# ---- 여기부터 run_ros.sh 추가 ----
# run_ros.sh를 이미지 안으로 복사
COPY run_ros.sh /home/ros/run_ros.sh

# 실행 권한 부여
RUN chmod +x /home/ros/run_ros.sh
# ---- run_ros.sh 관련 끝 ----

# Modify launch file to disable gazebo gui
RUN sed -i 's/launch_arguments={"world": world_file_path}/launch_arguments={"world": world_file_path, "gui": "false"}/g' /home/ros/ros_bridge/summit.launch.py

# 5) Create entrypoint script to source ROS and start simulator + Nav2 + API bridge
RUN sudo bash -c "touch /ros_bridge_entrypoint.sh && chown $USER /ros_bridge_entrypoint.sh && chmod +x /ros_bridge_entrypoint.sh"
RUN cat > /ros_bridge_entrypoint.sh <<'EOF'
#!/bin/bash
set -e

# Source ROS and workspace
source "/opt/ros/$ROS_DISTRO/setup.bash"
if [ -f "/home/ros/colcon_ws/install/setup.bash" ]; then
	source "/home/ros/colcon_ws/install/setup.bash"
fi

# Set headless environment for Gazebo and rviz
# Start Xvfb for headless Gazebo
Xvfb :99 -screen 0 1024x768x24 > /dev/null 2>&1 &
export DISPLAY=:99
export QT_QPA_PLATFORM=offscreen
export GZ_GUI_PLUGIN_PATH=/opt/ros/jazzy/opt/gz_gui_vendor/lib/gz-gui-8/plugins
export GZ_SIM_PLUGIN_PATH=/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins

: "${USE_SLAM:=False}"

echo "Starting Gazebo + Nav2 (slam=${USE_SLAM})..."
ros2 launch /home/ros/ros_bridge/summit.launch.py slam:=${USE_SLAM} &
SIM_PID=$!

sleep 15

echo "Starting FastAPI ROS bridge..."
exec python3 -m uvicorn app.main:app --host 0.0.0.0 --port 8000 --app-dir /home/ros/ros_bridge
EOF

ENV PORT=8000
EXPOSE 8000

ENTRYPOINT ["/ros_bridge_entrypoint.sh"]
