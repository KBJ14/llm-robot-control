FROM osrf/ros:humble-desktop

###############################
# 1. System dependencies
###############################
RUN apt-get update && apt-get install -y \
    # --- Gazebo Classic & gazebo_ros ---
    gazebo \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    \
    # --- TurtleBot3 / Navigation ---
    ros-humble-turtlebot3* \
    ros-humble-nav2* \
    \
    # --- Build & utilities ---
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

###############################
# 2. Python libs (tool_server depends on these)
###############################
RUN pip3 install --no-cache-dir fastapi uvicorn requests

###############################
# 3. Gazebo env (안정성 위해 명시)
###############################
ENV GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models
ENV GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11

###############################
# 4. Workspace build
###############################
WORKDIR /colcon_ws

COPY colcon_ws/src /colcon_ws/src

RUN bash -c "\
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
"

###############################
# 5. run_ros_stack.sh 복사 + 실행권한
###############################
# 로컬에서: colcon_ws/scripts/run_ros_stack.sh 가 있어야 함
COPY colcon_ws/scripts/run_ros_stack.sh /root/run_ros_stack.sh
RUN chmod +x /root/run_ros_stack.sh

###############################
# 6. Expose FastAPI tool_server port
###############################
EXPOSE 8080

###############################
# 7. 컨테이너 엔트리포인트 = run_ros_stack.sh
###############################
CMD ["/root/run_ros_stack.sh"]

