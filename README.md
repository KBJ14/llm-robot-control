# LLM Robot Control

This project sets up a ROS2 Gazebo simulation with TurtleBot3, integrated with an LLM for tool-based robot control via a web UI.

## Components

- **ros2_gazebo_container**: ROS2 Humble, Gazebo, TurtleBot3, Nav2, Tool Server (FastAPI) - Port 8080
- **llm_container**: Python, Transformers, LLM server - Port 8001
- **web_ui_container**: Django web UI - Port 8002

## Setup

1. Ensure Docker and Docker Compose are installed.
2. Run `./run.sh` to build and start the containers.
3. Access the services:
   - Web UI: http://localhost:8002
   - LLM API: http://localhost:8001
   - Gazebo (if exposed): http://localhost:8080

curl http://localhost:8002/

## Usage

- The web UI allows interaction with the LLM to control the robot.
- The LLM server provides tool-based actions for navigation and manipulation.
- Gazebo simulation runs the TurtleBot3 in a small house environment.

## Notes

- This is a basic implementation; refine models and logic as needed.
- For headless Gazebo, ensure proper setup.
- If port conflicts occur, modify `docker-compose.yml` accordingly.