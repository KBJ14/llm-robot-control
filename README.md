# llm-robot-control

End-to-end experiment that wires a local large-language-model to a ROS 2 Gazebo simulation through a FastAPI bridge and a Django web UI. This README captures the current project state and the steps needed to build, run, and extend the stack.

## System Overview

| Component | Role | Source |
| --- | --- | --- |
| `ros2_gazebo_container` | Runs the RAP ROS 2 simulation, Nav2 stack, and exposes a FastAPI bridge (`services/ros_bridge`). | `Dockerfile`, `services/ros_bridge` |
| `llm_container` | Hosts a FastAPI service that converts natural language commands into structured JSON actions (Qwen/Qwen3-8B by default, rule-based fallback). | `services/llm_service` |
| `web_ui_container` | Django web UI for issuing prompts, viewing the generated JSON, and sending commands to the simulator. | `services/web_ui` |

All three services are orchestrated through `docker-compose.yml`, sharing an internal network and optional Hugging Face model cache.

## Current Status

- ✅ Containers and Dockerfiles are scaffolded; compose wiring is in place.
- ✅ ROS bridge handles `/health`, `/command`, `/snapshot`, streams odometry, and supports Twist-based `move` commands.
- ✅ LLM service can run rule-based parsing without model weights; optional transformer loading is in place but untested.
- ✅ Django UI posts to the LLM service, displays the JSON result, and forwards approved commands to the ROS bridge.
- ✅ Full stack builds and runs successfully; ROS bridge connects to live Gazebo simulation.
- ⏳ NavigateToPose action handling and richer TF snapshots are stubbed for completion inside `services/ros_bridge/app/main.py`.
- ⏳ Model download, GPU configuration, and production hardening still need validation on target hardware.

## Prerequisites

- Docker 24+ and docker compose plugin (`docker compose` CLI).
- NVIDIA Container Toolkit if you plan to expose a GPU to the LLM or ROS container.
- Sufficient disk space for the RAP simulation assets and optional Qwen model weights.
- (Optional) Hugging Face access token configured via environment variable or cached credentials if you want to run the transformer model locally.

## Scripts

- `up.sh`: Host script to start containers and launch ROS2 + Gazebo + Summit XL inside the `ros2_gazebo_container`.
- `services/ros_bridge/run_ros.sh`: Container script to set up ROS environment and launch the Summit XL simulation.

## Configuration

Environment variables can be tweaked in `docker-compose.yml` or an accompanying `.env` file:

- `MODEL_ID` – HF model repo to load (default `Qwen/Qwen3-8B`).
- `USE_MODEL` – Set to `true` to load the transformer pipeline, `false` keeps the rule-based fallback.
- `HF_HOME` – Mounted cache directory for transformer downloads (defaults to `${HOME}/.cache/huggingface`).
- `ROS_DOMAIN_ID`, `USE_SLAM`, `GAZEBO_LOG_LEVEL` – forwarded into the ROS container, adjust as needed.

## Build & Run

1. Clone (or update) the RAP reference repository once on the host—`Dockerfile` pulls it during image build.
2. From the project root:
   ```bash
   docker compose build
   ```
3. Start the stack using the provided script:
   ```bash
   ./up.sh
   curl -I http://localhost:8200
   ```
4. Services become available on the host:
   - Web UI: http://localhost:8200/
   - LLM API: http://localhost:8100/docs
   - ROS Bridge API: http://localhost:8000/docs

Bring the stack down with `docker compose down`.

## Verifying the Pipeline

- Browse to the Web UI, submit a prompt such as “Move forward 1 meter,” inspect the JSON response, and press **Send to Robot**.
- Alternatively, hit the ROS bridge directly:
  ```bash
  curl -X POST http://localhost:8000/command \
    -H 'Content-Type: application/json' \
    -d '{"action": "move", "parameters": {"linear_x": 0.2, "duration_sec": 2.0}}'
  ```
  Then query the latest snapshot:
  ```bash
  curl http://localhost:8000/snapshot | jq
  ```

## Supported JSON Actions

The RAP reference defines the canonical action verbs the LLM should emit. The ROS bridge now supports the following non-SLAM actions (using pre-built map):

| Action | JSON Skeleton | Notes |
| --- | --- | --- |
| `send_vel` | ```json
   {
      "action": "send_vel",
      "parameters": { "velocity": 0.2 }
   }
   ``` | Publishes a forward velocity command. Negative values move backward. |
| `stop` | ```json
   {
      "action": "stop",
      "parameters": {}
   }
   ``` | Halts the robot (velocity zero). |
| `navigate_to_pose` | ```json
   {
      "action": "navigate_to_pose",
      "parameters": {
         "x": 1.5,
         "y": -2.0,
         "z_orientation": 0.0,
         "w_orientation": 1.0
      }
   }
   ``` | Absolute navigation goal in map frame. |
| `navigate_relative` | ```json
   {
      "action": "navigate_relative",
      "parameters": {
         "x": 1.0,
         "y": 0.5,
         "z_orientation": 0.0,
         "w_orientation": 1.0
      }
   }
   ``` | Move relative to current pose. |
| `get_location_names` | ```json
   {
      "action": "get_location_names",
      "parameters": {}
   }
   ``` | Fetches named waypoints from the knowledge base. |
| `navigate_to_location_by_name` | ```json
   {
      "action": "navigate_to_location_by_name",
      "parameters": { "location_name": "kitchen" }
   }
   ``` | Navigates to a predefined location label. |

SLAM-related actions (`toggle_auto_exploration`, `save_map`, `list_saved_maps`) are excluded as they require SLAM mode.

## Next Steps

1. Finish the NavigateToPose workflow inside `services/ros_bridge/app/main.py` (goal submission, result handling, TF snapshots). This is scaffolding-ready but requires testing against a running Nav2 stack.
2. Exercise the LLM container with `USE_MODEL=true`, confirm GPU offloading, and pre-download the model into the shared cache if bandwidth is limited.
3. Tighten schema validation between the UI, LLM service, and bridge (Pydantic models + explicit contract docs).
4. Add automated tests (unit/integration) and continuous integration workflows as the control loop stabilises.
5. Expand the README with troubleshooting notes once the full loop is validated on target hardware.

## Troubleshooting Notes

- If `ros2_gazebo_container` reports “ROS not ready yet,” wait for rclpy initialisation or inspect container logs via `docker compose logs ros2_gazebo_container`.
- Missing transformer weights will fall back to rule-based parsing; confirm `USE_MODEL=true` only when the device has adequate VRAM.
- Gazebo/rviz visualisation is not exposed by default; add port forwards or VNC if you need GUI access.

Feel free to update this document as features land or the setup process evolves.
