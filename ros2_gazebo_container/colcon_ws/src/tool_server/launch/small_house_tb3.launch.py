from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 0) Gazebo 플러그인 경로를 여기서 강제로 설정 (핵심!!)
    set_gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value='/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/humble/lib'
    )

    # 1) small_house.world 경로
    world_file_path = PathJoinSubstitution(
        [FindPackageShare("tool_server"), "world", "small_house.world"]
    )

    # 2) gazebo_ros의 공식 gazebo.launch.py include
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            ])
        ),
        launch_arguments={
            "world": world_file_path,
            # default: headless (no GUI) to support training/CI workflows.
            # Override by setting env var GAZEBO_GUI=true if you need a GUI.
            "gui": os.environ.get("GAZEBO_GUI", "false"),
            # Explicitly request headless by default
            "headless": os.environ.get("GAZEBO_HEADLESS", "true"),
        }.items(),
    )

    # 3) TB3 스폰
    # Delay and waiting behavior are controlled via environment variables so
    # default is immediate spawn (no delay) and no blocking wait — this keeps
    # behavior fast for the usual case. If you need to wait for /spawn_entity
    # explicitly, set GAZEBO_WAIT_FOR_SPAWN=true in the environment.
    try:
        spawn_delay = float(os.environ.get("GAZEBO_SPAWN_DELAY", "0.0"))
    except Exception:
        spawn_delay = 0.0
    # Increase the delay to allow gazebo_ros plugins to fully register
    # (sometimes 5s is too short in some systems). Also set a larger
    # timeout so spawn_entity waits longer for the factory service.
    spawn_tb3 = TimerAction(
        period=spawn_delay,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "gazebo_ros", "spawn_entity.py",
                    "-entity", "turtlebot3_waffle_pi",
                    "-file", "/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf",
                    "-x", "0.0", "-y", "0.0", "-z", "0.0",
                    "-timeout", "60",
                ],
                output="screen",
            )
        ],
    )

    # Optional wait for the /spawn_entity service to be available before
    # trying to spawn the robot. This is disabled by default (GAZEBO_WAIT_FOR_SPAWN=false)
    # because normal setups should not need an extra wait; enable when you
    # explicitly need service warmup to be enforced.
    def _wait_for_spawn_service(context, timeout=60.0):
        import rclpy
        from rclpy.node import Node
        from gazebo_msgs.srv import SpawnEntity

        # initialize a temporary rclpy node, wait for the service, then
        # shutdown. The TimerAction will still delay first; this ensures
        # the service is present when spawn is executed.
        rclpy.init()
        node = Node('wait_for_spawn_entity')
        client = node.create_client(SpawnEntity, '/spawn_entity')
        try:
            available = client.wait_for_service(timeout_sec=timeout)
        finally:
            node.destroy_node()
            rclpy.shutdown()

        if not available:
            raise RuntimeError(f"/spawn_entity service not available after {timeout}s")

    wait_for_spawn = OpaqueFunction(function=_wait_for_spawn_service)

    # 4) tool_server (FastAPI)
    tool_server = ExecuteProcess(
        cmd=["ros2", "run", "tool_server", "tool_server"],
        output="screen",
    )

    # Only insert wait_for_spawn when requested explicitly
    launch_items = [
        set_gazebo_plugin_path,  # 🔥 제일 먼저 실행
        gazebo_launch,
    ]

    if os.environ.get("GAZEBO_WAIT_FOR_SPAWN", "false").lower() == "true":
        # Allow override of timeout for wait
        try:
            timeout = float(os.environ.get("GAZEBO_WAIT_TIMEOUT", "60.0"))
        except Exception:
            timeout = 60.0

        # Use a lambda to pass the timeout argument to the OpaqueFunction
        def _wait_with_timeout(context):
            return _wait_for_spawn_service(context, timeout=timeout)

        launch_items.append(OpaqueFunction(function=_wait_with_timeout))

    launch_items.extend([
        spawn_tb3,
        tool_server,
    ])

    return LaunchDescription(launch_items)
