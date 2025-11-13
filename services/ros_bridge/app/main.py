import asyncio
import time
from typing import Optional, Dict, Any

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

# Lazy import rclpy to allow module to be imported before ROS is fully ready
import threading

app = FastAPI(title="ROS2 Bridge", version="0.1.0")

# ----------------------------
# Data Models
# ----------------------------
class JsonCommand(BaseModel):
    action: str
    parameters: Dict[str, Any] = {}

class CommandResult(BaseModel):
    status: str
    message: str
    snapshot: Dict[str, Any]

# ----------------------------
# RAP Helpers (non-SLAM actions)
# ----------------------------
LOCATIONS = {
    "gym": {
        "position": {"x": 1.9517535073729964, "y": 4.359393291484201, "z": 0.0},
        "orientation": {
            "x": 1.6302566310137402e-08,
            "y": 2.9703213238180324e-08,
            "z": -0.07945176214102775,
            "w": 0.9968387118750377,
        },
    },
    "kitchen": {
        "position": {"x": 7.353217566768062, "y": -3.458078519447155, "z": 0.0},
        "orientation": {
            "x": 1.611930208234276e-08,
            "y": 2.980589390984495e-08,
            "z": -0.07325043342926793,
            "w": 0.997313578571165,
        },
    },
    "living room": {
        "position": {"x": 1.084137940689, "y": -0.383112079564818, "z": 0.0},
        "orientation": {
            "x": 3.316520260505064e-08,
            "y": 6.931688679143018e-09,
            "z": -0.8089064668855616,
            "w": 0.5879373502599718,
        },
    },
    "office": {
        "position": {"x": -4.9521764504716765, "y": -3.573205806403106, "z": 0.0},
        "orientation": {
            "x": -3.238093524948138e-08,
            "y": 9.961584542476143e-09,
            "z": 0.9923116132365216,
            "w": -0.1237645435329962,
        },
    },
    "bedroom": {
        "position": {"x": -4.002267652240865, "y": -0.060121871401907084, "z": 0.0},
        "orientation": {
            "x": -2.1636165143756515e-08,
            "y": 2.6069771799291994e-08,
            "z": 0.8980250477792399,
            "w": 0.43994433007039957,
        },
    },
}

# ----------------------------
# ROS Helper (simplified skeleton)
# ----------------------------
_ros_ready = False
_node = None
_state_lock = threading.Lock()
_last_snapshot: Dict[str, Any] = {}

# Placeholders; full implementation will wire to /summit/cmd_vel, NavigateToPose, odom, TF

def ros_thread():
    """Initialize rclpy and spin in background collecting state."""
    global _ros_ready, _node, _last_snapshot
    try:
        import rclpy
        from rclpy.node import Node
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import Twist, TransformStamped
        from tf2_ros import StaticTransformBroadcaster
        from rclpy.action import ActionClient
        # from tf2_ros import Buffer, TransformListener

        rclpy.init()
        class BridgeNode(Node):
            def __init__(self):
                super().__init__('ros_bridge_node')
                self.cmd_pub = self.create_publisher(Twist, '/summit/cmd_vel', 10)
                self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
                # Additional for RAP actions
                from nav2_msgs.action import NavigateToPose
                self.navigate_to_pose_client = ActionClient(self, NavigateToPose, '/summit/navigate_to_pose')
                self.odom_received = False  # Track if odometry has been received
                self.last_odom = None
                # Fake odom publisher
                self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
                self.tf_broadcaster = StaticTransformBroadcaster(self)
                self.timer = self.create_timer(0.1, self.publish_fake_odom)
            def odom_cb(self, msg: Odometry):
                with _state_lock:
                    self.odom_received = True
                    self.last_odom = msg
                    _last_snapshot = {
                        'stamp': self.get_clock().now().to_msg().sec,  # coarse
                        'pose': {
                            'x': msg.pose.pose.position.x,
                            'y': msg.pose.pose.position.y,
                            'z': msg.pose.pose.position.z,
                        },
                        'orientation': {
                            'x': msg.pose.pose.orientation.x,
                            'y': msg.pose.pose.orientation.y,
                            'z': msg.pose.pose.orientation.z,
                            'w': msg.pose.pose.orientation.w,
                        },
                        'linear_velocity': {
                            'x': msg.twist.twist.linear.x,
                            'y': msg.twist.twist.linear.y,
                            'z': msg.twist.twist.linear.z,
                        },
                        'angular_velocity': {
                            'x': msg.twist.twist.angular.x,
                            'y': msg.twist.twist.angular.y,
                            'z': msg.twist.twist.angular.z,
                        }
                    }
            def publish_fake_odom(self):
                """Publish fake odometry data for testing purposes."""
                with _state_lock:
                    if not self.odom_received:
                        # Publish initial fake odom and TF
                        odom_msg = Odometry()
                        odom_msg.header.stamp = self.get_clock().now().to_msg()
                        odom_msg.header.frame_id = 'odom'
                        odom_msg.child_frame_id = 'base_link'
                        odom_msg.pose.pose.position.x = 0.0
                        odom_msg.pose.pose.position.y = 0.0
                        odom_msg.pose.pose.position.z = 0.0
                        odom_msg.pose.pose.orientation.w = 1.0
                        self.odom_publisher.publish(odom_msg)
                        self.last_odom = odom_msg
                        # TF
                        t = TransformStamped()
                        t.header.stamp = self.get_clock().now().to_msg()
                        t.header.frame_id = "odom"
                        t.child_frame_id = "base_link"
                        t.transform.translation.x = 0.0
                        t.transform.translation.y = 0.0
                        t.transform.translation.z = 0.0
                        t.transform.rotation.w = 1.0
                        self.tf_broadcaster.sendTransform(t)
                        self.odom_received = True
                    else:
                        # Modify the last_odom position slightly to simulate movement
                        self.last_odom.pose.pose.position.x += 0.01
                        self.last_odom.pose.pose.position.y += 0.01
                        # Publish the modified odometry
                        self.odom_publisher.publish(self.last_odom)
                        # Also, publish a static transform between odom and base_link
                        t = TransformStamped()
                        t.header.stamp = self.get_clock().now().to_msg()
                        t.header.frame_id = "odom"
                        t.child_frame_id = "base_link"
                        t.transform.translation.x = self.last_odom.pose.pose.position.x
                        t.transform.translation.y = self.last_odom.pose.pose.position.y
                        t.transform.translation.z = self.last_odom.pose.pose.position.z
                        t.transform.rotation = self.last_odom.pose.pose.orientation
                        self.tf_broadcaster.sendTransform(t)
        _node = BridgeNode()
        _ros_ready = True
        rclpy.spin(_node)
    except Exception as e:
        print(f"ROS thread failed: {e}")
    finally:
        try:
            import rclpy
            rclpy.shutdown()
        except Exception:
            pass

threading.Thread(target=ros_thread, daemon=True).start()

# ----------------------------
# Utility
# ----------------------------
async def wait_for_pose_change(timeout: float = 3.0, interval: float = 0.1):
    """Wait until odom pose changes to reflect actual movement (or timeout)."""
    start_time = time.time()
    with _state_lock:
        baseline = _last_snapshot.get('pose') if _last_snapshot else None
    while time.time() - start_time < timeout:
        await asyncio.sleep(interval)
        with _state_lock:
            current = _last_snapshot.get('pose') if _last_snapshot else None
        if baseline and current and (
            abs(current['x'] - baseline['x']) > 1e-3 or
            abs(current['y'] - baseline['y']) > 1e-3 or
            abs(current['z'] - baseline['z']) > 1e-3
        ):
            return True
    return False

# ----------------------------
# RAP Action Functions (non-SLAM)
# ----------------------------
def send_vel(velocity: float) -> str:
    """Sets the forward velocity of the robot."""
    from geometry_msgs.msg import Twist
    twist = Twist()
    twist.linear.x = velocity
    _node.cmd_pub.publish(twist)
    return f"Velocity set to {velocity}"

def stop() -> str:
    """Stops or halts the robot by setting its velocity to zero."""
    from geometry_msgs.msg import Twist
    twist = Twist()
    _node.cmd_pub.publish(twist)
    return "Robot stopped"

def navigate_to_pose(x: float, y: float, z_orientation: float, w_orientation: float) -> str:
    """Moves the robot to an absolute position on the map."""
    from nav2_msgs.action import NavigateToPose
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "map"
    goal_msg.pose.header.stamp = _node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = x
    goal_msg.pose.pose.position.y = y
    goal_msg.pose.pose.orientation.z = z_orientation
    goal_msg.pose.pose.orientation.w = w_orientation
    _node.navigate_to_pose_client.send_goal_async(goal_msg)
    return f"Navigation goal sent to x: {x}, y: {y}, orientation_z: {z_orientation}, orientation_w: {w_orientation}."

def navigate_relative(x: float, y: float, z_orientation: float, w_orientation: float) -> str:
    """Moves the robot relative to its current position."""
    from nav2_msgs.action import NavigateToPose
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "base_link"
    goal_msg.pose.header.stamp = _node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = x
    goal_msg.pose.pose.position.y = y
    goal_msg.pose.pose.orientation.z = z_orientation
    goal_msg.pose.pose.orientation.w = w_orientation
    _node.navigate_to_pose_client.send_goal_async(goal_msg)
    return f"Relative navigation goal sent to x: {x}, y: {y}, orientation_z: {z_orientation}, orientation_w: {w_orientation}."

def get_location_names() -> str:
    """Returns a list of available location names."""
    return f"Available locations: {', '.join(LOCATIONS.keys())}"

def navigate_to_location_by_name(location_name: str) -> str:
    """Moves the robot to a predefined location by its name."""
    location_name_lower = location_name.lower()
    if location_name_lower not in LOCATIONS:
        return f"Location '{location_name}' not found. Available locations are: {', '.join(LOCATIONS.keys())}"
    loc_data = LOCATIONS[location_name_lower]
    pos = loc_data["position"]
    orient = loc_data["orientation"]
    from nav2_msgs.action import NavigateToPose
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "map"
    goal_msg.pose.header.stamp = _node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = pos["x"]
    goal_msg.pose.pose.position.y = pos["y"]
    goal_msg.pose.pose.orientation.z = orient["z"]
    goal_msg.pose.pose.orientation.w = orient["w"]
    _node.navigate_to_pose_client.send_goal_async(goal_msg)
    return f"Navigation goal sent to location '{location_name}'. Position: {pos}, Orientation: {orient}."

# ----------------------------
# Endpoints
# ----------------------------
@app.get('/health')
async def health():
    return {'ros_ready': _ros_ready, 'odom_received': _node.odom_received if _node else False}

@app.post('/command', response_model=CommandResult)
async def command(cmd: JsonCommand):
    if not _ros_ready:
        raise HTTPException(status_code=503, detail='ROS not ready yet')

    applied = False
    msg = ''
    try:
        if cmd.action == 'send_vel' and 'velocity' in cmd.parameters:
            msg = send_vel(float(cmd.parameters['velocity']))
            applied = True
        elif cmd.action == 'stop':
            msg = stop()
            applied = True
        elif cmd.action == 'navigate_to_pose' and all(k in cmd.parameters for k in ['x', 'y', 'z_orientation', 'w_orientation']):
            msg = navigate_to_pose(
                float(cmd.parameters['x']),
                float(cmd.parameters['y']),
                float(cmd.parameters['z_orientation']),
                float(cmd.parameters['w_orientation'])
            )
            applied = True
        elif cmd.action == 'navigate_relative' and all(k in cmd.parameters for k in ['x', 'y', 'z_orientation', 'w_orientation']):
            msg = navigate_relative(
                float(cmd.parameters['x']),
                float(cmd.parameters['y']),
                float(cmd.parameters['z_orientation']),
                float(cmd.parameters['w_orientation'])
            )
            applied = True
        elif cmd.action == 'get_location_names':
            msg = get_location_names()
            applied = True
        elif cmd.action == 'navigate_to_location_by_name' and 'location_name' in cmd.parameters:
            msg = navigate_to_location_by_name(str(cmd.parameters['location_name']))
            applied = True
        else:
            msg = 'Unsupported action'
    except Exception as e:
        msg = f'Failed to execute action: {e}'

    # Wait a short period to allow movement and get snapshot
    if cmd.action in ['send_vel', 'stop']:
        await wait_for_pose_change(timeout=2.0)
    with _state_lock:
        snapshot_copy = dict(_last_snapshot) if _last_snapshot else {}

    status = 'ok' if applied else 'ignored'
    return CommandResult(status=status, message=msg, snapshot=snapshot_copy)

@app.get('/snapshot')
async def snapshot():
    with _state_lock:
        return dict(_last_snapshot) if _last_snapshot else {}

