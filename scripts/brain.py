#!/usr/bin/env python3
"""Elly OS: Unified Robot Controller & Process Switchboard.

This script runs on the Jetson Nano robot. It handles starting and stopping
sensors and drivers in screen sessions, running a persistent movement service
as a ROS 2 node, and sending movement commands/sequences to the service.
"""

import argparse
import json
import math
import os
import re
import socket
import subprocess
import sys
import threading
import time

# --- Setup Constants ---
GALACTIC = "source /opt/ros/galactic/setup.bash"
MYAGV = "source ~/myagv_ros2/install/setup.bash"
ASTRA = "source ~/ros2_astra_ws/install/setup.bash"

SCREEN_NAMES = {
    "base": "base",
    "lidar": "lidar",
    "camera": "camera",
    "map_2d": "slam_2d",
    "map_3d": "rtabmap",
    "motion": "motion_service"
}

SERVICES = {
    "base": f"{GALACTIC} && {MYAGV} && ros2 run myagv_odometry myagv_odometry_node",
    "lidar": f"{ASTRA} && {MYAGV} && ros2 launch myagv_odometry myagv_active.launch.py",
    "camera": f"{ASTRA} && ros2 launch orbbec_camera astra2_updated.launch.py",
    "map_2d": f"{GALACTIC} && {MYAGV} && ros2 run slam_gmapping slam_gmapping --ros-args -p use_sim_time:=false",
    "map_3d": f"{GALACTIC} && {MYAGV} && ros2 launch rtabmap_ros rtabmap.launch.py frame_id:=base_footprint subscribe_scan:=true scan_topic:=/scan subscribe_depth:=true approx_sync:=true rgb_topic:=/camera/color/image_raw depth_topic:=/camera/depth/image_raw camera_info_topic:=/camera/color/camera_info visual_odometry:=false odom_topic:=/odometry/filtered queue_size:=50 qos_scan:=2 rtabmapviz:=false",
    "motion": f"{GALACTIC} && {MYAGV} && python3 ~/scripts/brain.py service"
}

LINEAR_DIRECTIONS = {
    "forward": (1.0, 0.0),
    "fwd": (1.0, 0.0),
    "back": (-1.0, 0.0),
    "backward": (-1.0, 0.0),
    "bwd": (-1.0, 0.0),
    "left": (0.0, 1.0),
    "right": (0.0, -1.0),
}

ANGULAR_DIRECTIONS = {
    "rotate_left": 1.0,
    "left_turn": 1.0,
    "ccw": 1.0,
    "rotate_right": -1.0,
    "right_turn": -1.0,
    "cw": -1.0,
}

HOLD_DIRECTIONS = {"stop", "hold"}


# --- Math Helpers ---
def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def unwrap_angle(previous_wrapped: float, current_wrapped: float, previous_unwrapped: float) -> float:
    return previous_unwrapped + normalize_angle(current_wrapped - previous_wrapped)


# --- Screen Process Management ---
def list_screens() -> str:
    try:
        res = subprocess.run(["screen", "-ls"], capture_output=True, text=True)
        return res.stdout
    except Exception:
        return ""


def is_screen_running(name: str) -> bool:
    output = list_screens()
    return re.search(r'\b\d+\.' + re.escape(name) + r'\b', output) is not None


def start_service(name: str) -> int:
    if name not in SCREEN_NAMES:
        print(f"[x] Error: Unknown service '{name}'", file=sys.stderr)
        return 1
    screen_name = SCREEN_NAMES[name]
    if is_screen_running(screen_name):
        print(f"[i] Service '{name}' (screen '{screen_name}') is already running.")
        return 0
    cmd = SERVICES[name]
    full_cmd = f"screen -dmS {screen_name} bash -c '{cmd}'"
    subprocess.run(full_cmd, shell=True, executable="/bin/bash")
    print(f"[ok] Started service '{name}' in screen '{screen_name}'.")
    return 0


def publish_emergency_stop():
    cmd = f"source /opt/ros/galactic/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{{linear: {{x: 0.0, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.0}}}}'"
    subprocess.run(cmd, shell=True, executable="/bin/bash", capture_output=True)


def stop_service(name: str) -> int:
    if name not in SCREEN_NAMES:
        print(f"[x] Error: Unknown service '{name}'", file=sys.stderr)
        return 1
    screen_name = SCREEN_NAMES[name]
    if not is_screen_running(screen_name):
        print(f"[i] Service '{name}' (screen '{screen_name}') is not running.")
        return 0
    if name == "motion":
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                client.settimeout(0.5)
                client.connect("/tmp/elly_motion.sock")
                client.sendall((json.dumps({"command": "stop"}) + "\n").encode('utf-8'))
        except Exception:
            pass
    subprocess.run(f"screen -XS {screen_name} quit", shell=True)
    if name in ["base", "motion"]:
        publish_emergency_stop()
    print(f"[ok] Stopped service '{name}' (screen '{screen_name}').")
    return 0


def print_status(socket_path: str):
    print("Elly OS System Status:")
    print("----------------------")
    for name, screen_name in SCREEN_NAMES.items():
        running = is_screen_running(screen_name)
        status_str = "RUNNING" if running else "STOPPED"
        print(f"  Service {name:<8} (screen: {screen_name:<15}) : {status_str}")
    print("\nMovement Service Connection:")
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(1.0)
            client.connect(socket_path)
            client.sendall((json.dumps({"command": "status"}) + "\n").encode('utf-8'))
            res = json.loads(client.recv(1024).decode('utf-8').strip())
            if res.get("ok"):
                active = "Busy" if res.get("active") else "Idle"
                print(f"  State: {active}")
                print(f"  Info : {res.get('status')}")
                if res.get("error"):
                    print(f"  Last Error: {res.get('error')}")
            else:
                print(f"  Error querying service: {res.get('message')}")
    except Exception:
        print("  Service is unreachable (offline)")


# --- ROS 2 Node & Controller Implementation ---
class BrainNode:
    """A clean ROS 2 node wrapper that runs a simple closed-loop movement loop."""

    def __init__(self):
        # Local import to prevent dependency issues when running scripts locally without ROS
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        import rclpy

        self.node = rclpy.create_node('brain_node')
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.node.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.current_pose = None  # (x, y, unwrapped_yaw)
        self.prev_wrapped_yaw = None
        self.unwrapped_yaw = 0.0
        self.odom_counter = 0
        self.current_velocity = (0.0, 0.0, 0.0)

        # Movement Execution state
        self.active_movement = None
        self.movement_done = threading.Event()
        self.stop_requested = False
        self.status_message = "Idle"
        self.error_message = ""
        self.success = False

    def get_logger(self):
        return self.node.get_logger()

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        wrapped_yaw = math.atan2(siny_cosp, cosy_cosp)
        if self.prev_wrapped_yaw is None:
            self.unwrapped_yaw = wrapped_yaw
        else:
            diff = wrapped_yaw - self.prev_wrapped_yaw
            diff = math.atan2(math.sin(diff), math.cos(diff))
            self.unwrapped_yaw += diff
        self.prev_wrapped_yaw = wrapped_yaw
        self.current_pose = (pos.x, pos.y, self.unwrapped_yaw)
        self.odom_counter += 1
        self.current_velocity = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z)

    def publish_zero(self):
        from geometry_msgs.msg import Twist
        self.cmd_pub.publish(Twist())

    def end_movement(self):
        self.publish_zero()
        self.active_movement = None
        self.status_message = "Idle"
        self.movement_done.set()

    def control_loop_cycle(self):
        from geometry_msgs.msg import Twist
        if self.active_movement is None:
            return
        now = time.monotonic()
        m = self.active_movement
        if now >= m['deadline']:
            if m['type'] == 'hold':
                self.success = True
            else:
                self.error_message = "Movement timed out"
                self.success = False
            self.end_movement()
            return
        if self.stop_requested:
            self.error_message = "Movement stopped by operator"
            self.success = False
            self.end_movement()
            return
        if m['type'] == 'hold':
            self.publish_zero()
            self.status_message = f"Holding: {m['deadline'] - now:.2f}s remaining"
            return
        if self.current_pose is None:
            self.status_message = "Waiting for odom feedback..."
            return
        curr_x, curr_y, curr_yaw = self.current_pose
        start_x, start_y, start_yaw = m['start_pose']
        if m['type'] == 'linear':
            tx = (curr_x - start_x) * math.cos(start_yaw) + (curr_y - start_y) * math.sin(start_yaw)
            ty = -(curr_x - start_x) * math.sin(start_yaw) + (curr_y - start_y) * math.cos(start_yaw)
            dx, dy = m['dir_vector']
            proj = dx * tx + dy * ty
            error = m['target_distance'] - proj
            if error <= 0.02:
                self.success = True
                self.end_movement()
                return
            speed = max(0.02, min(m['speed'], error * 1.5))
            twist = Twist()
            twist.linear.x = dx * speed
            twist.linear.y = dy * speed
            self.cmd_pub.publish(twist)
            self.status_message = f"Moving: error={error:.3f}m, speed={speed:.3f}m/s"
        elif m['type'] == 'angular':
            sign = m['sign']
            rotated = sign * (curr_yaw - start_yaw)
            error = m['target_angle_rad'] - rotated
            if error <= math.radians(2.0):
                self.success = True
                self.end_movement()
                return
            speed = max(0.1, min(m['speed'], error * 2.0))
            twist = Twist()
            twist.angular.z = sign * speed
            self.cmd_pub.publish(twist)
            self.status_message = f"Rotating: error={math.degrees(error):.1f}deg, speed={speed:.3f}rad/s"


def execute_single_step(node: BrainNode, direction: str, amount, speed: float = None) -> dict:
    if direction == "navigate":
        if not isinstance(amount, list) or len(amount) < 2:
            return {"ok": False, "message": "Navigate amount must be a list: [x, y] or [x, y, yaw]"}
    else:
        if amount is None or isinstance(amount, list) or amount <= 0:
            return {"ok": False, "message": "Amount must be positive"}
    if direction in LINEAR_DIRECTIONS:
        dx, dy = LINEAR_DIRECTIONS[direction]
        max_speed = speed if speed is not None else 0.1
        if max_speed <= 0:
            return {"ok": False, "message": "Speed must be positive"}
        node.movement_done.clear()
        node.stop_requested = False
        node.error_message = ""
        node.success = False
        if node.current_pose is None:
            timeout_feedback = time.monotonic() + 2.0
            while node.current_pose is None:
                if time.monotonic() > timeout_feedback:
                    return {"ok": False, "message": "No odometry feedback from robot"}
                time.sleep(0.01)
        start_pose = node.current_pose
        deadline = time.monotonic() + max(5.0, (amount / max_speed) * 2.0)
        node.active_movement = {
            "type": "linear",
            "dir_vector": (dx, dy),
            "target_distance": amount,
            "speed": max_speed,
            "start_pose": start_pose,
            "deadline": deadline
        }
        node.status_message = f"Running linear: {direction} {amount}m"
        node.movement_done.wait()
        if node.success:
            return {"ok": True, "message": "Movement completed"}
        else:
            return {"ok": False, "message": node.error_message}
    elif direction in ANGULAR_DIRECTIONS:
        sign = ANGULAR_DIRECTIONS[direction]
        max_speed = speed if speed is not None else 0.3
        if max_speed <= 0:
            return {"ok": False, "message": "Speed must be positive"}
        node.movement_done.clear()
        node.stop_requested = False
        node.error_message = ""
        node.success = False
        if node.current_pose is None:
            timeout_feedback = time.monotonic() + 2.0
            while node.current_pose is None:
                if time.monotonic() > timeout_feedback:
                    return {"ok": False, "message": "No odometry feedback from robot"}
                time.sleep(0.01)
        start_pose = node.current_pose
        target_rad = math.radians(amount)
        deadline = time.monotonic() + max(5.0, (target_rad / max_speed) * 2.0)
        node.active_movement = {
            "type": "angular",
            "sign": sign,
            "target_angle_rad": target_rad,
            "speed": max_speed,
            "start_pose": start_pose,
            "deadline": deadline
        }
        node.status_message = f"Running angular: {direction} {amount}deg"
        node.movement_done.wait()
        if node.success:
            return {"ok": True, "message": "Rotation completed"}
        else:
            return {"ok": False, "message": node.error_message}
    elif direction in HOLD_DIRECTIONS:
        node.movement_done.clear()
        node.stop_requested = False
        node.error_message = ""
        node.success = False
        deadline = time.monotonic() + amount
        node.active_movement = {
            "type": "hold",
            "deadline": deadline,
            "duration": amount
        }
        node.status_message = f"Running hold: {amount}s"
        node.movement_done.wait()
        if node.success:
            return {"ok": True, "message": "Hold completed"}
        else:
            return {"ok": False, "message": node.error_message}
    elif direction == "navigate":
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        
        x = float(amount[0])
        y = float(amount[1])
        yaw_deg = float(amount[2]) if len(amount) > 2 else 0.0
        
        if not hasattr(node, 'nav_client'):
            node.nav_client = ActionClient(node.node, NavigateToPose, 'navigate_to_pose')
            
        if not node.nav_client.wait_for_server(timeout_sec=3.0):
            return {"ok": False, "message": "NavigateToPose action server not available. Ensure map_2d_load is running."}
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = node.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        
        yaw_rad = math.radians(yaw_deg)
        goal_msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        
        node.status_message = f"Navigating to x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.1f}°"
        
        send_goal_future = node.nav_client.send_goal_async(goal_msg)
        while not send_goal_future.done():
            time.sleep(0.01)
            
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return {"ok": False, "message": "Navigation goal rejected by Nav2 planner"}
            
        get_result_future = goal_handle.get_result_async()
        while not get_result_future.done():
            time.sleep(0.05)
            
        status = get_result_future.result().status
        if status == 4: # GoalStatus.STATUS_SUCCEEDED
            return {"ok": True, "message": "Navigation completed"}
        else:
            return {"ok": False, "message": f"Navigation failed with status code {status}"}
    else:
        return {"ok": False, "message": f"Unknown direction '{direction}'"}


def handle_request(node: BrainNode, request: dict) -> dict:
    command = request.get("command")
    if command == "status":
        active = node.active_movement is not None
        return {
            "ok": True,
            "active": active,
            "status": node.status_message,
            "error": node.error_message
        }
    elif command == "stop":
        if node.active_movement is not None:
            node.stop_requested = True
            return {"ok": True, "message": "Stop command sent"}
        else:
            node.publish_zero()
            return {"ok": True, "message": "Already idle"}
    elif command == "move":
        if node.active_movement is not None:
            return {"ok": False, "message": "Robot is busy"}
        direction = request.get("direction")
        amount = request.get("amount")
        speed = request.get("speed")
        return execute_single_step(node, direction, amount, speed)
    elif command == "sequence":
        if node.active_movement is not None:
            return {"ok": False, "message": "Robot is busy"}
        steps = request.get("steps")
        if not isinstance(steps, list):
            return {"ok": False, "message": "Steps must be a list"}
        for i, step in enumerate(steps):
            direction = step.get("direction")
            amount = step.get("amount")
            speed = step.get("speed")
            res = execute_single_step(node, direction, amount, speed)
            if not res["ok"]:
                return {"ok": False, "message": f"Step {i+1} failed: {res['message']}"}
        return {"ok": True, "message": f"Sequence of {len(steps)} steps completed successfully"}
    else:
        return {"ok": False, "message": f"Unknown command '{command}'"}


def socket_server(node: BrainNode, socket_path: str):
    if os.path.exists(socket_path):
        os.unlink(socket_path)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(socket_path)
    os.chmod(socket_path, 0o600)
    server.listen(5)
    while True:
        try:
            conn, _ = server.accept()
        except Exception:
            break
        with conn:
            try:
                data = b""
                while not data.endswith(b"\n"):
                    chunk = conn.recv(1024)
                    if not chunk:
                        break
                    data += chunk
                if not data:
                    continue
                request = json.loads(data.decode('utf-8'))
                response = handle_request(node, request)
                conn.sendall((json.dumps(response) + "\n").encode('utf-8'))
            except Exception as e:
                response = {"ok": False, "message": f"Server error: {e}"}
                try:
                    conn.sendall((json.dumps(response) + "\n").encode('utf-8'))
                except Exception:
                    pass


def run_service(socket_path: str):
    import rclpy
    rclpy.init()
    node = BrainNode()
    server_thread = threading.Thread(target=socket_server, args=(node, socket_path), daemon=True)
    server_thread.start()
    node.get_logger().info("Elly movement service initialized")
    try:
        while rclpy.ok():
            rclpy.spin_once(node.node, timeout_sec=0.02)
            node.control_loop_cycle()
    finally:
        node.publish_zero()
        node.node.destroy_node()
        rclpy.shutdown()


# --- Socket Client Implementation ---
def run_client(socket_path: str, command: str, args) -> int:
    payload = {"command": command}
    if command == "move":
        payload["direction"] = args.direction
        payload["amount"] = args.amount
        if args.speed is not None:
            payload["speed"] = args.speed
    elif command == "sequence":
        data = args.data.strip()
        try:
            if os.path.isfile(data):
                with open(data, 'r') as f:
                    steps = json.load(f)
            else:
                steps = json.loads(data)
        except Exception as e:
            print(f"[x] Error: Invalid JSON input or file path: {e}", file=sys.stderr)
            return 2
        if not isinstance(steps, list):
            print("[x] Error: JSON sequence must be a list of steps", file=sys.stderr)
            return 2
        payload["steps"] = steps

    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(600.0)  # Large timeout for multi-step execution
            client.connect(socket_path)
            client.sendall((json.dumps(payload) + "\n").encode('utf-8'))
            response = b""
            while not response.endswith(b"\n"):
                chunk = client.recv(1024)
                if not chunk:
                    break
                response += chunk
        if not response:
            print("[x] Error: No response from movement service", file=sys.stderr)
            return 1
        res = json.loads(response.decode('utf-8'))
        if res.get("ok"):
            print(f"[ok] {res.get('message')}")
            return 0
        else:
            print(f"[x] Error: {res.get('message')}", file=sys.stderr)
            return 1
    except Exception as e:
        print(f"[x] Error: Connection failed: {e}", file=sys.stderr)
        return 1


# --- CLI Main ---
def main():
    parser = argparse.ArgumentParser(description="Elly OS: Unified Robot Controller & Process Switchboard")
    subparsers = parser.add_subparsers(dest="command", required=True)

    start_parser = subparsers.add_parser("start", help="Start a ROS service")
    start_parser.add_argument("service", choices=list(SCREEN_NAMES.keys()))

    stop_parser = subparsers.add_parser("stop", help="Stop a ROS service")
    stop_parser.add_argument("service", choices=list(SCREEN_NAMES.keys()))

    subparsers.add_parser("status", help="Show system status")
    subparsers.add_parser("service", help="Run the movement socket service (ROS 2 Node)")

    move_parser = subparsers.add_parser("move", help="Send a movement command to the service")
    move_parser.add_argument("direction", help="Direction of movement (forward, back, left, right, rotate_left, rotate_right, ccw, cw)")
    move_parser.add_argument("amount", type=float, help="Distance in meters or angle in degrees")
    move_parser.add_argument("speed", type=float, nargs="?", default=None, help="Optional speed (m/s or rad/s)")

    sequence_parser = subparsers.add_parser("sequence", help="Send a sequence of movements to the service")
    sequence_parser.add_argument("data", help="JSON string or file path containing the movement sequence list")

    subparsers.add_parser("stop_motion", help="Abort any active movement")

    args = parser.parse_args()
    socket_path = "/tmp/elly_motion.sock"

    if args.command == "start":
        sys.exit(start_service(args.service))
    elif args.command == "stop":
        sys.exit(stop_service(args.service))
    elif args.command == "status":
        print_status(socket_path)
    elif args.command == "service":
        run_service(socket_path)
    elif args.command == "move":
        sys.exit(run_client(socket_path, "move", args))
    elif args.command == "sequence":
        sys.exit(run_client(socket_path, "sequence", args))
    elif args.command == "stop_motion":
        sys.exit(run_client(socket_path, "stop", None))


if __name__ == "__main__":
    main()
