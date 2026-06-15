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
        from sensor_msgs.msg import LaserScan
        from rclpy.qos import qos_profile_sensor_data
        import rclpy

        self.node = rclpy.create_node('brain_node')
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.node.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.node.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        
        # TF Listener for LiDAR map/odom tracking
        from tf2_ros import Buffer, TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        
        self.current_pose = None  # (x, y, unwrapped_yaw)
        self.prev_wrapped_yaw = None
        self.unwrapped_yaw = 0.0
        self.odom_counter = 0
        self.current_velocity = (0.0, 0.0, 0.0)
        self.latest_scan = None
        self.last_scan_time = 0.0

        # Movement Execution state
        self.active_movement = None
        self.movement_done = threading.Event()
        self.stop_requested = False
        self.status_message = "Idle"
        self.error_message = ""
        self.success = False

    def get_logger(self):
        return self.node.get_logger()

    def get_tf_pose(self, target_frame, source_frame='base_footprint'):
        try:
            import rclpy
            t = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            pos = t.transform.translation
            ori = t.transform.rotation
            siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
            cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return (pos.x, pos.y, yaw)
        except Exception:
            return None

    def get_current_pose_best(self):
        pose = self.get_tf_pose('map', 'base_footprint')
        if pose is not None:
            return pose, 'map'
        pose = self.get_tf_pose('odom', 'base_footprint')
        if pose is not None:
            return pose, 'odom'
        return self.current_pose, 'odom_sub'

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

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.last_scan_time = time.monotonic()

    def get_front_clearance(self, cone_half_angle_deg: float = 30.0) -> float:
        if self.latest_scan is None or (time.monotonic() - self.last_scan_time) > 1.0:
            return float('inf')
        msg = self.latest_scan
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        min_dist = float('inf')
        
        limit_rad = math.radians(cone_half_angle_deg)
        threshold_rad = math.pi - limit_rad
        
        for i, dist in enumerate(msg.ranges):
            if dist < msg.range_min or dist > msg.range_max or math.isnan(dist) or math.isinf(dist):
                continue
            angle = angle_min + i * angle_inc
            # Normalize to [-pi, pi]
            angle = math.atan2(math.sin(angle), math.cos(angle))
            # The physical front of the robot is at 180 degrees (pi radians)
            if abs(angle) >= threshold_rad:
                if dist < min_dist:
                    min_dist = dist
        return min_dist

    def get_rear_clearance(self, cone_half_angle_deg: float = 30.0) -> float:
        if self.latest_scan is None or (time.monotonic() - self.last_scan_time) > 1.0:
            return float('inf')
        msg = self.latest_scan
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        min_dist = float('inf')
        
        limit_rad = math.radians(cone_half_angle_deg)
        
        for i, dist in enumerate(msg.ranges):
            if dist < msg.range_min or dist > msg.range_max or math.isnan(dist) or math.isinf(dist):
                continue
            angle = angle_min + i * angle_inc
            # Normalize to [-pi, pi]
            angle = math.atan2(math.sin(angle), math.cos(angle))
            # The physical rear of the robot is at 0 degrees in laser_link frame
            if abs(angle) <= limit_rad:
                if dist < min_dist:
                    min_dist = dist
        return min_dist

    def get_lidar_yaw_change(self, start_scan_ranges, max_angle_rad=None) -> float:
        if self.latest_scan is None or not start_scan_ranges:
            return 0.0
            
        curr_ranges = self.latest_scan.ranges
        n = len(curr_ranges)
        if n == 0 or n != len(start_scan_ranges):
            return 0.0
            
        # Pre-clean both arrays to avoid redundant nan/inf/range checks inside the nested loops
        start_clean = [
            r if (r is not None and not math.isnan(r) and not math.isinf(r) and r > 0.08) else None
            for r in start_scan_ranges
        ]
        curr_clean = [
            r if (r is not None and not math.isnan(r) and not math.isinf(r) and r > 0.08) else None
            for r in curr_ranges
        ]
        
        angle_increment = self.latest_scan.angle_increment
        if max_angle_rad is not None:
            # Search within target + 25 degrees margin
            margin_rad = math.radians(25.0)
            limit_bins = int((max_angle_rad + margin_rad) / angle_increment)
            limit_bins = max(5, min(n // 2, limit_bins))
            min_k = -limit_bins
            max_k = limit_bins
        else:
            min_k = -n // 2
            max_k = n // 2
            
        # Determine whether to use a direct fine search (for small angles) or a coarse-to-fine search (for large angles)
        search_range_size = max_k - min_k
        
        if search_range_size <= 60:
            # Direct fine search for maximum accuracy on small angles
            best_k = 0
            min_diff = float('inf')
            for k in range(min_k, max_k + 1):
                diff = 0.0
                count = 0
                for i in range(n):
                    val_start = start_clean[i]
                    val_curr = curr_clean[(i + k) % n]
                    if val_start is not None and val_curr is not None:
                        diff += abs(val_start - val_curr)
                        count += 1
                if count > n // 6:
                    avg_diff = diff / count
                    if avg_diff < min_diff:
                        min_diff = avg_diff
                        best_k = k
        else:
            # Coarse-to-fine search for speed on large angles (e.g. 180 deg checks)
            # 1. Coarse Search
            stride = 4
            down_start = start_clean[::stride]
            down_curr = curr_clean[::stride]
            nc = len(down_start)
            
            best_kc = 0
            min_diff_c = float('inf')
            
            min_kc = min_k // stride
            max_kc = max_k // stride
            min_kc = max(-nc // 2, min_kc)
            max_kc = min(nc // 2, max_kc + 1)
            
            for k in range(min_kc, max_kc):
                diff = 0.0
                count = 0
                for i in range(nc):
                    val_start = down_start[i]
                    val_curr = down_curr[(i + k) % nc]
                    if val_start is not None and val_curr is not None:
                        diff += abs(val_start - val_curr)
                        count += 1
                if count > nc // 6:
                    avg_diff = diff / count
                    if avg_diff < min_diff_c:
                        min_diff_c = avg_diff
                        best_kc = k
                        
            # 2. Fine Search
            center_k = best_kc * stride
            search_radius = stride + 2
            
            best_k = center_k
            min_diff = float('inf')
            
            for k in range(center_k - search_radius, center_k + search_radius + 1):
                diff = 0.0
                count = 0
                for i in range(n):
                    val_start = start_clean[i]
                    val_curr = curr_clean[(i + k) % n]
                    if val_start is not None and val_curr is not None:
                        diff += abs(val_start - val_curr)
                        count += 1
                if count > n // 6:
                    avg_diff = diff / count
                    if avg_diff < min_diff:
                        min_diff = avg_diff
                        best_k = k
                        
        angle_change = -best_k * angle_increment
        angle_change = math.atan2(math.sin(angle_change), math.cos(angle_change))
        return angle_change

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
        if m['type'] == 'lead':
            return
        if now >= m['deadline']:
            if m['type'] in ['hold', 'creep']:
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
        if m['type'] == 'creep':
            d = self.get_front_clearance()
            target_distance = m['target_distance']
            
            if d == float('inf'):
                if self.latest_scan is None or (time.monotonic() - self.last_scan_time) > 1.0:
                    self.publish_zero()
                    self.status_message = "Creeping: waiting for active scan..."
                    return
                error = 1.0  # Creep forward searchingly
                d_str = "clear"
            else:
                error = d - target_distance
                d_str = f"{d:.2f}m"
                
            if error > 0.02:
                speed = max(0.015, min(m['speed'], error * 0.4))
                twist = Twist()
                twist.linear.x = speed
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.status_message = f"Creeping forward: dist={d_str}, speed={speed:.3f}m/s"
            else:
                self.publish_zero()
                self.status_message = f"Creeping: close to leg/obstacle ({d_str})"
            return
        if m['type'] == 'linear':
            frame = m.get('frame', 'odom_sub')
            if frame == 'odom_sub':
                if self.current_pose is None:
                    self.status_message = "Waiting for odom feedback..."
                    return
                curr_x, curr_y, curr_yaw = self.current_pose
            else:
                tf_pose = self.get_tf_pose(frame, 'base_footprint')
                if tf_pose is None:
                    self.status_message = f"Waiting for TF frame {frame}..."
                    return
                curr_x, curr_y, curr_yaw = tf_pose
            
            start_x, start_y, start_yaw = m['start_pose']
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
            self.status_message = f"Moving ({frame}): error={error:.3f}m, speed={speed:.3f}m/s"
        elif m['type'] == 'angular':
            sign = m['sign']
            if self.latest_scan is not None:
                if 'start_scan_ranges' not in m:
                    m['start_scan_ranges'] = list(self.latest_scan.ranges)
                yaw_change = self.get_lidar_yaw_change(m['start_scan_ranges'], max_angle_rad=m['target_angle_rad'])
                rotated = sign * yaw_change
                mode_str = "LiDAR"
            else:
                if self.current_pose is None:
                    self.status_message = "Waiting for sensor feedback..."
                    return
                curr_x, curr_y, curr_yaw = self.current_pose
                start_x, start_y, start_yaw = m['start_pose']
                rotated = sign * (curr_yaw - start_yaw)
                mode_str = "odom"
            
            error = m['target_angle_rad'] - rotated
            if error <= math.radians(2.5):
                self.success = True
                self.end_movement()
                return
            speed = max(0.12, min(m['speed'], error * 1.8))
            twist = Twist()
            twist.angular.z = sign * speed
            self.cmd_pub.publish(twist)
            self.status_message = f"Rotating ({mode_str}): error={math.degrees(error):.1f}deg, speed={speed:.3f}rad/s"


def execute_single_step(node: BrainNode, direction: str, amount, speed: float = None, duration: float = None, look_interval: float = None, detect_range: float = None) -> dict:
    if direction in ["navigate", "lead"]:
        if not isinstance(amount, list) or len(amount) < 2:
            return {"ok": False, "message": f"{direction.capitalize()} amount must be a list: [x, y] or [x, y, yaw]"}
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
        start_pose, frame = node.get_current_pose_best()
        if start_pose is None:
            timeout_feedback = time.monotonic() + 2.0
            while start_pose is None:
                if time.monotonic() > timeout_feedback:
                    return {"ok": False, "message": "No position feedback from robot"}
                time.sleep(0.01)
                start_pose, frame = node.get_current_pose_best()
                
        deadline = time.monotonic() + max(5.0, (amount / max_speed) * 2.0)
        node.active_movement = {
            "type": "linear",
            "dir_vector": (dx, dy),
            "target_distance": amount,
            "speed": max_speed,
            "start_pose": start_pose,
            "frame": frame,
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
        if node.current_pose is None and node.latest_scan is None:
            timeout_feedback = time.monotonic() + 2.0
            while node.current_pose is None and node.latest_scan is None:
                if time.monotonic() > timeout_feedback:
                    return {"ok": False, "message": "No sensor feedback from robot"}
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
    elif direction == "creep":
        max_speed = speed if speed is not None else 0.05
        if max_speed <= 0:
            return {"ok": False, "message": "Speed must be positive"}
        run_duration = duration if duration is not None else 20.0
        node.movement_done.clear()
        node.stop_requested = False
        node.error_message = ""
        node.success = False
        deadline = time.monotonic() + run_duration
        node.active_movement = {
            "type": "creep",
            "target_distance": amount,
            "speed": max_speed,
            "deadline": deadline,
            "duration": run_duration
        }
        node.status_message = f"Running creep: target={amount}m, duration={run_duration}s"
        node.movement_done.wait()
        if node.success:
            return {"ok": True, "message": "Creep completed"}
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
    elif direction == "lead":
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from geometry_msgs.msg import Twist

        x = float(amount[0])
        y = float(amount[1])
        yaw_deg = float(amount[2]) if len(amount) > 2 else 0.0
        
        max_speed = speed if speed is not None else 0.4
        if max_speed <= 0:
            return {"ok": False, "message": "Speed must be positive"}

        if not hasattr(node, 'nav_client'):
            node.nav_client = ActionClient(node.node, NavigateToPose, 'navigate_to_pose')

        if not node.nav_client.wait_for_server(timeout_sec=3.0):
            return {"ok": False, "message": "NavigateToPose action server not available. Ensure map_2d_load is running."}

        node.movement_done.clear()
        node.stop_requested = False
        node.error_message = ""
        node.success = False
        node.active_movement = {
            "type": "lead",
            "target_pose": [x, y, yaw_deg]
        }

        def send_goal(gx, gy, gyaw):
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = node.node.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = gx
            goal_msg.pose.pose.position.y = gy
            yaw_rad = math.radians(gyaw)
            goal_msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
            return node.nav_client.send_goal_async(goal_msg)

        def run_sub_step(sub_dir, sub_amt, sub_spd=None):
            res = execute_single_step(node, sub_dir, sub_amt, sub_spd)
            node.active_movement = {
                "type": "lead",
                "target_pose": [x, y, yaw_deg]
            }
            return res

        def rotate_relative(angle_deg, rotate_speed=0.4):
            if node.latest_scan is None:
                direction = "rotate_left" if angle_deg > 0 else "rotate_right"
                run_sub_step(direction, abs(angle_deg), rotate_speed)
                return True
                
            start_ranges = list(node.latest_scan.ranges)
            target_rad = math.radians(abs(angle_deg))
            sign = 1.0 if angle_deg > 0 else -1.0
            
            rate = 0.05
            timeout = time.monotonic() + 10.0
            while time.monotonic() < timeout:
                if node.stop_requested:
                    return False
                    
                yaw_change = node.get_lidar_yaw_change(start_ranges, max_angle_rad=target_rad)
                rotated = sign * yaw_change
                error = target_rad - rotated
                
                if error <= math.radians(2.5):
                    break
                    
                cmd_speed = max(0.12, min(rotate_speed, error * 1.8))
                twist = Twist()
                twist.angular.z = sign * cmd_speed
                node.cmd_pub.publish(twist)
                time.sleep(rate)
                
            node.publish_zero()
            return True

        if node.current_pose is None:
            timeout_feedback = time.monotonic() + 2.0
            while node.current_pose is None:
                if time.monotonic() > timeout_feedback:
                    node.active_movement = None
                    return {"ok": False, "message": "No odometry feedback from robot"}
                time.sleep(0.01)

        detect_val = detect_range if detect_range is not None else 1.5
        look_val = look_interval if look_interval is not None else 8.0

        node.status_message = "Lead: turning 180° to find human before starting..."
        rotate_relative(180, rotate_speed=max_speed)

        node.status_message = "Lead: waiting for human to stand in front..."
        human_ready = False
        while not human_ready:
            if node.stop_requested:
                node.active_movement = None
                return {"ok": False, "message": "Lead goal stopped before starting"}
            
            clearance = node.get_front_clearance(cone_half_angle_deg=90.0)
            if clearance <= detect_val:
                human_ready = True
                node.status_message = f"Lead: human detected at {clearance:.2f}m! Starting..."
                time.sleep(1.0)
                break
                
            if clearance == float('inf'):
                node.status_message = f"Lead: waiting for human (no one in sight, target <= {detect_val:.2f}m)"
            else:
                node.status_message = f"Lead: waiting for human (closest is {clearance:.2f}m, target <= {detect_val:.2f}m)"
            
            node.publish_zero()
            time.sleep(0.1)

        node.status_message = "Lead: turning back to path..."
        rotate_relative(-180, rotate_speed=max_speed)

        node.status_message = f"Lead: starting navigation to x={x:.2f}, y={y:.2f}"
        
        outer_success = False
        goal_handle = None
        
        send_goal_future = send_goal(x, y, yaw_deg)
        while not send_goal_future.done():
            if node.stop_requested:
                break
            time.sleep(0.01)
            
        if not node.stop_requested:
            goal_handle = send_goal_future.result()
            
        if goal_handle is None or not goal_handle.accepted:
            node.active_movement = None
            return {"ok": False, "message": "Lead goal rejected by Nav2 planner"}

        get_result_future = goal_handle.get_result_async()
        last_look_time = time.monotonic()
        
        while not get_result_future.done():
            if node.stop_requested:
                goal_handle.cancel_goal_async()
                break
                
            if time.monotonic() - last_look_time > look_val:
                node.status_message = "Lead: pausing to look back..."
                cancel_future = goal_handle.cancel_goal_async()
                while not cancel_future.done():
                    time.sleep(0.01)
                
                node.publish_zero()
                time.sleep(0.5)
                
                node.status_message = "Lead: turning 180° to find human..."
                rotate_relative(180, rotate_speed=max_speed)
                
                human_present = False
                while not human_present:
                    if node.stop_requested:
                        break
                    clearance = node.get_front_clearance(cone_half_angle_deg=90.0)
                    if clearance <= detect_val:
                        human_present = True
                        node.status_message = f"Lead: human detected at {clearance:.2f}m!"
                        time.sleep(1.0)
                        break
                    
                    if clearance == float('inf'):
                        node.status_message = f"Lead: waiting for human (no one in sight, target <= {detect_val:.2f}m)"
                    else:
                        node.status_message = f"Lead: waiting for human (closest is {clearance:.2f}m, target <= {detect_val:.2f}m)"
                    
                    node.publish_zero()
                    time.sleep(0.1)
                
                node.status_message = "Lead: turning back to path..."
                rotate_relative(-180, rotate_speed=max_speed)
                
                if node.stop_requested:
                    break
                    
                node.status_message = "Lead: resuming path navigation..."
                send_goal_future = send_goal(x, y, yaw_deg)
                while not send_goal_future.done():
                    time.sleep(0.01)
                goal_handle = send_goal_future.result()
                if not goal_handle.accepted:
                    node.active_movement = None
                    return {"ok": False, "message": "Lead goal resumption failed"}
                get_result_future = goal_handle.get_result_async()
                last_look_time = time.monotonic()
                
            time.sleep(0.1)
            
        if not node.stop_requested and get_result_future.done():
            status = get_result_future.result().status
            if status == 4:
                outer_success = True
                
        node.active_movement = None
        if outer_success:
            return {"ok": True, "message": "Guided navigation completed successfully"}
        else:
            return {"ok": False, "message": "Guided navigation failed or stopped"}
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
        duration = request.get("duration")
        look_interval = request.get("look_interval")
        detect_range = request.get("detect_range")
        return execute_single_step(node, direction, amount, speed, duration, look_interval, detect_range)
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
            duration = step.get("duration")
            look_interval = step.get("look_interval")
            detect_range = step.get("detect_range")
            res = execute_single_step(node, direction, amount, speed, duration, look_interval, detect_range)
            if not res["ok"]:
                return {"ok": False, "message": f"Step {i+1} failed: {res['message']}"}
        return {"ok": True, "message": f"Sequence of {len(steps)} steps completed successfully"}
    else:
        return {"ok": False, "message": f"Unknown command '{command}'"}


def handle_client_connection(node: BrainNode, conn):
    with conn:
        try:
            data = b""
            while not data.endswith(b"\n"):
                chunk = conn.recv(1024)
                if not chunk:
                    break
                data += chunk
            if not data:
                return
            request = json.loads(data.decode('utf-8'))
            response = handle_request(node, request)
            conn.sendall((json.dumps(response) + "\n").encode('utf-8'))
        except Exception as e:
            response = {"ok": False, "message": f"Server error: {e}"}
            try:
                conn.sendall((json.dumps(response) + "\n").encode('utf-8'))
            except Exception:
                pass


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
        threading.Thread(target=handle_client_connection, args=(node, conn), daemon=True).start()


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
