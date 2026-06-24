#!/usr/bin/env python3
"""Elly OS: Unified Robot Controller & Process Switchboard.

This script runs on the Jetson Nano robot. It handles starting and stopping
sensors and drivers in screen sessions, running a persistent movement service
as a ROS 2 node, and sending movement commands/sequences to the service.
"""

import argparse
from dataclasses import dataclass
from enum import Enum
import json
import math
import os
import re
import socket
import subprocess
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

# --- Setup Constants ---
GALACTIC = "source /opt/ros/galactic/setup.bash"
MYAGV = "source ~/myagv_ros2/install/setup.bash"
ASTRA = "source ~/ros2_astra_ws/install/setup.bash"
CAMERA_COMMAND = (
    "ros2 launch orbbec_camera astra2.launch.py "
    "uvc_backend:=v4l2 color_width:=640 color_height:=480 color_fps:=10 "
    "color_format:=UYVY depth_width:=640 depth_height:=480 depth_fps:=10 "
    "enable_ir:=false enable_sync_output_accel_gyro:=false time_domain:=device"
)

SCREEN_NAMES = {
    "lidar": "lidar",
    "camera": "camera",
    "map_2d": "slam_2d",
    "map_3d": "rtabmap",
    "motion": "motion_service"
}

SERVICES = {
    "lidar": f"{ASTRA} && {MYAGV} && ros2 launch myagv_odometry myagv_active.launch.py",
    "camera": f"{ASTRA} && {CAMERA_COMMAND}",
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

TURN_DIRECTIONS = {
    "turn_left": 1.0,
    "turn_right": -1.0,
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


class FollowState(str, Enum):
    WAITING_INITIAL = "waiting_initial"
    NAVIGATING = "navigating"
    TURNING_TO_HUMAN = "turning_to_human"
    WAITING_LOOKBACK = "waiting_lookback"
    TURNING_TO_PATH = "turning_to_path"
    TURNING_AT_GOAL = "turning_at_goal"
    WAITING_AT_GOAL = "waiting_at_goal"
    COMPLETED = "completed"
    FAILED = "failed"
    STOPPED = "stopped"


class FollowEvent(str, Enum):
    HUMAN_FOUND = "human_found"
    HUMAN_TIMEOUT = "human_timeout"
    LOOK_DUE = "look_due"
    GOAL_REACHED = "goal_reached"
    NAVIGATION_FAILED = "navigation_failed"
    ROTATION_COMPLETE = "rotation_complete"
    ROTATION_FAILED = "rotation_failed"
    STOP_REQUESTED = "stop_requested"


@dataclass(frozen=True)
class FollowMeConfig:
    look_interval: float = 20.0
    detect_range: float = 0.5
    wait_timeout: float = 15.0
    scan_max_age: float = 0.8
    sector_half_angle_deg: float = 30.0
    confirm_samples: int = 3

    def validate(self) -> None:
        positive = {
            "look_interval": self.look_interval,
            "detect_range": self.detect_range,
            "wait_timeout": self.wait_timeout,
            "scan_max_age": self.scan_max_age,
            "sector_half_angle_deg": self.sector_half_angle_deg,
        }
        for name, value in positive.items():
            if value <= 0:
                raise ValueError("%s must be greater than zero" % name)
        if self.confirm_samples < 1:
            raise ValueError("confirm_samples must be at least one")
        if self.sector_half_angle_deg > 90.0:
            raise ValueError("sector_half_angle_deg must be no more than 90 degrees")


@dataclass(frozen=True)
class FollowResult:
    ok: bool
    reason: str
    message: str


class FollowStateMachine:
    def __init__(self) -> None:
        self.state = FollowState.WAITING_INITIAL

    def transition(self, event: FollowEvent) -> FollowState:
        if event == FollowEvent.STOP_REQUESTED:
            self.state = FollowState.STOPPED
            return self.state
        if event in (FollowEvent.NAVIGATION_FAILED, FollowEvent.ROTATION_FAILED):
            self.state = FollowState.FAILED
            return self.state

        transitions = {
            (FollowState.WAITING_INITIAL, FollowEvent.HUMAN_FOUND): FollowState.NAVIGATING,
            (FollowState.WAITING_INITIAL, FollowEvent.HUMAN_TIMEOUT): FollowState.FAILED,
            (FollowState.NAVIGATING, FollowEvent.LOOK_DUE): FollowState.TURNING_TO_HUMAN,
            (FollowState.NAVIGATING, FollowEvent.GOAL_REACHED): FollowState.TURNING_AT_GOAL,
            (
                FollowState.TURNING_TO_HUMAN,
                FollowEvent.ROTATION_COMPLETE,
            ): FollowState.WAITING_LOOKBACK,
            (FollowState.WAITING_LOOKBACK, FollowEvent.HUMAN_FOUND): FollowState.TURNING_TO_PATH,
            (FollowState.WAITING_LOOKBACK, FollowEvent.HUMAN_TIMEOUT): FollowState.FAILED,
            (
                FollowState.TURNING_TO_PATH,
                FollowEvent.ROTATION_COMPLETE,
            ): FollowState.NAVIGATING,
            (
                FollowState.TURNING_AT_GOAL,
                FollowEvent.ROTATION_COMPLETE,
            ): FollowState.WAITING_AT_GOAL,
            (FollowState.WAITING_AT_GOAL, FollowEvent.HUMAN_FOUND): FollowState.COMPLETED,
            (FollowState.WAITING_AT_GOAL, FollowEvent.HUMAN_TIMEOUT): FollowState.FAILED,
        }
        key = (self.state, event)
        if key not in transitions:
            raise ValueError("Invalid follow-me transition: %s + %s" % key)
        self.state = transitions[key]
        return self.state


class FollowMeRunner:
    def __init__(self, runtime, goal, config: FollowMeConfig) -> None:
        config.validate()
        self.runtime = runtime
        self.goal = goal
        self.config = config
        self.machine = FollowStateMachine()

    def _state(self, message: str) -> None:
        self.runtime.set_state(self.machine.state.value, message)

    def _stop_result(self) -> FollowResult:
        self.machine.transition(FollowEvent.STOP_REQUESTED)
        self._state("Follow-me stopped by operator")
        return FollowResult(False, "operator_stop", "Follow-me stopped by operator")

    def _rotation_failed(self, message: str) -> FollowResult:
        self.machine.transition(FollowEvent.ROTATION_FAILED)
        self._state(message)
        return FollowResult(False, "rotation_failed", message)

    def run(self) -> FollowResult:
        self._state("Waiting for a person in the visible LiDAR sector")
        found = self.runtime.wait_for_person(
            "front",
            self.config.wait_timeout,
            require_foreground=False,
        )
        if self.runtime.stop_requested():
            return self._stop_result()
        if not found:
            self.machine.transition(FollowEvent.HUMAN_TIMEOUT)
            message = "No person was detected in the visible LiDAR sector before the wait timeout"
            self._state(message)
            return FollowResult(False, "initial_human_timeout", message)

        self.machine.transition(FollowEvent.HUMAN_FOUND)
        while True:
            self._state("Navigating to the goal")
            navigation = self.runtime.start_navigation(self.goal)
            if navigation is None:
                if self.runtime.stop_requested():
                    return self._stop_result()
                self.machine.transition(FollowEvent.NAVIGATION_FAILED)
                message = "Nav2 rejected or failed to start the goal"
                self._state(message)
                return FollowResult(False, "navigation_start_failed", message)

            event, detail = self.runtime.wait_for_navigation(
                navigation,
                self.config.look_interval,
            )
            if event == FollowEvent.STOP_REQUESTED:
                self.runtime.cancel_navigation(navigation)
                return self._stop_result()
            if event == FollowEvent.NAVIGATION_FAILED:
                self.machine.transition(event)
                self._state(detail)
                return FollowResult(False, "navigation_failed", detail)
            if event == FollowEvent.GOAL_REACHED:
                self.machine.transition(event)
                if not self.runtime.complete_destination():
                    if self.runtime.stop_requested():
                        return self._stop_result()
                    self.machine.transition(FollowEvent.NAVIGATION_FAILED)
                    message = "Could not finish the move_to_point() movement"
                    self._state(message)
                    return FollowResult(False, "destination_finish_failed", message)
                self._state("Goal reached; turning around to verify the person arrived")
                if not self.runtime.rotate_180():
                    if self.runtime.stop_requested():
                        return self._stop_result()
                    return self._rotation_failed("Could not complete the goal lookback turn")
                self.machine.transition(FollowEvent.ROTATION_COMPLETE)
                self._state("Waiting for the person at the goal")
                found = self.runtime.wait_for_person(
                    "front",
                    self.config.wait_timeout,
                    require_foreground=False,
                )
                if self.runtime.stop_requested():
                    return self._stop_result()
                if not found:
                    self.machine.transition(FollowEvent.HUMAN_TIMEOUT)
                    message = "Goal reached, but no person was detected before the wait timeout"
                    self._state(message)
                    return FollowResult(False, "goal_human_timeout", message)
                self.machine.transition(FollowEvent.HUMAN_FOUND)
                message = "Goal reached and person confirmed"
                self._state(message)
                return FollowResult(True, "completed", message)

            self.machine.transition(FollowEvent.LOOK_DUE)
            self._state("Lookback interval expired; stopping Nav2")
            if not self.runtime.cancel_navigation(navigation):
                self.machine.transition(FollowEvent.NAVIGATION_FAILED)
                message = "Nav2 did not confirm goal cancellation"
                self._state(message)
                return FollowResult(False, "navigation_cancel_failed", message)

            self._state("Turning 180 degrees to check behind")
            if not self.runtime.rotate_180():
                if self.runtime.stop_requested():
                    return self._stop_result()
                return self._rotation_failed("Could not complete the periodic lookback turn")
            self.machine.transition(FollowEvent.ROTATION_COMPLETE)

            self._state("Waiting for the person behind the robot")
            found = self.runtime.wait_for_person(
                "front",
                self.config.wait_timeout,
                require_foreground=False,
            )
            if self.runtime.stop_requested():
                return self._stop_result()
            if not found:
                self.machine.transition(FollowEvent.HUMAN_TIMEOUT)
                message = "Person was not detected during the lookback wait"
                self._state(message)
                return FollowResult(False, "human_lost", message)

            self.machine.transition(FollowEvent.HUMAN_FOUND)
            self._state("Person confirmed; holding before the return turn")
            if not self.runtime.hold_stopped(0.75):
                return self._stop_result()
            self._state("Person confirmed; turning back toward the goal")
            if not self.runtime.rotate_180():
                if self.runtime.stop_requested():
                    return self._stop_result()
                return self._rotation_failed("Could not complete the return-to-path turn")
            self.machine.transition(FollowEvent.ROTATION_COMPLETE)


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
    if name == "motion":
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
                detection = res.get("detection") or {}
                if detection.get("scan_fresh"):
                    clearance = detection.get("visible_clearance_m")
                    clearance_text = (
                        "clear" if clearance is None else f"{clearance:.3f} m"
                    )
                    detected = "YES" if detection.get("presence_detected") else "no"
                    print(f"  LiDAR visible-sector clearance: {clearance_text}")
                    print(
                        "  Presence <= "
                        f"{detection.get('detect_range_m'):.3f} m: {detected}"
                    )
                else:
                    print(
                        "  LiDAR detection: unavailable "
                        f"({detection.get('reason', 'unknown')})"
                    )
                turn = res.get("last_turn")
                if turn:
                    print(
                        "  Last turn: requested "
                        f"{turn.get('requested_deg')}deg, measured "
                        f"{turn.get('measured_deg')}deg, error "
                        f"{turn.get('error_deg')}deg ({turn.get('state')})"
                    )
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
        
        self.current_pose: Optional[Tuple[float, float, float]] = None
        self.prev_wrapped_yaw: Optional[float] = None
        self.unwrapped_yaw = 0.0
        self.odom_counter = 0
        self.current_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.latest_scan: Any = None
        self.last_scan_time = 0.0

        # Movement Execution state
        self.active_movement: Optional[Dict[str, Any]] = None
        self.movement_done = threading.Event()
        self.stop_requested = False
        self.status_message = "Idle"
        self.error_message = ""
        self.success = False
        self.command_lock = threading.Lock()
        self.last_turn: Optional[Dict[str, Any]] = None
        self.nav_client: Any = None

    def get_logger(self):
        return self.node.get_logger()

    def get_tf_pose_sample(self, target_frame, source_frame='base_footprint'):
        try:
            from rclpy.time import Time
            t = self.tf_buffer.lookup_transform(target_frame, source_frame, Time())
            pos = t.transform.translation
            ori = t.transform.rotation
            siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
            cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            stamp = (t.header.stamp.sec, t.header.stamp.nanosec)
            return (pos.x, pos.y, yaw), stamp
        except Exception:
            return None

    def get_tf_pose(self, target_frame, source_frame='base_footprint'):
        sample = self.get_tf_pose_sample(target_frame, source_frame)
        return None if sample is None else sample[0]

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

    def check_front_proximity(
        self,
        target_distance: float,
        cone_half_angle_deg: float = 30.0,
    ):
        """Return the shared LiDAR clearance and proximity decision."""

        clearance = self.get_front_clearance(cone_half_angle_deg)
        return clearance, clearance <= target_distance

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
            
        # Determine whether to use a direct fine search or a coarse-to-fine search.
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

    def publish_zero_burst(self, count: int = 8, interval: float = 0.03):
        for _ in range(count):
            self.publish_zero()
            time.sleep(interval)

    def _pose_log(self, pose):
        if pose is None:
            return None
        return {
            "x": round(pose[0], 4),
            "y": round(pose[1], 4),
            "yaw_deg": round(math.degrees(pose[2]), 2),
        }

    def pose_diagnostics_snapshot(self):
        snapshot = {}
        for frame in ("map", "odom"):
            pose = self.get_tf_pose(frame, 'base_footprint')
            if pose is not None:
                snapshot[frame] = self._pose_log(pose)
        if self.current_pose is not None:
            snapshot["odom_sub"] = self._pose_log(self.current_pose)
        return snapshot

    def pose_diagnostics_delta(self, start, end):
        delta = {}
        for frame, start_pose in start.items():
            end_pose = end.get(frame)
            if not end_pose:
                continue
            dx = end_pose["x"] - start_pose["x"]
            dy = end_pose["y"] - start_pose["y"]
            yaw_delta = normalize_angle(
                math.radians(end_pose["yaw_deg"] - start_pose["yaw_deg"])
            )
            delta[frame] = {
                "dx_m": round(dx, 4),
                "dy_m": round(dy, 4),
                "closure_error_m": round(math.hypot(dx, dy), 4),
                "wrapped_yaw_delta_deg": round(math.degrees(yaw_delta), 2),
            }
        return delta

    def record_turn_sample(self, movement, now, pose, rotated, error, frame):
        last_sample = movement.get("last_sample_at", 0.0)
        if now - last_sample < 0.25:
            return
        sample = {
            "t_s": round(now - movement.get("turn_start_monotonic", now), 3),
            "frame": frame,
            "measured_deg": round(math.degrees(rotated), 2),
            "error_deg": round(math.degrees(error), 2),
            "pose": self._pose_log(pose),
        }
        movement.setdefault("turn_samples", []).append(sample)
        if len(movement["turn_samples"]) > 120:
            movement["turn_samples"] = movement["turn_samples"][-120:]
        movement["last_sample_at"] = now

        last_log = movement.get("last_ros_log_at", 0.0)
        if now - last_log >= 1.0:
            self.get_logger().info("turn_progress " + json.dumps(sample, sort_keys=True))
            movement["last_ros_log_at"] = now

    def finish_turn_diagnostics(self, movement, state, reason):
        end_snapshot = self.pose_diagnostics_snapshot()
        measured_deg = round(math.degrees(movement.get("turn_progress_rad", 0.0)), 2)
        requested_deg = round(math.degrees(movement.get("target_angle_rad", 0.0)), 2)
        report = {
            "state": state,
            "reason": reason,
            "direction": "turn_left" if movement.get("sign", 1.0) > 0 else "turn_right",
            "feedback": movement.get("feedback_frame"),
            "requested_deg": requested_deg,
            "measured_deg": measured_deg,
            "error_deg": round(requested_deg - measured_deg, 2),
            "radius_m": round(movement.get("radius", 0.0), 4),
            "speed_mps": round(movement.get("speed", 0.0), 4),
            "angular_speed_radps": round(movement.get("angular_speed", 0.0), 4),
            "finish_tolerance_deg": round(
                math.degrees(movement.get("finish_tolerance_rad", 0.0)),
                2,
            ),
            "start_pose": movement.get("pose_start", {}),
            "end_pose": end_snapshot,
            "pose_delta": self.pose_diagnostics_delta(
                movement.get("pose_start", {}),
                end_snapshot,
            ),
            "samples": movement.get("turn_samples", [])[-80:],
        }
        self.last_turn = report
        self.get_logger().info("turn_diagnostics " + json.dumps(report, sort_keys=True))
        return report

    def detection_snapshot(self, detect_range: float = 0.5):
        if self.latest_scan is None:
            return {"scan_fresh": False, "reason": "no_scan"}
        scan_age = time.monotonic() - self.last_scan_time
        if scan_age > 0.8:
            return {
                "scan_fresh": False,
                "scan_age_seconds": round(scan_age, 3),
                "reason": "stale_scan",
            }
        clearance = self.get_front_clearance(
            cone_half_angle_deg=FollowMeConfig().sector_half_angle_deg
        )
        return {
            "scan_fresh": True,
            "scan_age_seconds": round(scan_age, 3),
            "detect_range_m": detect_range,
            "visible_clearance_m": (
                None if clearance == float("inf") else round(clearance, 3)
            ),
            "presence_detected": clearance <= detect_range,
        }

    def end_movement(self):
        self.publish_zero_burst()
        self.active_movement = None
        self.status_message = "Idle"
        self.movement_done.set()

    def control_loop_cycle(self):
        from geometry_msgs.msg import Twist
        if self.active_movement is None:
            return
        now = time.monotonic()
        m = self.active_movement
        if m["type"] in ("follow_me", "navigate"):
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
        if m['type'] == 'turn':
            sign = m['sign']
            target = m['target_angle_rad']
            rotated = m.get('turn_progress_rad', 0.0)
            mode_str = "waiting"

            frame = m.get('feedback_frame', 'odom_sub')
            if frame == 'odom_sub':
                pose = self.current_pose
            else:
                pose = self.get_tf_pose(frame, 'base_footprint')

            if pose is not None:
                yaw = pose[2]
                if m.get('turn_prev_yaw') is None:
                    m['turn_prev_yaw'] = yaw
                    m['turn_unwrapped_yaw'] = yaw
                    m['turn_start_yaw'] = yaw
                else:
                    unwrapped = unwrap_angle(
                        m['turn_prev_yaw'],
                        yaw,
                        m['turn_unwrapped_yaw'],
                    )
                    m['turn_prev_yaw'] = yaw
                    m['turn_unwrapped_yaw'] = unwrapped
                    rotated = max(0.0, sign * (unwrapped - m['turn_start_yaw']))
                    m['turn_progress_rad'] = rotated
                mode_str = frame

            error = target - rotated
            if error <= m.get('finish_tolerance_rad', math.radians(8.0)):
                self.success = True
                self.publish_zero_burst(count=16, interval=0.015)
                self.end_movement()
                return

            linear_speed = m['speed']
            angular_speed = m['angular_speed']
            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = sign * angular_speed
            self.cmd_pub.publish(twist)
            direction = "left" if sign > 0 else "right"
            self.status_message = (
                f"Turning {direction} ({mode_str}): "
                f"error={math.degrees(error):.1f}deg, "
                f"linear={linear_speed:.3f}m/s, angular={angular_speed:.3f}rad/s"
            )
            return
        if m['type'] == 'creep':
            target_distance = m['target_distance']
            d, target_reached = self.check_front_proximity(target_distance)
            
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
                
            if not target_reached and error > 0.02:
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


class BrainFollowRuntime:
    """ROS/Nav2 adapter used by the ROS-independent follow-me runner."""

    def __init__(
        self,
        node: BrainNode,
        config: FollowMeConfig,
        rotation_steps=None,
        destination_steps=None,
    ):
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient

        self.node = node
        self.config = config
        self.rotation_steps = list(rotation_steps or [
            {"direction": "rotate_left", "amount": 180.0, "speed": 1.0},
            {"direction": "stop", "amount": 1.0},
        ])
        self.destination_steps = list(destination_steps or [])
        self.current_phase = "waiting_initial"
        self.active_navigation: Any = None
        if node.nav_client is None:
            node.nav_client = ActionClient(node.node, NavigateToPose, "navigate_to_pose")

    def set_state(self, phase: str, message: str):
        self.current_phase = phase
        self.node.status_message = "Follow-me [%s]: %s" % (phase, message)
        if isinstance(self.node.active_movement, dict):
            self.node.active_movement["phase"] = phase
            self.node.active_movement["detail"] = message

    def stop_requested(self) -> bool:
        return self.node.stop_requested

    def hold_stopped(self, duration: float) -> bool:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if self.stop_requested():
                self.node.publish_zero_burst()
                return False
            self.node.publish_zero()
            time.sleep(0.05)
        self.node.publish_zero_burst()
        return True

    def _fresh_scan(self):
        if self.node.latest_scan is None:
            return None
        if time.monotonic() - self.node.last_scan_time > self.config.scan_max_age:
            return None
        return self.node.latest_scan

    def wait_for_person(
        self,
        sector: str,
        timeout: float,
        require_foreground: bool,
    ) -> bool:
        deadline = time.monotonic() + timeout
        last_sample_time = None

        if require_foreground:
            raise ValueError("Foreground-baseline detection is not supported")

        consecutive = 0
        last_sample_time = None
        while time.monotonic() < deadline:
            if self.stop_requested():
                return False
            if self._fresh_scan() is None:
                consecutive = 0
                remaining = max(0.0, deadline - time.monotonic())
                self.set_state(
                    self.current_phase,
                    "Waiting for fresh LiDAR data (%.1fs remaining)" % remaining,
                )
                self.node.publish_zero()
                time.sleep(0.05)
                continue
            if self.node.last_scan_time == last_sample_time:
                time.sleep(0.02)
                continue
            last_sample_time = self.node.last_scan_time

            clearance, person_present = self.node.check_front_proximity(
                self.config.detect_range,
                self.config.sector_half_angle_deg,
            )
            if person_present:
                consecutive += 1
                remaining = max(0.0, deadline - time.monotonic())
                self.set_state(
                    self.current_phase,
                    (
                        "Presence at %.2fm "
                        "(confirming %d/%d, %.1fs remaining)"
                    )
                    % (
                        clearance,
                        consecutive,
                        self.config.confirm_samples,
                        remaining,
                    ),
                )
                if consecutive >= self.config.confirm_samples:
                    return True
            else:
                consecutive = 0
                remaining = max(0.0, deadline - time.monotonic())
                self.set_state(
                    self.current_phase,
                    "No presence within %.2fm (%.1fs remaining)"
                    % (self.config.detect_range, remaining),
                )
            self.node.publish_zero()
        return False

    def start_navigation(self, goal):
        from nav2_msgs.action import NavigateToPose

        if not self.node.nav_client.wait_for_server(timeout_sec=3.0):
            return None
        x, y, yaw_deg = goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.node.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        yaw_rad = math.radians(yaw_deg)
        goal_msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        future = self.node.nav_client.send_goal_async(goal_msg)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return None
        handle = future.result()
        if handle is None or not handle.accepted:
            return None
        navigation = {
            "handle": handle,
            "result": handle.get_result_async(),
        }
        self.active_navigation = navigation
        if self.stop_requested():
            self.cancel_navigation(navigation)
            return None
        return navigation

    def wait_for_navigation(self, navigation, look_interval: float):
        from action_msgs.msg import GoalStatus

        deadline = time.monotonic() + look_interval
        result_future = navigation["result"]
        while True:
            if self.stop_requested():
                return FollowEvent.STOP_REQUESTED, "Follow-me stopped by operator"
            if result_future.done():
                result = result_future.result()
                self.active_navigation = None
                if result.status == GoalStatus.STATUS_SUCCEEDED:
                    self.node.publish_zero_burst()
                    return FollowEvent.GOAL_REACHED, "Navigation goal reached"
                return (
                    FollowEvent.NAVIGATION_FAILED,
                    "Nav2 finished with status code %s" % result.status,
                )
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return FollowEvent.LOOK_DUE, "Lookback interval expired"
            self.set_state(
                self.current_phase,
                "Navigating; next person check in %.1fs" % remaining,
            )
            time.sleep(0.1)

    def cancel_navigation(self, navigation) -> bool:
        from action_msgs.msg import GoalStatus

        accepted = False
        for _ in range(2):
            cancel_future = navigation["handle"].cancel_goal_async()
            deadline = time.monotonic() + 3.0
            while not cancel_future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            if cancel_future.done():
                response = cancel_future.result()
                accepted = bool(response and response.goals_canceling)
                if accepted:
                    break

        result_future = navigation["result"]
        result_deadline = time.monotonic() + 3.0
        while not result_future.done() and time.monotonic() < result_deadline:
            time.sleep(0.02)
        if result_future.done():
            accepted = accepted or result_future.result().status == GoalStatus.STATUS_CANCELED
        if accepted:
            self.active_navigation = None
        self.node.publish_zero_burst()
        return accepted

    def shutdown(self):
        if self.active_navigation is not None:
            try:
                self.cancel_navigation(self.active_navigation)
            except Exception:
                pass
        self.node.publish_zero_burst()

    def _run_recipe(self, steps, detail: str) -> bool:
        for step in steps:
            if self.stop_requested():
                return False
            self.set_state(self.current_phase, detail)
            options = {}
            if step.get("speed") is not None:
                options["speed"] = step["speed"]
            if step.get("duration") is not None:
                options["duration"] = step["duration"]
            if step.get("radius") is not None:
                options["radius"] = step["radius"]
            if step.get("finish_tolerance") is not None:
                options["finish_tolerance"] = step["finish_tolerance"]
            result = execute_single_step(
                self.node,
                step.get("direction"),
                step.get("amount"),
                **options,
            )
            was_stopped = self.node.stop_requested
            self.node.active_movement = {
                "type": "follow_me",
                "phase": self.current_phase,
                "detail": "",
                "detect_range": self.config.detect_range,
            }
            if was_stopped:
                self.node.stop_requested = True
            if not result.get("ok"):
                return False
        return True

    def rotate_180(self) -> bool:
        """Run the rotate_180() recipe supplied by movements.py."""

        completed = self._run_recipe(
            self.rotation_steps,
            "Running rotate_180() movement recipe",
        )
        self.node.last_turn = {
            "requested_deg": 180.0,
            "measured_deg": None,
            "error_deg": None,
            "feedback": "rotate_180() movement recipe",
            "state": "completed" if completed else "failed",
        }
        return completed

    def complete_destination(self) -> bool:
        """Run the steps after navigate in move_to_point()."""

        return self._run_recipe(
            self.destination_steps[1:],
            "Finishing move_to_point() movement recipe",
        )


def execute_follow_me(
    node: BrainNode,
    amount: Any,
    speed: Optional[float] = None,
    look_interval: Optional[float] = None,
    detect_range: Optional[float] = None,
    wait_timeout: Optional[float] = None,
    rotation_steps: Optional[List[Dict[str, Any]]] = None,
    destination_steps: Optional[List[Dict[str, Any]]] = None,
) -> Dict[str, Any]:
    destination_steps = list(destination_steps or [
        {"direction": "navigate", "amount": amount},
    ])
    if not destination_steps or destination_steps[0].get("direction") != "navigate":
        return {
            "ok": False,
            "message": "move_to_point() must begin with a navigate step",
        }
    amount = destination_steps[0].get("amount")
    if not isinstance(amount, list) or len(amount) < 2:
        return {
            "ok": False,
            "message": "move_to_point() navigate amount must be [x, y, yaw]",
        }
    x = float(amount[0])
    y = float(amount[1])
    yaw_deg = float(amount[2]) if len(amount) > 2 else 0.0
    config = FollowMeConfig(
        look_interval=look_interval if look_interval is not None else 20.0,
        detect_range=detect_range if detect_range is not None else 0.5,
        wait_timeout=wait_timeout if wait_timeout is not None else 15.0,
    )
    try:
        config.validate()
    except ValueError as error:
        return {"ok": False, "message": str(error)}

    node.movement_done.clear()
    node.stop_requested = False
    node.error_message = ""
    node.success = False
    node.active_movement = {
        "type": "follow_me",
        "phase": "waiting_initial",
        "target_pose": [x, y, yaw_deg],
        "detail": "",
        "detect_range": config.detect_range,
    }
    runtime = None
    try:
        runtime = BrainFollowRuntime(
            node,
            config,
            rotation_steps=rotation_steps,
            destination_steps=destination_steps,
        )
        result = FollowMeRunner(runtime, (x, y, yaw_deg), config).run()
        node.success = result.ok
        if not result.ok:
            node.error_message = result.message
        return {"ok": result.ok, "message": result.message, "reason": result.reason}
    except Exception as error:
        node.error_message = "%s: %s" % (type(error).__name__, error)
        return {
            "ok": False,
            "message": "Follow-me controller failed: %s" % node.error_message,
            "reason": "unhandled_exception",
        }
    finally:
        if runtime is not None:
            runtime.shutdown()
        else:
            node.publish_zero_burst()
        node.active_movement = None
        node.movement_done.set()
        if node.success:
            node.status_message = "Follow-me completed"


def execute_single_step(
    node: BrainNode,
    direction: str,
    amount: Any,
    speed: Optional[float] = None,
    duration: Optional[float] = None,
    look_interval: Optional[float] = None,
    detect_range: Optional[float] = None,
    wait_timeout: Optional[float] = None,
    rotation_steps: Optional[List[Dict[str, Any]]] = None,
    destination_steps: Optional[List[Dict[str, Any]]] = None,
    radius: Optional[float] = None,
    finish_tolerance: Optional[float] = None,
) -> Dict[str, Any]:
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
    elif direction in TURN_DIRECTIONS:
        sign = TURN_DIRECTIONS[direction]
        max_speed = speed if speed is not None else 0.2
        turn_radius = radius if radius is not None else 0.25
        if max_speed <= 0:
            return {"ok": False, "message": "Speed must be positive"}
        if turn_radius <= 0:
            return {"ok": False, "message": "Turn radius must be positive"}
        node.movement_done.clear()
        node.stop_requested = False
        node.error_message = ""
        node.success = False
        target_rad = math.radians(amount)
        angular_speed = max_speed / turn_radius
        finish_tolerance_rad = math.radians(
            finish_tolerance if finish_tolerance is not None else 8.0
        )
        start_pose, feedback_frame = node.get_current_pose_best()
        if start_pose is None:
            timeout_feedback = time.monotonic() + 2.0
            while start_pose is None:
                if time.monotonic() > timeout_feedback:
                    return {"ok": False, "message": "No turn pose feedback from map/odom"}
                time.sleep(0.01)
                start_pose, feedback_frame = node.get_current_pose_best()
        expected_duration = target_rad / angular_speed
        deadline = time.monotonic() + max(8.0, expected_duration * 4.0)
        node.active_movement = {
            "type": "turn",
            "sign": sign,
            "target_angle_rad": target_rad,
            "speed": max_speed,
            "angular_speed": angular_speed,
            "radius": turn_radius,
            "finish_tolerance_rad": finish_tolerance_rad,
            "feedback_frame": feedback_frame,
            "turn_start_yaw": start_pose[2],
            "turn_prev_yaw": start_pose[2],
            "turn_unwrapped_yaw": start_pose[2],
            "turn_progress_rad": 0.0,
            "deadline": deadline,
        }
        node.status_message = (
            f"Running turn: {direction} {amount}deg radius={turn_radius:.2f}m with {feedback_frame} feedback"
        )
        node.movement_done.wait()
        if node.success:
            return {"ok": True, "message": "Turn completed"}
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
        from action_msgs.msg import GoalStatus

        x = float(amount[0])
        y = float(amount[1])
        yaw_deg = float(amount[2]) if len(amount) > 2 else 0.0

        node.stop_requested = False
        node.error_message = ""
        node.active_movement = {
            "type": "navigate",
            "phase": "navigating",
            "target_pose": [x, y, yaw_deg],
        }
        runtime = BrainFollowRuntime(node, FollowMeConfig())
        try:
            navigation = runtime.start_navigation((x, y, yaw_deg))
            if navigation is None:
                if node.stop_requested:
                    return {"ok": False, "message": "Navigation stopped by operator"}
                return {
                    "ok": False,
                    "message": "NavigateToPose action server unavailable or goal rejected",
                }

            node.status_message = (
                f"Navigating to x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.1f}deg"
            )
            result_future = navigation["result"]
            while not result_future.done():
                if node.stop_requested:
                    runtime.cancel_navigation(navigation)
                    return {"ok": False, "message": "Navigation stopped by operator"}
                time.sleep(0.05)

            status = result_future.result().status
            runtime.active_navigation = None
            if status == GoalStatus.STATUS_SUCCEEDED:
                return {"ok": True, "message": "Navigation completed"}
            return {
                "ok": False,
                "message": "Navigation failed with status code %s" % status,
            }
        finally:
            runtime.shutdown()
            node.active_movement = None
    elif direction == "lead":
        return execute_follow_me(
            node,
            amount,
            speed=speed,
            look_interval=look_interval,
            detect_range=detect_range,
            wait_timeout=wait_timeout,
            rotation_steps=rotation_steps,
            destination_steps=destination_steps,
        )
    else:
        return {"ok": False, "message": f"Unknown direction '{direction}'"}


def handle_request(node: BrainNode, request: Dict[str, Any]) -> Dict[str, Any]:
    command = request.get("command")
    if command == "status":
        active = node.active_movement is not None
        movement = node.active_movement if isinstance(node.active_movement, dict) else {}
        detect_range = movement.get("detect_range", 0.5)
        return {
            "ok": True,
            "active": active,
            "status": node.status_message,
            "phase": movement.get("phase"),
            "target_pose": movement.get("target_pose"),
            "detection": node.detection_snapshot(detect_range),
            "last_turn": node.last_turn,
            "error": node.error_message,
        }
    elif command == "stop":
        if node.active_movement is not None:
            node.stop_requested = True
            return {"ok": True, "message": "Stop command sent"}
        else:
            node.publish_zero_burst()
            return {"ok": True, "message": "Already idle"}
    elif command == "move":
        if not node.command_lock.acquire(blocking=False):
            return {"ok": False, "message": "Robot is busy"}
        try:
            direction = request.get("direction")
            if not isinstance(direction, str):
                return {"ok": False, "message": "Direction must be a string"}
            amount = request.get("amount")
            speed = request.get("speed")
            duration = request.get("duration")
            look_interval = request.get("look_interval")
            detect_range = request.get("detect_range")
            wait_timeout = request.get("wait_timeout", request.get("deadtime"))
            radius = request.get("radius")
            finish_tolerance = request.get("finish_tolerance")
            return execute_single_step(
                node,
                direction,
                amount,
                speed,
                duration,
                look_interval,
                detect_range,
                wait_timeout,
                request.get("rotation_steps"),
                request.get("destination_steps"),
                radius,
                finish_tolerance,
            )
        finally:
            node.command_lock.release()
    elif command == "sequence":
        if not node.command_lock.acquire(blocking=False):
            return {"ok": False, "message": "Robot is busy"}
        try:
            steps = request.get("steps")
            if not isinstance(steps, list):
                return {"ok": False, "message": "Steps must be a list"}
            for i, step in enumerate(steps):
                if not isinstance(step, dict):
                    return {"ok": False, "message": f"Step {i+1} must be an object"}
                direction = step.get("direction")
                if not isinstance(direction, str):
                    return {"ok": False, "message": f"Step {i+1} direction must be a string"}
                amount = step.get("amount")
                speed = step.get("speed")
                duration = step.get("duration")
                look_interval = step.get("look_interval")
                detect_range = step.get("detect_range")
                wait_timeout = step.get("wait_timeout", step.get("deadtime"))
                radius = step.get("radius")
                finish_tolerance = step.get("finish_tolerance")
                res = execute_single_step(
                    node,
                    direction,
                    amount,
                    speed,
                    duration,
                    look_interval,
                    detect_range,
                    wait_timeout,
                    step.get("rotation_steps"),
                    step.get("destination_steps"),
                    radius,
                    finish_tolerance,
                )
                if not res["ok"]:
                    return {"ok": False, "message": f"Step {i+1} failed: {res['message']}"}
            return {"ok": True, "message": f"Sequence of {len(steps)} steps completed successfully"}
        finally:
            node.command_lock.release()
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
    payload: Dict[str, Any] = {"command": command}
    if command == "move":
        payload["direction"] = args.direction
        payload["amount"] = args.amount
        if args.speed is not None:
            payload["speed"] = args.speed
        if getattr(args, "radius", None) is not None:
            payload["radius"] = args.radius
        if getattr(args, "finish_tolerance", None) is not None:
            payload["finish_tolerance"] = args.finish_tolerance
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
    move_parser.add_argument("radius", type=float, nargs="?", default=None, help="Optional turn radius for turn_left/turn_right")
    move_parser.add_argument("finish_tolerance", type=float, nargs="?", default=None, help="Optional turn finish tolerance in degrees")

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
