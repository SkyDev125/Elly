#!/usr/bin/env python3
"""Autonomous Self-Localization & Recovery Node for Elly.

This script calls the global localization reinitialization service, then
drives the robot (rotating and translating) while using the LiDAR to actively
avoid obstacles. It monitors AMCL pose covariance and stops automatically
once the robot is successfully localized.
"""

import sys
import math
import time
from typing import Optional
import elly as elly_console
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

class EllyAutoFinder(Node):
    def __init__(self):
        super().__init__('elly_autofinder')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.reinit_client = self.create_client(Empty, '/reinitialize_global_localization')
        
        # State machine variables
        self.latest_scan = None
        self.latest_covariance = None
        self.state = 'INIT'  # INIT -> ROTATING -> TRANSLATING -> AVOID_OBSTACLE -> DONE | FAILED
        self.state_start_time = time.time()
        self.search_start_time: Optional[float] = None
        self.last_scan_time = 0.0
        self.last_print_time = 0.0
        self.turn_dir = 1.0
        
        # Configuration
        self.safe_distance = 0.40      # m (distance to trigger obstacle avoidance)
        self.cov_threshold_pos = 0.05  # covariance threshold for x, y
        self.cov_threshold_yaw = 0.05  # covariance threshold for yaw
        
        # Start control loop timer (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Trigger the initial global localization service call
        self.trigger_global_localization()

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.last_scan_time = time.time()

    def pose_callback(self, msg):
        self.latest_covariance = msg.pose.covariance

    def trigger_global_localization(self):
        print("[i] Waiting for /reinitialize_global_localization service...")
        if not self.reinit_client.wait_for_service(timeout_sec=5.0):
            print("[x] Error: Global localization service not available. Is the navigation/AMCL stack running?")
            self.state = 'FAILED'
            return
            
        req = Empty.Request()
        print("[i] Calling global localization reinitialization (re-scattering particles)...")
        self.future = self.reinit_client.call_async(req)
        self.future.add_done_callback(self.reinit_done_callback)
        
    def reinit_done_callback(self, future):
        try:
            future.result()
            print("[ok] Global localization reset. Starting search routine!")
            self.state = 'ROTATING'
            self.state_start_time = time.time()
            self.search_start_time = time.time()
        except Exception as e:
            print(f"[x] Service call failed: {e}")
            self.state = 'FAILED'

    def check_obstacle_forward(self) -> bool:
        if self.latest_scan is None or (time.time() - self.last_scan_time) > 1.0:
            return True  # Fail-safe: assume obstacle if scanner is offline/stale
            
        msg = self.latest_scan
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        
        for i, dist in enumerate(msg.ranges):
            # Skip out of range or nan values
            if dist < msg.range_min or dist > msg.range_max or math.isnan(dist) or math.isinf(dist):
                continue
                
            angle = angle_min + i * angle_inc
            # Normalize angle to [-pi, pi]
            angle = math.atan2(math.sin(angle), math.cos(angle))
            
            # Monitor front cone of -45 to +45 degrees
            if -math.radians(45) <= angle <= math.radians(45):
                if dist < self.safe_distance:
                    return True
        return False

    def choose_turn_direction(self) -> float:
        """Evaluate left vs right LiDAR scans to find the clear direction to turn."""
        if self.latest_scan is None:
            return 1.0  # Default to CCW (left)
            
        msg = self.latest_scan
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        
        left_dist_sum = 0.0
        left_count = 0
        right_dist_sum = 0.0
        right_count = 0
        
        for i, dist in enumerate(msg.ranges):
            if dist < msg.range_min or dist > msg.range_max or math.isnan(dist) or math.isinf(dist):
                continue
                
            angle = angle_min + i * angle_inc
            angle = math.atan2(math.sin(angle), math.cos(angle))
            
            # Left side (15 to 75 degrees)
            if math.radians(15) <= angle <= math.radians(75):
                left_dist_sum += dist
                left_count += 1
            # Right side (-75 to -15 degrees)
            elif -math.radians(75) <= angle <= -math.radians(-15):
                right_dist_sum += dist
                right_count += 1
                
        left_avg = left_dist_sum / left_count if left_count > 0 else 0.0
        right_avg = right_dist_sum / right_count if right_count > 0 else 0.0
        
        # Choose direction with the larger average distance (clearance)
        return 1.0 if left_avg >= right_avg else -1.0

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        if self.state == 'INIT':
            return
            
        if self.state == 'FAILED':
            print("[x] Auto-find failed.")
            self.stop_robot()
            sys.exit(1)
            
        if self.state == 'DONE':
            print("\n[✔] SUCCESS: Elly has successfully localized itself!")
            self.stop_robot()
            sys.exit(0)
            
        now = time.time()
        elapsed = now - self.state_start_time
        
        # Periodically output AMCL covariance details
        if self.latest_covariance is not None:
            cov_x = self.latest_covariance[0]
            cov_y = self.latest_covariance[7]
            cov_yaw = self.latest_covariance[35]
            
            if now - self.last_print_time > 1.5:
                status_suffix = ""
                if (
                    cov_x < self.cov_threshold_pos
                    and cov_y < self.cov_threshold_pos
                    and cov_yaw < self.cov_threshold_yaw
                    and self.search_start_time is not None
                ):
                    time_remaining = 15.0 - (now - self.search_start_time)
                    if time_remaining > 0:
                        status_suffix = f" (Converged! Verifying for {time_remaining:.1f}s...)"
                print(f"[i] Uncertainty: Position={max(cov_x, cov_y):.4f} (target < {self.cov_threshold_pos}), Yaw={cov_yaw:.4f} (target < {self.cov_threshold_yaw}){status_suffix}")
                self.last_print_time = now
                
            if cov_x < self.cov_threshold_pos and cov_y < self.cov_threshold_pos and cov_yaw < self.cov_threshold_yaw:
                if self.search_start_time is not None and (now - self.search_start_time) >= 15.0:
                    self.state = 'DONE'
                    return
        else:
            if now - self.last_print_time > 2.0:
                print("[i] Waiting for pose feedback (/amcl_pose)...")
                self.last_print_time = now
                
        # Main State Machine
        if self.state == 'ROTATING':
            # Spin to let AMCL match features 360 degrees
            twist = Twist()
            twist.angular.z = 0.5  # CCW rotation
            self.cmd_pub.publish(twist)
            
            if elapsed > 12.0:  # Spin for ~12 seconds
                self.state = 'TRANSLATING'
                self.state_start_time = now
                print("[i] Initial spin completed. Moving forward...")
                
        elif self.state == 'TRANSLATING':
            # Check for obstacles in front
            if self.check_obstacle_forward():
                print("[!] Obstacle detected in front! Scanning for safe escape angle...")
                self.stop_robot()
                self.turn_dir = self.choose_turn_direction()
                self.state = 'AVOID_OBSTACLE'
                self.state_start_time = now
            else:
                twist = Twist()
                twist.linear.x = 0.08  # Slow translation speed
                self.cmd_pub.publish(twist)
                
                if elapsed > 6.0:  # Translate for 6 seconds, then spin again
                    self.state = 'ROTATING'
                    self.state_start_time = now
                    print("[i] Moving done. Spinning to update localization context...")
                    
        elif self.state == 'AVOID_OBSTACLE':
            # Rotate in the chosen safe direction
            twist = Twist()
            twist.angular.z = self.turn_dir * 0.5
            self.cmd_pub.publish(twist)
            
            if elapsed > 3.0:  # Turn for ~3 seconds (~90 degrees)
                self.state = 'TRANSLATING'
                self.state_start_time = now
                print("[i] Obstacle cleared. Resuming translation...")

def main():
    if not elly_console.ensure_services(["lidar", "navigation"]):
        sys.exit(1)
    rclpy.init()
    node = EllyAutoFinder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[i] Auto-find interrupted by operator.")
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
