#!/usr/bin/env python3
"""Programmatic Navigation Client for Elly.

Usage:
  elly_navigate <x> <y> [yaw_deg]
"""

import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class EllyNavigator(Node):
    def __init__(self):
        super().__init__('elly_navigator')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x: float, y: float, yaw_deg: float):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        
        yaw_rad = math.radians(yaw_deg)
        goal_msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        print(f"[i] Waiting for NavigateToPose action server...")
        self._action_client.wait_for_server()
        
        print(f"[i] Sending goal: x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.1f}°")
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("\n[x] Goal rejected by Nav2 planner!")
            rclpy.shutdown()
            sys.exit(1)
        print("\n[ok] Goal accepted. Navigating...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4: # GoalStatus.STATUS_SUCCEEDED
            print("\n[✔] Success! Arrived at destination.")
        else:
            print(f"\n[x] Navigation failed with status code: {status}")
        rclpy.shutdown()
        sys.exit(0)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        sys.stdout.write(f"\rDistance remaining: {feedback.distance_remaining:.2f} m   ")
        sys.stdout.flush()

def main():
    if len(sys.argv) < 3:
        print("Usage: elly_navigate <x> <y> [yaw_deg]")
        sys.exit(1)
        
    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        yaw_deg = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    except ValueError:
        print("[x] Error: Coordinates must be numbers")
        sys.exit(1)
        
    rclpy.init()
    navigator = EllyNavigator()
    navigator.send_goal(x, y, yaw_deg)
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        print("\n[i] Navigation cancelled by user.")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
