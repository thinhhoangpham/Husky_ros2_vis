#!/usr/bin/env python3
"""Send the Husky a navigation goal in the map frame.

Usage:  python3 tools/nav_goal.py X Y [YAW_DEG]

park's terrain spans x -50..50, y -26.55..23.45 and has NO ground plane
(CLAUDE.md gotcha #25): a goal outside it sends the robot into the void,
which presents as "the robot vanished". Goals are rejected before sending.
"""

import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

TERRAIN = (-50.0, 50.0, -26.55, 23.45)
NS = "a200_0000"


def in_terrain(x: float, y: float, margin: float = 1.0) -> bool:
    xmin, xmax, ymin, ymax = TERRAIN
    return (xmin + margin) <= x <= (xmax - margin) and (ymin + margin) <= y <= (ymax - margin)


def make_pose(x: float, y: float, yaw: float) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = "map"
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


class GoalSender(Node):
    def __init__(self) -> None:
        super().__init__("nav_goal")
        self.client = ActionClient(self, NavigateToPose, f"/{NS}/navigate_to_pose")

    def send(self, pose: PoseStamped) -> int:
        if not self.client.wait_for_server(timeout_sec=10.0):
            print("  FAIL: navigate_to_pose action server not available")
            print("        run tools/check_nav2_ready.py first")
            return 1
        goal = NavigateToPose.Goal()
        pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose = pose
        print(f"  sending goal: x {pose.pose.position.x:.2f}  y {pose.pose.position.y:.2f}")
        fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if handle is None or not handle.accepted:
            print("  FAIL: goal rejected by the action server")
            return 1
        print("  goal accepted, navigating...")
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        status = res_fut.result().status
        # action_msgs/GoalStatus: 4 = SUCCEEDED
        if status == 4:
            print("  PASS: goal reached")
            return 0
        print(f"  FAIL: navigation ended with status {status}")
        return 1


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    x, y = float(sys.argv[1]), float(sys.argv[2])
    yaw = math.radians(float(sys.argv[3])) if len(sys.argv) > 3 else 0.0
    if not in_terrain(x, y):
        print(f"  REFUSED: ({x}, {y}) is outside park's terrain {TERRAIN}")
        print("           park has no ground plane; the robot would fall.")
        return 2
    rclpy.init()
    node = GoalSender()
    try:
        return node.send(make_pose(x, y, yaw))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
