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
from rclpy.executors import SingleThreadedExecutor
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
        # A dedicated executor, not rclpy's implicit global one: check_local_avoidance
        # spins its tracker node in the main thread while send() runs in a worker,
        # and one executor entered from two threads raises "Executor is already
        # spinning". Single-node, single-thread callers are unaffected.
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self)
        self._handle = None
        self._goal_fut = None

    def send(self, pose: PoseStamped) -> int:
        if not self.client.wait_for_server(timeout_sec=10.0):
            print("  FAIL: navigate_to_pose action server not available")
            print("        run tools/check_nav2_ready.py first")
            return 1
        goal = NavigateToPose.Goal()
        pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose = pose
        print(f"  sending goal: x {pose.pose.position.x:.2f}  y {pose.pose.position.y:.2f}")
        fut = self._goal_fut = self.client.send_goal_async(goal)
        self._exec.spin_until_future_complete(fut)
        handle = fut.result()
        if handle is None or not handle.accepted:
            print("  FAIL: goal rejected by the action server")
            return 1
        self._handle, self._goal_fut = handle, None
        print("  goal accepted, navigating...")
        res_fut = handle.get_result_async()
        self._exec.spin_until_future_complete(res_fut)
        self._handle = None
        status = res_fut.result().status
        # action_msgs/GoalStatus: 4 = SUCCEEDED
        if status == 4:
            print("  PASS: goal reached")
            return 0
        print(f"  FAIL: navigation ended with status {status}")
        return 1

    def cancel(self) -> None:
        """Cancel a goal this sender accepted but never awaited to completion.

        No-op unless send() left a goal in flight (i.e. it raised between
        acceptance and the result). Without this, a crash leaves nav2 driving
        the robot unattended.
        """
        handle, self._handle = self._handle, None
        if handle is None and self._goal_fut is not None:
            # send() raised before it read the acceptance: the goal request was
            # still transmitted, so resolve the handle now rather than leave a
            # goal nav2 has accepted but nobody owns.
            fut, self._goal_fut = self._goal_fut, None
            try:
                self._exec.spin_until_future_complete(fut, timeout_sec=10.0)
                handle = fut.result()
            except Exception as exc:                   # noqa: BLE001 - best effort
                print(f"  WARN: could not resolve the in-flight goal: {exc}")
        if handle is None or not handle.accepted:
            return
        print("  cancelling the in-flight goal")
        try:
            self._exec.spin_until_future_complete(handle.cancel_goal_async(),
                                                  timeout_sec=10.0)
        except Exception as exc:                       # noqa: BLE001 - best effort
            print(f"  WARN: cancel failed: {exc}")

    def destroy_node(self) -> bool:
        self._exec.remove_node(self)
        self._exec.shutdown()
        return super().destroy_node()


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
