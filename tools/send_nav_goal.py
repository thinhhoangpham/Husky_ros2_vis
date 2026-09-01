#!/usr/bin/env python3
"""Send one nav2 goal and report acceptance, result status, time and error.

Distinct from tools/nav_goal.py, which is the park-specific entry point (it
refuses goals outside park's terrain, since park has no ground plane - gotcha
#25). This one is world-agnostic and reports the numbers a demo step has to
record: accepted or rejected, the terminal GoalStatus, elapsed wall-clock
seconds, and the final position error against the requested goal.

The final pose is read from TF map -> base_link. Ruling D3: this stack remaps
/tf and /tf_static to /a200_0000/tf(_static), so rclpy is initialised with
explicit remaps - a default TransformListener subscribes to the global names
this stack never publishes and simply never gets a transform.

Usage:  python3 -m tools.send_nav_goal X Y [--yaw-deg 0] [--timeout 300]
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from tools.nav_goal import make_pose

NS = "a200_0000"
ACTION = f"/{NS}/navigate_to_pose"
SERVER_TIMEOUT = 10.0
GOAL_TIMEOUT = 300.0

# action_msgs/msg/GoalStatus
STATUS_NAMES = {0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING",
                4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
STATUS_SUCCEEDED = 4


class NavGoal(Node):
    def __init__(self, action: str) -> None:
        super().__init__("send_nav_goal")
        self.client = ActionClient(self, NavigateToPose, action)
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self)
        self._handle = None

    def pose(self):
        try:
            tf = self.buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        return (t.x, t.y)

    def send(self, x: float, y: float, yaw: float, timeout: float) -> int:
        if not self.client.wait_for_server(timeout_sec=SERVER_TIMEOUT):
            print(f"  FAIL: no navigate_to_pose action server within "
                  f"{SERVER_TIMEOUT:.0f} s - run tools/check_nav2_ready.py")
            return 1

        goal = NavigateToPose.Goal()
        pose = make_pose(x, y, yaw)
        pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose = pose
        print(f"  goal: ({x:.2f}, {y:.2f}) yaw {math.degrees(yaw):.1f} deg")

        started = time.time()
        fut = self.client.send_goal_async(goal)
        self._exec.spin_until_future_complete(fut, timeout_sec=SERVER_TIMEOUT)
        handle = fut.result()
        if handle is None or not handle.accepted:
            print("  accepted: no - the action server rejected the goal")
            return 1
        self._handle = handle
        print("  accepted: yes")

        res_fut = handle.get_result_async()
        self._exec.spin_until_future_complete(res_fut, timeout_sec=timeout)
        elapsed = time.time() - started
        if not res_fut.done():
            print(f"  FAIL: no result within {timeout:.0f} s; cancelling")
            self.cancel()
            return 1
        self._handle = None
        status = res_fut.result().status
        print(f"  result: {STATUS_NAMES.get(status, status)} ({status})")
        print(f"  elapsed: {elapsed:.1f} s")

        final = self.pose()
        if final is None:
            print("  WARN: no map -> base_link transform; cannot measure error")
        else:
            err = math.hypot(final[0] - x, final[1] - y)
            print(f"  final pose: ({final[0]:.3f}, {final[1]:.3f})")
            print(f"  position error: {err:.3f} m")

        if status == STATUS_SUCCEEDED:
            print("  PASS: goal reached")
            return 0
        print("  FAIL: navigation did not succeed")
        return 1

    def cancel(self) -> None:
        handle, self._handle = self._handle, None
        if handle is None or not handle.accepted:
            return
        try:
            self._exec.spin_until_future_complete(handle.cancel_goal_async(),
                                                  timeout_sec=10.0)
        except Exception as exc:                       # noqa: BLE001 - best effort
            print(f"  WARN: cancel failed: {exc}")

    def destroy_node(self) -> bool:
        self._exec.remove_node(self)
        self._exec.shutdown()
        return super().destroy_node()


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("x", type=float)
    p.add_argument("y", type=float)
    p.add_argument("--yaw-deg", type=float, default=0.0)
    p.add_argument("--action", default=ACTION)
    p.add_argument("--timeout", type=float, default=GOAL_TIMEOUT)
    p.add_argument("--namespace", default=NS)
    args = p.parse_args(argv)

    rclpy.init(args=[
        "--ros-args",
        "-r", f"/tf:=/{args.namespace}/tf",
        "-r", f"/tf_static:=/{args.namespace}/tf_static",
    ])
    node = NavGoal(args.action)
    try:
        return node.send(args.x, args.y, math.radians(args.yaw_deg), args.timeout)
    except BaseException:
        node.cancel()          # never leave nav2 driving an unowned goal
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
