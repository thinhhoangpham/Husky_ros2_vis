#!/usr/bin/env python3
"""Prove the local costmap avoids a tree the prior map does not know about.

The 15 arbol4 small trees are deliberately excluded from the prior map
(tools/generate_park_maps.py), so the global planner routes straight through
them. This sends a goal whose straight-line path passes through one, then
measures the robot's closest approach while it drives.

Success means: the goal was reached AND the robot never came closer to the
tree than its radius plus a margin - i.e. it went around, not through.

Usage:  python3 -m tools.check_local_avoidance      # from the repo root
"""

import math
import sys

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from tools.nav_goal import GoalSender, make_pose

# arbolpartes4_clone_2, from worlds/park.sdf. 3.10 m across -> 1.55 m radius.
TREE = (5.67, 4.58)
TREE_RADIUS = 1.55
ROBOT_HALF_WIDTH = 0.34
MIN_CLEARANCE = TREE_RADIUS + ROBOT_HALF_WIDTH        # 1.89 m

# Chosen so the straight line from the spawn (45.64, 0.02) passes through TREE.
GOAL = (-25.0, 8.08)


class Tracker(Node):
    def __init__(self) -> None:
        super().__init__("check_local_avoidance")
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.closest = float("inf")
        self.samples = 0
        self.create_timer(0.2, self._sample)

    def _sample(self) -> None:
        try:
            tf = self.buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return
        t = tf.transform.translation
        d = math.hypot(t.x - TREE[0], t.y - TREE[1])
        self.closest = min(self.closest, d)
        self.samples += 1


def main() -> int:
    # Ruling D3: this stack remaps /tf and /tf_static to /a200_0000/tf(_static);
    # a default TransformListener only sees global /tf, which this stack never
    # publishes, so remap at node construction (same technique as
    # check_nav2_ready.py / check_map_alignment.py).
    rclpy.init(args=[
        "--ros-args",
        "-r", "/tf:=/a200_0000/tf",
        "-r", "/tf_static:=/a200_0000/tf_static",
    ])
    tracker = Tracker()
    sender = GoalSender()

    print(f"  unmapped tree at {TREE}, radius {TREE_RADIUS} m")
    print(f"  goal {GOAL} - straight line from the spawn passes through it")
    print(f"  required clearance: {MIN_CLEARANCE:.2f} m\n")

    # Drive the goal on the sender while the tracker samples pose in parallel.
    import threading
    result = {}

    def run() -> None:
        try:
            result["rc"] = sender.send(make_pose(GOAL[0], GOAL[1], 0.0))
        except Exception as exc:                       # noqa: BLE001
            result["exc"] = exc

    th = threading.Thread(target=run, daemon=True)
    th.start()
    while th.is_alive():
        rclpy.spin_once(tracker, timeout_sec=0.2)
    th.join()

    # A goal that was accepted but never awaited would keep the robot driving
    # after this process exits, silently changing the start state of the next
    # run. No-op on the success path.
    if "exc" in result:
        print(f"  ERROR in the goal sender: {result['exc']}")
    sender.cancel()

    rc = result.get("rc", 1)
    print(f"\n  pose samples: {tracker.samples}")
    print(f"  closest approach to the tree: {tracker.closest:.2f} m")

    tracker.destroy_node()
    sender.destroy_node()
    rclpy.shutdown()

    if tracker.samples < 10:
        print("  FAIL: too few pose samples; did the robot move at all?")
        return 1
    if rc != 0:
        print("  FAIL: navigation did not reach the goal")
        return 1
    if tracker.closest < MIN_CLEARANCE:
        print(f"  FAIL: drove within {tracker.closest:.2f} m of the tree "
              f"(need {MIN_CLEARANCE:.2f} m) - it was not avoided")
        return 1
    print("  PASS: goal reached and the unmapped tree was avoided")
    return 0


if __name__ == "__main__":
    sys.exit(main())
