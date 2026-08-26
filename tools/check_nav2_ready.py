#!/usr/bin/env python3
"""Prove the nav2 stack is actually up, not merely launched.

Nav2 started before map -> odom exists comes up healthy but useless: no crash,
no error, goals silently ignored. This is the gate that catches that.

Also settles the open cmd_vel question: the Husky requires TwistStamped
(CLAUDE.md gotcha #3). If nav2 publishes plain Twist, the robot reports
success and never moves.

Ruling D3: the whole Clearpath stack remaps /tf and /tf_static to
/a200_0000/tf and /a200_0000/tf_static. tf2_ros's Python TransformListener
subscribes to the absolute, un-namespaced /tf and /tf_static by default, and
`ros2 run tf2_ros tf2_echo` does the same, so neither can see this stack's
transforms out of the box. This script builds its own rclpy node and passes
`-r /tf:=/a200_0000/tf -r /tf_static:=/a200_0000/tf_static` as ROS args at
node construction, so the TransformListener's subscriptions land on the
namespaced topics. It bounds the wait with spin_once() calls (not a sleep
loop, not `timeout` on a ros2 CLI process per gotcha #8) since a transform
lookup is inherently async.

One-shot. No sleeps, no polling loops.

Usage:  python3 tools/check_nav2_ready.py
"""

import subprocess
import sys
import time

NS = "/a200_0000"
# The maps group (lifecycle_manager_maps, this launch file's own Stage 2)
# plus every node lifecycle_manager_navigation manages, per
# nav2_bringup/launch/navigation_launch.py's `lifecycle_nodes` list. Extended
# 2026-08-26 after collision_monitor's absence from this list let a READY
# report pass with the cmd_vel output chain silently broken downstream of
# controller_server (task 6 fix cycle, defect A).
LIFECYCLE = ["map_server", "filter_mask_server", "costmap_filter_info_server",
             "controller_server", "smoother_server", "planner_server",
             "route_server", "behavior_server", "velocity_smoother",
             "collision_monitor", "bt_navigator", "waypoint_follower",
             "docking_server"]


def sh(cmd: str, timeout: float = None) -> str:
    try:
        return subprocess.run(["bash", "-lc", f"source /opt/ros/jazzy/setup.bash && {cmd}"],
                              capture_output=True, text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired as e:
        return (e.stdout or b"").decode() if isinstance(e.stdout, bytes) else (e.stdout or "")


def check_transform() -> bool:
    """One-shot map -> odom lookup via a private rclpy node with /tf remapped.

    Avoids `ros2 run tf2_ros tf2_echo`, which listens on global /tf and will
    never see this namespaced stack's transforms (ruling D3), and avoids
    killing a ros2 CLI process with `timeout` (gotcha #8).
    """
    import rclpy
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

    rclpy.init(args=[
        "--ros-args",
        "-r", "/tf:=/a200_0000/tf",
        "-r", "/tf_static:=/a200_0000/tf_static",
    ])
    node = Node("check_nav2_ready_tf")
    buf = Buffer()
    TransformListener(buf, node)

    found = False
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and not found:
        rclpy.spin_once(node, timeout_sec=0.2)
        if buf.can_transform("map", "odom", rclpy.time.Time()):
            found = True

    if found:
        try:
            t = buf.lookup_transform("map", "odom", rclpy.time.Time())
            tr = t.transform.translation
            print(f"  map -> odom  : OK  (x={tr.x:.3f} y={tr.y:.3f} z={tr.z:.3f})")
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            print(f"  map -> odom  : MISSING ({e})")
            found = False
    else:
        print("  map -> odom  : MISSING")

    node.destroy_node()
    rclpy.shutdown()
    return found


def main() -> int:
    failures = []

    print("== transforms")
    ok = check_transform()
    if not ok:
        failures.append("map -> odom not published (GPS localization not up?)")

    print("== lifecycle nodes")
    # `ros2 lifecycle get` resolves node names via `ros2 node list`, which
    # returned empty here even though every node's services are live and
    # answering (verified with `ros2 service list` / a direct service call)
    # - a daemon node-discovery gap, not a lifecycle problem. Call
    # GetState on each node's own service directly instead, which is what
    # actually reflects whether the node is up.
    for n in LIFECYCLE:
        out = sh(f"ros2 service call {NS}/{n}/get_state lifecycle_msgs/srv/GetState "
                  f"'{{}}' 2>/dev/null", timeout=5.0)
        good = "label='active'" in out
        print(f"  {n:30s}: {'active' if good else 'NOT ACTIVE'}")
        if not good:
            reason = out.strip()[:80] or "no response (service unavailable within 5s)"
            failures.append(f"{n} is not active (get_state: {reason})")

    print("== action servers")
    acts = sh("ros2 action list 2>/dev/null")
    for a in ("navigate_to_pose", "follow_waypoints"):
        ok = f"{NS}/{a}" in acts
        print(f"  {a:30s}: {'OK' if ok else 'MISSING'}")
        if not ok:
            failures.append(f"action {a} not advertised")

    print("== cmd_vel contract (gotcha #3)")
    info = sh(f"ros2 topic info -v {NS}/cmd_vel 2>/dev/null")
    stamped = "geometry_msgs/msg/TwistStamped" in info
    plain = "geometry_msgs/msg/Twist\n" in info or "geometry_msgs/msg/Twist " in info
    print(f"  TwistStamped : {'yes' if stamped else 'NO'}")
    if plain and not stamped:
        print("  !! nav2 publishes plain Twist; the Husky will never move.")
        failures.append("cmd_vel type mismatch: nav2 publishes Twist, robot needs TwistStamped")
    elif not stamped:
        failures.append("cmd_vel has no TwistStamped publisher")

    print("== costmaps")
    for t in (f"{NS}/global_costmap/costmap", f"{NS}/local_costmap/costmap"):
        n = sh(f"ros2 topic info {t} 2>/dev/null | grep -c 'Publisher count: 1'").strip()
        ok = n == "1"
        print(f"  {t.split('/')[-2]:30s}: {'OK' if ok else 'no publisher'}")
        if not ok:
            failures.append(f"{t} has no publisher")

    if failures:
        print("\nNOT READY:")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("\nREADY")
    return 0


if __name__ == "__main__":
    sys.exit(main())
