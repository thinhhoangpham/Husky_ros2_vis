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
# Must match launch/park_stock.launch.py's LIFECYCLE_NODES exactly, in order:
# a single lifecycle_manager_navigation brings all ten up, map nodes
# first (ruling D4, 2026-08-26 - the former separate lifecycle_manager_maps
# raced nav2's own manager and stalled with map_server inactive while
# planner_server went active). Extended 2026-08-26 after collision_monitor's
# absence from this list let a READY report pass with the cmd_vel output
# chain silently broken downstream of controller_server (task 6 fix cycle,
# defect A).
# Trimmed 2026-08-27: route_server, smoother_server and docking_server were
# removed from the launch file as unused.
LIFECYCLE = ["map_server",
             "controller_server", "planner_server", "behavior_server",
             "velocity_smoother", "collision_monitor", "bt_navigator",
             "waypoint_follower"]


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


def _lifecycle_states_from_clients(nodes: list[str], clients: dict,
                                    spin_until_future_complete,
                                    make_request,
                                    first_service_timeout: float,
                                    per_service_timeout: float) -> dict[str, str]:
    """Pure core of get_lifecycle_states, decoupled from rclpy/lifecycle_msgs
    so it can be unit tested without ROS. `clients` maps node name -> an
    object exposing `.wait_for_service(timeout_sec)` and `.call_async(req)`
    (a real rclpy Client, or a fake with the same shape in tests).
    `spin_until_future_complete(future, timeout_sec)` drives the future to
    completion; `make_request()` builds a fresh GetState.Request (or fake).

    Root cause of the false-negative this replaces: get_lifecycle_states
    used to call client.call_async(...) with no preceding wait_for_service.
    A brand-new rclpy participant has not finished DDS discovery of these
    services yet, so the request went out before the service existed and the
    future never completed - every node read "unavailable" even when nav2
    was healthy and active within 3-4s. wait_for_service blocks (bounded, not
    a sleep) until discovery completes or the timeout expires, so the
    call_async that follows is only issued once the service is actually
    reachable.

    The first node pays the FULL cost of DDS graph discovery for this brand
    new participant (rclpy.init() + node creation happened just before this
    is called); every later node reuses that same discovered graph, so it
    only needs a short budget to catch normal per-service jitter.

    Returns {node_name: state_label}; a node whose service is never
    discovered within its wait_for_service budget, or whose GetState call
    never responds within `per_service_timeout`, gets the label
    "unavailable" - same label as before this fix, so callers and printed
    output are unchanged.
    """
    states = {}
    for i, n in enumerate(nodes):
        client = clients[n]
        wait_budget = first_service_timeout if i == 0 else per_service_timeout
        if not client.wait_for_service(timeout_sec=wait_budget):
            states[n] = "unavailable"
            continue
        future = client.call_async(make_request())
        spin_until_future_complete(future, timeout_sec=per_service_timeout)
        result = future.result()
        states[n] = result.current_state.label if result is not None else "unavailable"
    return states


def get_lifecycle_states(nodes: list[str] = LIFECYCLE, ns: str = NS,
                          per_service_timeout: float = 3.0,
                          first_service_timeout: float = 10.0) -> dict[str, str]:
    """Query GetState on every lifecycle node's own service from a SINGLE
    rclpy process/node, instead of shelling out `ros2 service call` once per
    node.

    Root cause of the original false-negative this replaced: a bare `ros2`
    CLI invocation costs ~1.4s of process startup on this machine, paid again
    for every one of the 10 lifecycle nodes. Paying rclpy/node startup exactly
    once and reusing it for all 10 GetState calls removes that startup cost
    from the per-node budget entirely.

    See _lifecycle_states_from_clients for the wait_for_service fix and the
    reasoning behind the two timeout budgets.
    """
    import rclpy
    from rclpy.node import Node
    from lifecycle_msgs.srv import GetState

    rclpy.init()
    node = Node("check_nav2_ready_lifecycle")
    clients = {n: node.create_client(GetState, f"{ns}/{n}/get_state") for n in nodes}

    states = _lifecycle_states_from_clients(
        nodes, clients,
        spin_until_future_complete=lambda future, timeout_sec: rclpy.spin_until_future_complete(
            node, future, timeout_sec=timeout_sec),
        make_request=GetState.Request,
        first_service_timeout=first_service_timeout,
        per_service_timeout=per_service_timeout,
    )

    node.destroy_node()
    rclpy.shutdown()
    return states


def nav_ready(sh=sh, tf_check=check_transform, lifecycle_check=get_lifecycle_states) -> list[str]:
    """Run every readiness check; return the list of failures (empty == READY)."""
    failures = []
    print("== transforms")
    if not tf_check():
        failures.append("map -> odom not published (GPS localization not up?)")
    print("== lifecycle nodes")
    # `ros2 lifecycle get` resolves node names via `ros2 node list`, which
    # returned empty here even though every node's services are live and
    # answering (verified with `ros2 service list` / a direct service call)
    # - a daemon node-discovery gap, not a lifecycle problem. Call GetState
    # on each node's own service directly instead (via a single in-process
    # rclpy node — see get_lifecycle_states), which is what actually
    # reflects whether the node is up.
    states = lifecycle_check()
    for n in LIFECYCLE:
        state = states.get(n, "unavailable")
        good = state == "active"
        print(f"  {n:30s}: {'active' if good else 'NOT ACTIVE'}")
        if not good:
            reason = (f"label='{state}'" if state != "unavailable"
                      else "no response (service unavailable within 3s)")
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
    return failures


def map_topic_published(topic_info: str) -> bool:
    """Pure check over `ros2 topic info -v` output for a topic with exactly
    one publisher. Split out of slam_ready so it is unit-testable without
    ROS, same shape as the other pure gates in this file."""
    return "Publisher count: 1" in topic_info


def slam_ready(sh=sh, tf_check=check_transform) -> list[str]:
    """Readiness gate for slam_toolbox mapping mode (launch/park_slam.launch.py),
    used instead of nav_ready(). nav_ready()'s lifecycle-node, action-server
    and costmap checks all name nav2 nodes (map_server, planner_server, ...)
    that simply do not exist under slam_toolbox - reusing it as-is would
    report every one of them "unavailable" forever, not READY.

    slam_toolbox's contract (see park_slam.launch.py's docstring) is: it
    publishes map -> odom itself, and it serves the live-built map on
    <ns>/map. Both are checked directly; nothing else in this stack is
    running in slam mode to check.
    """
    failures = []
    print("== transforms")
    if not tf_check():
        failures.append("map -> odom not published (slam_toolbox not up?)")
    print("== map topic")
    info = sh(f"ros2 topic info -v {NS}/map 2>/dev/null")
    ok = map_topic_published(info)
    print(f"  {NS}/map : {'OK' if ok else 'no publisher'}")
    if not ok:
        failures.append(f"{NS}/map has no publisher")
    return failures


def main() -> int:
    failures = nav_ready()
    if failures:
        print("\nNOT READY:")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("\nREADY")
    return 0


if __name__ == "__main__":
    sys.exit(main())
