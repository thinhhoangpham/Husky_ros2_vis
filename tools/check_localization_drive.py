#!/usr/bin/env python3
"""Measure how far the localization estimate drifts from truth on a straight leg.

Drives a fixed straight leg with raw cmd_vel (no nav2), then stops and holds a
rest window, logging three positions at 10 Hz:

  * EKF map pose    - TF map -> base_link
  * raw GPS         - /a200_0000/odometry/gps (nav_msgs/Odometry, map frame)
  * Gazebo truth    - the a200_0000/robot entry of the world's dynamic pose feed

Truth is the yardstick for everything, including the leg length: the leg ends
when *truth* displacement reaches --distance, so a bad estimate cannot shorten
or lengthen the test it is being judged by.

Truth source (deviation from the design note, which said /world/<w>/pose/info):
that topic carries the static world layout and does not stream the robot, so
this uses /world/<w>/dynamic_pose/info, which does - measured ~53 Hz on park.
`gz` is only on PATH after sourcing /opt/ros/jazzy/setup.bash, so the streaming
subprocess is launched through bash -lc with that source (CLAUDE.md Workflow).

Ruling D3: the Clearpath stack remaps /tf and /tf_static to /a200_0000/tf and
/a200_0000/tf_static; tf2_ros's TransformListener subscribes to the absolute
names, so rclpy is initialised with explicit remaps (same as
check_nav2_ready.py). tf_static's TRANSIENT_LOCAL durability comes from
TransformListener itself.

cmd_vel is geometry_msgs/TwistStamped on this stack (CLAUDE.md gotcha #3).
Gazebo services and topics are world-scoped (gotcha #26), hence --world.

Usage:  python3 tools/check_localization_drive.py [--distance 15] [--speed 0.5]
"""

import argparse
import csv
import math
import os
import re
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener

NS = "/a200_0000"
MODEL = "a200_0000/robot"

# worlds/park.sdf terrain rectangle (gotcha #25: there is no ground plane, so
# leaving it means falling out of the world).
TERRAIN_X = (-50.0, 50.0)
TERRAIN_Y = (-26.55, 23.45)
TERRAIN_MARGIN = 3.0

CMD_HZ = 20.0
LOG_HZ = 10.0
ZERO_SECONDS = 3.0
REST_SECONDS = 3.0


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def ang_diff_deg(a: float, b: float) -> float:
    return math.degrees(abs(math.atan2(math.sin(a - b), math.cos(a - b))))


class TruthStream:
    """Streams the robot's world pose out of `gz topic -e` in a thread."""

    def __init__(self, world: str) -> None:
        cmd = (f"source /opt/ros/jazzy/setup.bash && "
               f"exec stdbuf -oL gz topic -e -t /world/{world}/dynamic_pose/info")
        self.proc = subprocess.Popen(["bash", "-c", cmd],
                                     stdout=subprocess.PIPE,
                                     stderr=subprocess.DEVNULL, text=True)
        self.lock = threading.Lock()
        self.pose = None            # (x, y, yaw)
        self.count = 0
        self.first_t = None
        self.last_t = None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self) -> None:
        depth = 0
        block: list[str] = []
        capturing = False
        for line in self.proc.stdout:
            s = line.strip()
            if not capturing:
                if s == "pose {":
                    capturing, depth, block = True, 1, []
                continue
            depth += line.count("{") - line.count("}")
            if depth > 0:
                block.append(line)
            else:
                capturing = False
                self._parse("".join(block))

    def _parse(self, txt: str) -> None:
        m = re.search(r'name: "([^"]*)"', txt)
        if not m or m.group(1) != MODEL:
            return
        pos = re.search(r"position \{([^}]*)\}", txt)
        ori = re.search(r"orientation \{([^}]*)\}", txt)
        if pos is None or ori is None:
            return

        def field(section: str, key: str) -> float:
            # protobuf text omits zero-valued fields (gotcha #20).
            f = re.search(rf"\b{key}: ([-0-9.e+]+)", section)
            return float(f.group(1)) if f else 0.0

        x = field(pos.group(1), "x")
        y = field(pos.group(1), "y")
        yaw = yaw_from_quat(field(ori.group(1), "x"), field(ori.group(1), "y"),
                            field(ori.group(1), "z"), field(ori.group(1), "w"))
        now = time.time()
        with self.lock:
            self.pose = (x, y, yaw)
            self.count += 1
            if self.first_t is None:
                self.first_t = now
            self.last_t = now

    def get(self):
        with self.lock:
            return self.pose

    def rate(self) -> float:
        with self.lock:
            if self.count < 2 or self.last_t is None:
                return 0.0
            span = self.last_t - self.first_t
            return (self.count - 1) / span if span > 0 else 0.0

    def stop(self) -> None:
        self.proc.terminate()


class DriveCheck(Node):
    def __init__(self, args, truth: TruthStream) -> None:
        super().__init__("check_localization_drive")
        self.args = args
        self.truth = truth
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.gps = None
        self.create_subscription(Odometry, f"{NS}/odometry/gps",
                                 self._on_gps, qos_profile_sensor_data)
        self.cmd = self.create_publisher(TwistStamped, f"{NS}/cmd_vel", 10)

        self.rows: list[dict] = []
        self.travelled = 0.0
        self.prev_truth = None
        self.phase = "drive"
        self.phase_start = time.time()
        self.t0 = self.phase_start
        self.done = False
        self.error = None

        self.create_timer(1.0 / CMD_HZ, self._tick_cmd)
        self.create_timer(1.0 / LOG_HZ, self._tick_log)

    def _on_gps(self, msg: Odometry) -> None:
        self.gps = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _publish(self, vx: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = vx
        self.cmd.publish(msg)

    def _tick_cmd(self) -> None:
        if self.done:
            return
        self._publish(self.args.speed if self.phase == "drive" else 0.0)

    def _ekf_pose(self):
        try:
            tf = self.buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    def _tick_log(self) -> None:
        if self.done:
            return
        now = time.time()
        truth = self.truth.get()
        if truth is None:
            if now - self.t0 > 10.0:
                self.error = "no truth pose from the Gazebo pose stream"
                self.done = True
            return

        if self.prev_truth is not None and self.phase == "drive":
            self.travelled += math.hypot(truth[0] - self.prev_truth[0],
                                         truth[1] - self.prev_truth[1])
        self.prev_truth = truth

        ekf = self._ekf_pose()
        row = {
            "t": now - self.t0,
            "phase": self.phase,
            "truth_x": truth[0], "truth_y": truth[1],
            "truth_yaw": math.degrees(truth[2]),
            "ekf_x": ekf[0] if ekf else "", "ekf_y": ekf[1] if ekf else "",
            "ekf_yaw": math.degrees(ekf[2]) if ekf else "",
            "gps_x": self.gps[0] if self.gps else "",
            "gps_y": self.gps[1] if self.gps else "",
            "err_ekf": math.hypot(ekf[0] - truth[0], ekf[1] - truth[1]) if ekf else "",
            "err_gps": (math.hypot(self.gps[0] - truth[0], self.gps[1] - truth[1])
                        if self.gps else ""),
            "yaw_err": ang_diff_deg(ekf[2], truth[2]) if ekf else "",
        }
        self.rows.append(row)

        if self.phase == "drive" and self.travelled >= self.args.distance:
            self.phase, self.phase_start = "zero", now
        elif self.phase == "zero" and now - self.phase_start >= ZERO_SECONDS:
            self.phase, self.phase_start = "rest", now
        elif self.phase == "rest" and now - self.phase_start >= REST_SECONDS:
            self._publish(0.0)
            self.done = True


def _stat(rows, key, fn):
    vals = [r[key] for r in rows if r[key] != ""]
    return fn(vals) if vals else float("nan")


def mean(v):
    return sum(v) / len(v)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--distance", type=float, default=15.0, help="leg length, m")
    p.add_argument("--speed", type=float, default=0.5, help="drive speed, m/s")
    p.add_argument("--world", default="park", help="Gazebo world name")
    p.add_argument("--motion-tol", type=float, default=0.5)
    p.add_argument("--rest-tol", type=float, default=0.2)
    p.add_argument("--yaw-tol", type=float, default=2.0)
    p.add_argument("--csv", default="/tmp/localization_drive.csv")
    args = p.parse_args()

    truth = TruthStream(args.world)
    rclpy.init(args=[
        "--ros-args",
        "-r", "/tf:=/a200_0000/tf",
        "-r", "/tf_static:=/a200_0000/tf_static",
    ])
    node = None
    try:
        # Safety gate: project the leg endpoint from truth and refuse to drive
        # off the terrain (gotcha #25 - there is nothing to land on).
        probe = Node("check_localization_drive_probe")
        start = None
        for _ in range(40):
            rclpy.spin_once(probe, timeout_sec=0.25)
            start = truth.get()
            if start is not None:
                break
        probe.destroy_node()
        if start is None:
            print("  FAIL: no truth pose on "
                  f"/world/{args.world}/dynamic_pose/info - is the sim up?")
            return 1
        ex = start[0] + args.distance * math.cos(start[2])
        ey = start[1] + args.distance * math.sin(start[2])
        print(f"  start (truth):  x {start[0]:+8.3f}  y {start[1]:+8.3f}  "
              f"yaw {math.degrees(start[2]):+7.2f} deg")
        print(f"  leg endpoint:   x {ex:+8.3f}  y {ey:+8.3f}  "
              f"({args.distance:.1f} m at {args.speed:.2f} m/s)")
        if not (TERRAIN_X[0] + TERRAIN_MARGIN <= ex <= TERRAIN_X[1] - TERRAIN_MARGIN
                and TERRAIN_Y[0] + TERRAIN_MARGIN <= ey <= TERRAIN_Y[1] - TERRAIN_MARGIN):
            print(f"\n  REFUSED: endpoint leaves the terrain rectangle "
                  f"x {TERRAIN_X[0]}..{TERRAIN_X[1]}, y {TERRAIN_Y[0]}..{TERRAIN_Y[1]} "
                  f"with a {TERRAIN_MARGIN:.0f} m margin. Reposition or shorten "
                  f"--distance.")
            return 2

        node = DriveCheck(args, truth)
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.error:
            print(f"\n  FAIL: {node.error}")
            return 1

        rows = node.rows
        motion = [r for r in rows if r["phase"] == "drive"]
        rest = [r for r in rows if r["phase"] == "rest"]

        with open(args.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)

        print(f"\n  truth sample rate: {truth.rate():.1f} Hz "
              f"({truth.count} samples)")
        print(f"  travelled (truth): {node.travelled:.3f} m   "
              f"log samples: {len(rows)} ({len(motion)} motion, {len(rest)} rest)")

        print("\n  t      truth x    y     |  ekf x     y    |  gps x     y    "
              "| err_ekf err_gps yaw_err")
        next_t = 0.0
        for r in rows:
            if r["t"] + 1e-9 < next_t:
                continue
            next_t = r["t"] + 1.0

            def f(v, w=7, d=2):
                return f"{v:{w}.{d}f}" if v != "" else " " * w

            print(f"  {r['t']:5.1f} {f(r['truth_x'])} {f(r['truth_y'])} | "
                  f"{f(r['ekf_x'])} {f(r['ekf_y'])} | "
                  f"{f(r['gps_x'])} {f(r['gps_y'])} | "
                  f"{f(r['err_ekf'])} {f(r['err_gps'])} {f(r['yaw_err'])}")

        ekf_motion_max = _stat(motion, "err_ekf", max)
        ekf_motion_mean = _stat(motion, "err_ekf", mean)
        ekf_rest_mean = _stat(rest, "err_ekf", mean)
        gps_motion_mean = _stat(motion, "err_gps", mean)
        yaw_motion_mean = _stat(motion, "yaw_err", mean)
        yaw_motion_max = _stat(motion, "yaw_err", max)
        yaw_rest_mean = _stat(rest, "yaw_err", mean)
        yaw_rest_max = _stat(rest, "yaw_err", max)

        print(f"\n  EKF position error, motion:  mean {ekf_motion_mean:.3f} m   "
              f"max {ekf_motion_max:.3f} m")
        print(f"  EKF position error, rest:    mean {ekf_rest_mean:.3f} m")
        print(f"  GPS raw error, motion:       mean {gps_motion_mean:.3f} m")
        print(f"  heading error, motion:       mean {yaw_motion_mean:.2f} deg  "
              f"max {yaw_motion_max:.2f} deg")
        print(f"  heading error, rest:         mean {yaw_rest_mean:.2f} deg  "
              f"max {yaw_rest_max:.2f} deg")
        print(f"  full 10 Hz log: {args.csv}")

        fails = []
        if not (ekf_motion_max <= args.motion_tol):
            fails.append(f"EKF motion error {ekf_motion_max:.3f} m "
                         f"> {args.motion_tol} m")
        if not (ekf_rest_mean <= args.rest_tol):
            fails.append(f"EKF rest error {ekf_rest_mean:.3f} m "
                         f"> {args.rest_tol} m")
        if not (yaw_motion_max <= args.yaw_tol):
            fails.append(f"heading error in motion {yaw_motion_max:.2f} deg "
                         f"> {args.yaw_tol} deg")
        if not (yaw_rest_max <= args.yaw_tol):
            fails.append(f"heading error at rest {yaw_rest_max:.2f} deg "
                         f"> {args.yaw_tol} deg")
        if fails:
            print("\n  FAIL: " + "\n        ".join(fails))
            return 1
        print("\n  PASS: localization tracks ground truth within tolerance")
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        truth.stop()


if __name__ == "__main__":
    sys.exit(main())
