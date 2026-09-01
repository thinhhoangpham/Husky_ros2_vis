#!/usr/bin/env python3
"""Drive one bounded segment - a straight leg or an in-place turn - with
direction-aware clearance gating.

This is the manual driving primitive behind DEMO.md's "Demo: warehouse SLAM
mapping run" Step 2: one segment per invocation, closed loop on
platform/odom/filtered, aborted the moment the sector the robot is moving
through is blocked. No collision monitor runs in that configuration, so this
gate is the only obstacle protection.

All the decision logic lives in tools/drive_geometry.py, which is ROS-free and
unit-tested; this file is the ROS plumbing around it. Read that module's
docstring for why the clearance check is direction-aware and why turn progress
is accumulated unwrapped.

Usage:
    python3 -m tools.drive_segment forward 3.0
    python3 -m tools.drive_segment reverse 1.0
    python3 -m tools.drive_segment turn_left 3.1416
    python3 -m tools.drive_segment turn_right 90 --degrees

Exit codes: 0 segment completed, 1 aborted on clearance or timed out, 2 usage.
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from tools.drive_geometry import MODES, TurnProgress, clearance_check, scan_points

NS = "/a200_0000"
CMD_TOPIC = f"{NS}/cmd_vel"                       # TwistStamped (gotcha #3)
ODOM_TOPIC = f"{NS}/platform/odom/filtered"
SCAN_TOPIC = f"{NS}/sensors/lidar2d_0/scan"

LINEAR_SPEED = 0.5                                # m/s, as driven on 2026-08-31
ANGULAR_SPEED = 0.4                               # rad/s
CMD_HZ = 20.0
ACQUIRE_TIMEOUT = 10.0                            # s to see first scan + odom
STOP_SECONDS = 0.5                                # zero-velocity tail
TIMEOUT_SLACK = 3.0                               # multiple of the ideal duration


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class SegmentDriver(Node):
    def __init__(self, args) -> None:
        super().__init__("drive_segment")
        self.args = args
        self.scan = None
        self.odom = None
        self.create_subscription(LaserScan, args.scan_topic, self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, args.odom_topic, self._on_odom,
                                 qos_profile_sensor_data)
        self.cmd = self.create_publisher(TwistStamped, args.cmd_topic, 10)

    # --- inputs ------------------------------------------------------------
    def _on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def _on_odom(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.odom = (p.x, p.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    def _points(self):
        s = self.scan
        return list(scan_points(s.ranges, s.angle_min, s.angle_increment,
                                s.range_min, s.range_max))

    # --- output ------------------------------------------------------------
    def _publish(self, vx: float, wz: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = vx
        msg.twist.angular.z = wz
        self.cmd.publish(msg)

    def _halt(self) -> None:
        """Zero velocity for a short tail so the last non-zero command cannot
        be the one left latched if this process exits."""
        deadline = time.time() + STOP_SECONDS
        while time.time() < deadline:
            self._publish(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=1.0 / CMD_HZ)

    # --- bounded waits -----------------------------------------------------
    def acquire(self, timeout: float = ACQUIRE_TIMEOUT) -> bool:
        """Wait on the real signals - one scan and one odom - under a deadline.

        A deadline on a received message, never a fixed sleep (CLAUDE.md
        Workflow).
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.scan is not None and self.odom is not None:
                return True
        return False

    # --- the segment -------------------------------------------------------
    def run(self) -> int:
        a = self.args
        turning = a.mode in ("turn_left", "turn_right")
        ideal = a.amount / (a.angular_speed if turning else a.linear_speed)
        timeout = ideal * TIMEOUT_SLACK + 2.0

        x0, y0, yaw0 = self.odom
        turn = TurnProgress(yaw0)
        progress = 0.0
        min_seen = math.inf
        started = time.time()

        vx = 0.0 if turning else (-a.linear_speed if a.mode == "reverse"
                                  else a.linear_speed)
        wz = 0.0 if not turning else (a.angular_speed if a.mode == "turn_left"
                                      else -a.angular_speed)

        unit = "rad" if turning else "m"
        print(f"  segment: {a.mode} {a.amount:.4f} {unit} "
              f"(timeout {timeout:.1f} s)")

        outcome, reason = "timeout", f"did not finish within {timeout:.1f} s"
        while time.time() - started < timeout:
            rclpy.spin_once(self, timeout_sec=1.0 / CMD_HZ)

            allowed, clearance, why = clearance_check(self._points(), a.mode)
            min_seen = min(min_seen, clearance)
            if not allowed:
                outcome, reason = "abort", why
                break

            x, y, yaw = self.odom
            if turning:
                turn.update(yaw)
                progress = turn.magnitude
            else:
                progress = math.hypot(x - x0, y - y0)
            if progress >= a.amount:
                outcome, reason = "done", "reached the commanded amount"
                break

            self._publish(vx, wz)

        self._halt()

        x, y, yaw = self.odom
        print(f"  end odom pose: ({x:.2f}, {y:.2f}) yaw {yaw:.3f}")
        print(f"  progress: {progress:.3f} {unit} of {a.amount:.3f}")
        print(f"  min clearance seen (in the swept sector): "
              f"{'n/a' if math.isinf(min_seen) else f'{min_seen:.2f} m'}")
        if outcome == "done":
            print(f"  PASS: {reason}")
            return 0
        print(f"  {'ABORTED' if outcome == 'abort' else 'TIMEOUT'}: {reason}")
        return 1


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("mode", choices=MODES)
    p.add_argument("amount", type=float,
                   help="metres for forward/reverse, radians for a turn")
    p.add_argument("--degrees", action="store_true",
                   help="interpret a turn amount in degrees")
    p.add_argument("--linear-speed", type=float, default=LINEAR_SPEED)
    p.add_argument("--angular-speed", type=float, default=ANGULAR_SPEED)
    p.add_argument("--namespace", default=NS)
    p.add_argument("--cmd-topic", default=CMD_TOPIC)
    p.add_argument("--odom-topic", default=ODOM_TOPIC)
    p.add_argument("--scan-topic", default=SCAN_TOPIC)
    args = p.parse_args(argv)
    if args.degrees:
        args.amount = math.radians(args.amount)
    if args.amount <= 0.0:
        p.error("amount must be positive")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    rclpy.init()
    node = SegmentDriver(args)
    try:
        if not node.acquire():
            missing = [n for n, v in (("scan", node.scan), ("odom", node.odom))
                       if v is None]
            print(f"  FAIL: no {' and no '.join(missing)} within "
                  f"{ACQUIRE_TIMEOUT:.0f} s - is the sim up?")
            return 1
        return node.run()
    except BaseException:
        # run() halts on every path it returns from; this covers the paths it
        # does not return from, so no non-zero command is ever left latched.
        node._halt()
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
