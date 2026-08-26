#!/usr/bin/env python3
"""Prove park's GPS maps world +x to increasing latitude, 1:1.

Ground truth from the original ROS 1 dataset (park_1.bag, 120 paired samples):
    North = +1.0008 * x   (residual 1 mm)
    East  = -0.9978 * y   (residual 1 mm)
    datum at world origin

Teleports the robot to two poses and compares the GPS delta against the
expected geographic delta. Run against a live park sim.

Usage:  python3 tools/check_gps_orientation.py
"""

import math
import subprocess
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

WORLD = "park"
MODEL = "a200_0000/robot"
TOPIC = "/a200_0000/sensors/gps_0/fix"

LAT0, LON0 = 49.9, 8.9
R = 6378137.0
M_PER_DEG_LAT = math.pi / 180.0 * R
M_PER_DEG_LON = math.pi / 180.0 * R * math.cos(math.radians(LAT0))

# Two poses 20 m apart in +x, on flat ground well inside the terrain.
POSE_A = (0.0, 0.0, 3.30)
POSE_B = (20.0, 0.0, 3.30)


def teleport(x: float, y: float, z: float) -> None:
    req = (f'name: "{MODEL}", position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{x: 0, y: 0, z: 0, w: 1}}')
    out = subprocess.run(
        ["gz", "service", "-s", f"/world/{WORLD}/set_pose",
         "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
         "--timeout", "3000", "--req", req],
        capture_output=True, text=True).stdout
    # gotcha #4: set_pose returns data: true even for a nonexistent entity,
    # so this only proves the service answered, not that the model moved.
    if "true" not in out:
        print(f"  FAIL: set_pose did not succeed: {out.strip()}")
        sys.exit(1)


class FixReader(Node):
    def __init__(self) -> None:
        super().__init__("check_gps_orientation")
        self.fix: NavSatFix | None = None
        self.create_subscription(NavSatFix, TOPIC, self._cb, qos_profile_sensor_data)

    def _cb(self, msg: NavSatFix) -> None:
        self.fix = msg

    def read(self, n: int = 12) -> NavSatFix:
        """GPS is 1 Hz; take several so we do not read a pre-teleport fix."""
        self.fix = None
        got = 0
        while got < n:
            rclpy.spin_once(self, timeout_sec=3.0)
            if self.fix is not None:
                got += 1
                last, self.fix = self.fix, None
        return last


def main() -> int:
    rclpy.init()
    node = FixReader()
    try:
        teleport(*POSE_A)
        a = node.read()
        teleport(*POSE_B)
        b = node.read()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    d_north = (b.latitude - a.latitude) * M_PER_DEG_LAT
    d_east = (b.longitude - a.longitude) * M_PER_DEG_LON
    dx = POSE_B[0] - POSE_A[0]

    print(f"  pose A: lat {a.latitude:.9f}  lon {a.longitude:.9f}")
    print(f"  pose B: lat {b.latitude:.9f}  lon {b.longitude:.9f}")
    print(f"  moved +{dx:.1f} m in world x")
    print(f"    -> north {d_north:+.3f} m   (expect {dx:+.1f})")
    print(f"    -> east  {d_east:+.3f} m   (expect  0.0)")

    ok = abs(d_north - dx) < 0.5 and abs(d_east) < 0.5
    if not ok and abs(d_north + dx) < 0.5:
        print("\n  FAIL: latitude moved the WRONG WAY -> flip heading_deg to -90")
        return 1
    if not ok and abs(d_east - dx) < 0.5:
        print("\n  FAIL: +x still maps to EAST -> heading_deg did not take effect")
        return 1
    if not ok:
        print("\n  FAIL: neither axis matches; check datum and heading_deg")
        return 1

    print(f"\n  datum check: lat0 {a.latitude - POSE_A[0] / M_PER_DEG_LAT:.6f} "
          f"(expect {LAT0})")
    print("  PASS: world +x maps to north, 1:1")
    return 0


if __name__ == "__main__":
    sys.exit(main())
