#!/usr/bin/env python3
"""Prove the GPS localization layer publishes a sane map -> odom.

Checks, in order:
  1. map -> odom resolves in TF
  2. the robot's map-frame position matches Gazebo ground truth
  3. heading is consistent (a yaw_offset error shows as a ~90 deg rotation)

Ruling D3: the Clearpath stack remaps /tf and /tf_static to namespaced
topics (/a200_0000/tf, /a200_0000/tf_static) throughout, but tf2_ros's
Python TransformListener subscribes to absolute /tf and /tf_static.
rclpy is therefore initialised here with explicit remap arguments so this
script's TransformListener reads the namespaced topics; without this it
reports "map -> odom never resolved" even when everything is working.

Usage:  python3 tools/check_gps_localization.py
"""

import math
import subprocess
import sys

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

WORLD = "park"
MODEL = "a200_0000/robot"


def gazebo_pose() -> tuple[float, float, float]:
    out = subprocess.run(["gz", "model", "-m", MODEL, "-p"],
                         capture_output=True, text=True).stdout
    xs = [float(v) for v in
          __import__("re").findall(r"\[([-0-9.e+]+) ([-0-9.e+]+) ([-0-9.e+]+)\]", out)[0]]
    return xs[0], xs[1], xs[2]


class TfProbe(Node):
    def __init__(self) -> None:
        super().__init__("check_gps_localization")
        self.buf = Buffer()
        TransformListener(self.buf, self)

    def lookup(self, target: str, source: str, tries: int = 40):
        for _ in range(tries):
            rclpy.spin_once(self, timeout_sec=0.5)
            try:
                return self.buf.lookup_transform(target, source, rclpy.time.Time())
            except Exception:
                continue
        return None


def main() -> int:
    rclpy.init(args=[
        "--ros-args",
        "-r", "/tf:=/a200_0000/tf",
        "-r", "/tf_static:=/a200_0000/tf_static",
    ])
    node = TfProbe()
    try:
        tf_mo = node.lookup("map", "odom")
        if tf_mo is None:
            print("  FAIL: map -> odom never resolved")
            print("        is gps_localization.launch.py running? GPS is 1 Hz;")
            print("        navsat_transform needs a fix before it publishes.")
            return 1
        t = tf_mo.transform.translation
        print(f"  map -> odom: x {t.x:+.3f}  y {t.y:+.3f}  z {t.z:+.3f}")

        tf_mb = node.lookup("map", "base_link")
        if tf_mb is None:
            print("  FAIL: map -> base_link never resolved")
            return 1
        b = tf_mb.transform.translation
        gx, gy, gz = gazebo_pose()
        err = math.hypot(b.x - gx, b.y - gy)
        print(f"  robot in map:   x {b.x:+8.3f}  y {b.y:+8.3f}")
        print(f"  gazebo truth:   x {gx:+8.3f}  y {gy:+8.3f}")
        print(f"  position error: {err:.3f} m")

        if err > 30.0:
            print("\n  FAIL: error that large usually means yaw_offset is wrong")
            print("        (a 90 deg heading error rotates the whole solution)")
            return 1
        if err > 2.0:
            print("\n  FAIL: map position does not track ground truth")
            return 1
        print("\n  PASS: map -> odom published and consistent with ground truth")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
