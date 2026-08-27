#!/usr/bin/env python3
"""Prove the generated prior map agrees with what the lidar actually sees.

A wrong mesh scale or pose produces a map that looks plausible but puts trees
in the wrong place, so the robot swerves around empty air and clips real
trunks. This measures the disagreement instead of eyeballing RViz.

Method: take one 2D lidar scan, convert each return to a map cell via TF, and
report what fraction land on cells the prior map calls occupied.

Usage:  python3 tools/check_map_alignment.py
"""

import math
import sys

import numpy as np
import rclpy
import yaml
from PIL import Image
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

MAP_YAML = "/home/thinhpham/Documents/Husky_viz/maps/park_map.yaml"
SCAN = "/a200_0000/sensors/lidar2d_0/scan"
TOLERANCE_CELLS = 6      # 0.30 m at 0.05 m/cell
MIN_HIT_FRACTION = 0.60


class ScanProbe(Node):
    def __init__(self) -> None:
        super().__init__("check_map_alignment")
        self.scan: LaserScan | None = None
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.create_subscription(LaserScan, SCAN, self._cb, qos_profile_sensor_data)

    def _cb(self, msg: LaserScan) -> None:
        self.scan = msg

    def grab(self):
        for _ in range(80):
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.scan is None:
                continue
            try:
                tf = self.buf.lookup_transform("map", self.scan.header.frame_id,
                                               rclpy.time.Time())
                return self.scan, tf
            except Exception:
                continue
        return None, None


def main() -> int:
    meta = yaml.safe_load(open(MAP_YAML))
    img = np.asarray(Image.open(MAP_YAML.replace(".yaml", ".pgm")))
    res = meta["resolution"]
    ox, oy = meta["origin"][0], meta["origin"][1]
    h = img.shape[0]
    occ = img == 0

    # Ruling D3: this stack remaps /tf and /tf_static to /a200_0000/tf(_static);
    # a default TransformListener only sees global /tf, which this stack never
    # publishes, so remap at node construction (same technique as
    # check_nav2_ready.py).
    rclpy.init(args=[
        "--ros-args",
        "-r", "/tf:=/a200_0000/tf",
        "-r", "/tf_static:=/a200_0000/tf_static",
    ])
    node = ScanProbe()
    try:
        scan, tf = node.grab()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if scan is None:
        print("  FAIL: no scan or no map->laser transform")
        return 1

    t = tf.transform.translation
    q = tf.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))

    hits = total = 0
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r <= scan.range_min or r >= scan.range_max * 0.98:
            continue
        a = scan.angle_min + i * scan.angle_increment + yaw
        x = t.x + r * math.cos(a)
        y = t.y + r * math.sin(a)
        c = int((x - ox) / res)
        rw = h - 1 - int((y - oy) / res)
        if not (0 <= c < img.shape[1] and 0 <= rw < h):
            continue
        total += 1
        lo_r, hi_r = max(0, rw - TOLERANCE_CELLS), min(h, rw + TOLERANCE_CELLS + 1)
        lo_c, hi_c = max(0, c - TOLERANCE_CELLS), min(img.shape[1], c + TOLERANCE_CELLS + 1)
        if occ[lo_r:hi_r, lo_c:hi_c].any():
            hits += 1

    if total == 0:
        print("  FAIL: no usable lidar returns (robot in the open?)")
        print("        drive nearer a tree line and retry")
        return 1

    frac = hits / total
    print(f"  lidar returns landing on mapped obstacles: {hits}/{total} = {frac:.1%}")
    print(f"  tolerance: {TOLERANCE_CELLS} cells ({TOLERANCE_CELLS * res:.2f} m)")
    if frac < MIN_HIT_FRACTION:
        print(f"\n  FAIL: below {MIN_HIT_FRACTION:.0%}; the prior map is misaligned")
        print("        suspect mesh scale, up-axis, or pose composition in sdf_geometry")
        return 1
    print("\n  PASS: prior map agrees with the live lidar")
    return 0


if __name__ == "__main__":
    sys.exit(main())
