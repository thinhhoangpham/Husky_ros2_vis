#!/usr/bin/env python3
"""Report one sample of the SLAM occupancy grid: size, origin and cell counts.

Used between segments of DEMO.md's warehouse SLAM mapping run to decide when
the map has stopped growing (the documented stopping rule is: stop when known
cells grow by less than 5% over two consecutive segments).

The map is LATCHED: /a200_0000/map is published transient-local + reliable and
only on change, so a sensor-data (best-effort, volatile) subscription reads
nothing at all and looks like a dead topic. That QoS mismatch is the one thing
this script has to get right.

Known cells are those with value >= 0; -1 is unknown. The bounding box is over
known cells only, so it measures explored extent rather than grid allocation.

Usage:  python3 -m tools.check_map_growth [--timeout 10]
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

NS = "/a200_0000"
MAP_TOPIC = f"{NS}/map"
OCCUPIED_THRESHOLD = 65        # matches the saved map's occupied_thresh 0.65
TIMEOUT = 10.0

MAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


def summarise(data, width: int, height: int, resolution: float,
              origin_x: float, origin_y: float) -> dict:
    """Pure counting over a row-major OccupancyGrid payload."""
    occupied = free = unknown = 0
    min_i = min_j = 1 << 30
    max_i = max_j = -1
    for idx, v in enumerate(data):
        if v < 0:
            unknown += 1
            continue
        if v >= OCCUPIED_THRESHOLD:
            occupied += 1
        else:
            free += 1
        i, j = idx % width, idx // width
        min_i, max_i = min(min_i, i), max(max_i, i)
        min_j, max_j = min(min_j, j), max(max_j, j)

    known = occupied + free
    if known:
        bbox = (origin_x + min_i * resolution, origin_y + min_j * resolution,
                origin_x + (max_i + 1) * resolution,
                origin_y + (max_j + 1) * resolution)
    else:
        bbox = None
    return {
        "width": width, "height": height, "resolution": resolution,
        "origin": (origin_x, origin_y),
        "occupied": occupied, "free": free, "unknown": unknown, "known": known,
        "bbox": bbox,
    }


class MapProbe(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("check_map_growth")
        self.msg = None
        self.create_subscription(OccupancyGrid, topic, self._on_map, MAP_QOS)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.msg = msg

    def wait(self, timeout: float) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.msg is not None:
                return True
        return False


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--map-topic", default=MAP_TOPIC)
    p.add_argument("--timeout", type=float, default=TIMEOUT)
    args = p.parse_args(argv)

    rclpy.init()
    node = MapProbe(args.map_topic)
    try:
        if not node.wait(args.timeout):
            print(f"  FAIL: no message on {args.map_topic} within "
                  f"{args.timeout:.0f} s (is slam_toolbox running?)")
            return 1
        m = node.msg
        info = m.info
        s = summarise(m.data, info.width, info.height, info.resolution,
                      info.origin.position.x, info.origin.position.y)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(f"  grid:       {s['width']} x {s['height']} px @ "
          f"{s['resolution']:.3f} m/pix")
    print(f"  origin:     [{s['origin'][0]:.3f}, {s['origin'][1]:.3f}]")
    print(f"  occupied:   {s['occupied']}")
    print(f"  free:       {s['free']}")
    print(f"  unknown:    {s['unknown']}")
    print(f"  known:      {s['known']}")
    if s["bbox"] is None:
        print("  bbox:       none - no known cells")
        return 1
    x0, y0, x1, y1 = s["bbox"]
    print(f"  known bbox: x {x0:.2f}..{x1:.2f}  y {y0:.2f}..{y1:.2f}  "
          f"({x1 - x0:.2f} x {y1 - y0:.2f} m)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
