#!/usr/bin/env python3
"""Send the Husky a navigation goal given as latitude/longitude.

Converts via robot_localization's fromLL service, which applies park's datum
and heading, then hands the resulting map-frame pose to nav_goal's sender.

Usage:  python3 tools/nav_goal_ll.py LAT LON [YAW_DEG]
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from geographic_msgs.msg import GeoPoint
from rclpy.node import Node
from robot_localization.srv import FromLL

from tools.nav_goal import GoalSender, in_terrain, make_pose

NS = "a200_0000"


class LLConverter(Node):
    def __init__(self) -> None:
        super().__init__("nav_goal_ll")
        self.cli = self.create_client(FromLL, f"/{NS}/fromLL")

    def to_map(self, lat: float, lon: float) -> tuple[float, float] | None:
        if not self.cli.wait_for_service(timeout_sec=10.0):
            print("  FAIL: /a200_0000/fromLL not available")
            print("        is gps_localization.launch.py running?")
            return None
        req = FromLL.Request()
        req.ll_point = GeoPoint(latitude=float(lat), longitude=float(lon), altitude=0.0)
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        r = fut.result()
        if r is None:
            print("  FAIL: fromLL call returned nothing")
            return None
        return r.map_point.x, r.map_point.y


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    lat, lon = float(sys.argv[1]), float(sys.argv[2])
    yaw = math.radians(float(sys.argv[3])) if len(sys.argv) > 3 else 0.0

    rclpy.init()
    conv = LLConverter()
    try:
        pt = conv.to_map(lat, lon)
        if pt is None:
            return 1
        x, y = pt
        print(f"  {lat:.9f}, {lon:.9f}  ->  map x {x:.2f}  y {y:.2f}")
        if not in_terrain(x, y):
            print("  REFUSED: that lat/lon is outside park's terrain.")
            print("           Check the datum: routes are valid only at 49.9 N, 8.9 E.")
            return 2
    finally:
        conv.destroy_node()

    node = GoalSender()
    try:
        return node.send(make_pose(x, y, yaw))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
