#!/usr/bin/env python3
"""Drive a sequence of waypoints from a route file via nav2's waypoint follower.

Usage:  python3 tools/nav_route.py routes/park_route_1.yaml
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
import yaml
from geographic_msgs.msg import GeoPoint
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_localization.srv import FromLL

from tools.nav_goal import in_terrain, make_pose

NS = "a200_0000"


class RouteRunner(Node):
    def __init__(self) -> None:
        super().__init__("nav_route")
        self.ll = self.create_client(FromLL, f"/{NS}/fromLL")
        self.client = ActionClient(self, FollowWaypoints, f"/{NS}/follow_waypoints")

    def resolve(self, route: dict) -> list | None:
        poses = []
        if route.get("frame") == "latlon":
            if not self.ll.wait_for_service(timeout_sec=10.0):
                print("  FAIL: /a200_0000/fromLL not available")
                return None
            for wp in route["waypoints"]:
                req = FromLL.Request()
                req.ll_point = GeoPoint(latitude=float(wp["lat"]),
                                        longitude=float(wp["lon"]), altitude=0.0)
                fut = self.ll.call_async(req)
                rclpy.spin_until_future_complete(self, fut)
                r = fut.result()
                if r is None:
                    print("  FAIL: fromLL call returned nothing")
                    return None
                poses.append((r.map_point.x, r.map_point.y))
        else:
            poses = [(float(wp["x"]), float(wp["y"])) for wp in route["waypoints"]]

        for i, (x, y) in enumerate(poses, 1):
            inside = in_terrain(x, y)
            print(f"  wp {i}: map x {x:8.2f}  y {y:7.2f}  {'ok' if inside else 'OUTSIDE TERRAIN'}")
            if not inside:
                print("  REFUSED: a waypoint lies outside park's terrain.")
                return None
        return [make_pose(x, y, 0.0) for x, y in poses]

    def run(self, poses: list) -> int:
        if not self.client.wait_for_server(timeout_sec=10.0):
            print("  FAIL: follow_waypoints action server not available")
            return 1
        goal = FollowWaypoints.Goal()
        stamp = self.get_clock().now().to_msg()
        for p in poses:
            p.header.stamp = stamp
        goal.poses = poses
        fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if handle is None or not handle.accepted:
            print("  FAIL: route rejected by the action server")
            return 1
        print(f"  route accepted: {len(poses)} waypoints")
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        result = res_fut.result()
        missed = list(result.result.missed_waypoints)
        if result.status == 4 and not missed:
            print("  PASS: all waypoints reached")
            return 0
        print(f"  FAIL: status {result.status}, missed waypoints {missed}")
        return 1


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    route = yaml.safe_load(open(sys.argv[1]))
    rclpy.init()
    node = RouteRunner()
    try:
        poses = node.resolve(route)
        if poses is None:
            return 2
        return node.run(poses)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
