#!/usr/bin/env python3
"""Measure AMCL's pose estimate against Gazebo ground truth.

Reports position error in metres and yaw error in degrees. Truth comes from the
world's dynamic pose feed (the same TruthStream that
tools/check_localization_drive.py uses - /world/<w>/pose/info carries only the
static layout, /world/<w>/dynamic_pose/info is the one that streams the robot);
the estimate comes from /a200_0000/amcl_pose.

Frame assumption, and the reason it holds for DEMO.md's warehouse run: the
error is computed by comparing the map-frame estimate directly against the
world-frame truth, which is only meaningful when the map frame coincides with
the Gazebo world frame. That is the case for a map built by slam_toolbox from
this spawn and then seeded into AMCL from truth via /a200_0000/initialpose. If
your map frame is offset from the world, pass --offset-x/--offset-y/--offset-yaw
(map origin expressed in the world frame) rather than reading the numbers as
localization error.

QoS: amcl_pose is subscribed twice, once VOLATILE and once TRANSIENT_LOCAL. A
transient-local subscriber is incompatible with a volatile publisher and vice
versa, and which one nav2's AMCL uses is a version detail; taking whichever
delivers avoids a silent no-data read (the failure family of gotcha #38).

AMCL only publishes on a filter update, so at rest the newest message can be
seconds old - that is expected, not a fault.

Usage:  python3 -m tools.check_amcl_error [--world warehouse] [--timeout 15]
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from tools.check_localization_drive import TruthStream, yaw_from_quat

NS = "/a200_0000"
AMCL_TOPIC = f"{NS}/amcl_pose"
WORLD = "warehouse"
TIMEOUT = 15.0

VOLATILE_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.VOLATILE,
                          history=HistoryPolicy.KEEP_LAST)
LATCHED_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)


def pose_error(estimate, truth) -> tuple[float, float]:
    """(position error in m, yaw error in deg) between two (x, y, yaw) poses."""
    ex, ey, eyaw = estimate
    tx, ty, tyaw = truth
    dyaw = math.atan2(math.sin(eyaw - tyaw), math.cos(eyaw - tyaw))
    return math.hypot(ex - tx, ey - ty), abs(math.degrees(dyaw))


def apply_offset(pose, offset) -> tuple[float, float, float]:
    """Express a map-frame pose in the world frame given the map origin."""
    x, y, yaw = pose
    ox, oy, oyaw = offset
    c, s = math.cos(oyaw), math.sin(oyaw)
    return (ox + c * x - s * y, oy + s * x + c * y, yaw + oyaw)


class AmclProbe(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("check_amcl_error")
        self.pose = None
        for qos in (VOLATILE_QOS, LATCHED_QOS):
            self.create_subscription(PoseWithCovarianceStamped, topic,
                                     self._on_pose, qos)

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    def wait(self, truth: TruthStream, timeout: float):
        """Bounded wait on both real signals, not a sleep."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            t = truth.get()
            if self.pose is not None and t is not None:
                return self.pose, t
        return None, truth.get()


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--world", default=WORLD, help="Gazebo world name (gotcha #26)")
    p.add_argument("--amcl-topic", default=AMCL_TOPIC)
    p.add_argument("--timeout", type=float, default=TIMEOUT)
    p.add_argument("--offset-x", type=float, default=0.0)
    p.add_argument("--offset-y", type=float, default=0.0)
    p.add_argument("--offset-yaw", type=float, default=0.0)
    p.add_argument("--max-position-error", type=float, default=0.50,
                   help="metres; above this the check fails")
    p.add_argument("--max-yaw-error", type=float, default=15.0,
                   help="degrees; above this the check fails")
    args = p.parse_args(argv)

    rclpy.init()
    truth = TruthStream(args.world)
    node = AmclProbe(args.amcl_topic)
    try:
        est, tru = node.wait(truth, args.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        truth.stop()

    if tru is None:
        print(f"  FAIL: no truth pose on /world/{args.world}/dynamic_pose/info "
              f"within {args.timeout:.0f} s (is the world name right?)")
        return 1
    if est is None:
        print(f"  FAIL: no message on {args.amcl_topic} within "
              f"{args.timeout:.0f} s (is AMCL running and seeded?)")
        return 1

    est_world = apply_offset(est, (args.offset_x, args.offset_y, args.offset_yaw))
    dpos, dyaw = pose_error(est_world, tru)
    print(f"  AMCL  (map):   ({est[0]:.3f}, {est[1]:.3f}) yaw {est[2]:.4f}")
    print(f"  truth (world): ({tru[0]:.3f}, {tru[1]:.3f}) yaw {tru[2]:.4f}")
    print(f"  position error: {dpos:.3f} m")
    print(f"  yaw error:      {dyaw:.1f} deg")

    if dpos > args.max_position_error or dyaw > args.max_yaw_error:
        print(f"  FAIL: outside {args.max_position_error:.2f} m / "
              f"{args.max_yaw_error:.1f} deg")
        return 1
    print("  PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
