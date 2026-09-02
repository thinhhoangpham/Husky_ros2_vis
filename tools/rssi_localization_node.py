#!/usr/bin/env python3
"""RSSI trilateration as a nav2 global-localization source for park.

Publishes a geometry_msgs/PoseWithCovarianceStamped in the `map` frame,
derived only from radio ranges to the fixed base stations. It is the
measurement half of launch/rssi_localization.launch.py; ekf_node_map fuses
this pose and is the ONLY publisher of map -> odom.

THIS NODE PUBLISHES NO TF. That is deliberate and load-bearing: two
publishers of map -> odom make the edge flicker between sources with no
clean error (same failure family as CLAUDE.md gotcha #7's double imu_enu
bridge). The pose is merely STAMPED `map`; nothing here looks `map` up in
TF, so the node runs and publishes correctly before the EKF has produced
the frame at all.

The measurement pipeline is inherited unchanged from the verified tools
tools/check_rssi_ranging.py, tools/check_rssi_localization.py and
tools/rssi_viz.py - tower discovery from the world file, the tower -> robot
ping on /broker/msgs answered on /husky/rx, the log-distance inversion

    d = 10 ** ((tx_power - l0 - rssi) / (10 * fading_exponent))

the dz reduction from slant to horizontal range, and the linearized
least-squares solve. Nothing here re-derives or re-tunes any of it.

FRAME - tower world coordinates are used as `map` coordinates VERBATIM. Under
park's current datum (heading_deg 0) the Gazebo world frame IS the ENU frame,
navsat_transform's yaw_offset is 0.0 and the datum sits at the world origin,
so `map` and the world frame coincide (CLAUDE.md gotcha #32). This would NOT
hold under park's old heading_deg 90.

NO GROUND TRUTH IN THE ESTIMATE. The solver never reads the simulator's pose.
That includes the robot's z: see ROBOT_Z_MAP below. The `ground_truth_compare`
parameter (default false) may read `gz model` for LOGGING ONLY; it can never
influence the published pose.

ORIENTATION IS NOT OBSERVABLE from ranges. The published orientation is
identity with a huge covariance, and config/rssi_localization.yaml fuses ONLY
x and y from this input. Fusing the identity quaternion as a yaw measurement
would drag the filter's heading to 0 rad and corrupt the whole estimate.

Parameters (all ROS parameters, tunable without editing this file):
  rate_hz              publish rate, default 1.0 - see PING_WAIT_S
  position_covariance  variance in m^2 on x and y, default 0.25
  ground_truth_compare log the gz pose alongside the solve, default false.
                       Measured on park: the `gz model` round trip adds ~4.8 s
                       per cycle, so the effective rate drops to ~0.17 Hz while
                       it is on. It is a diagnostic, not something to leave
                       enabled while nav2 is consuming the pose.
"""

import math
import os
import sys
import time
import xml.etree.ElementTree as ET
import re
import subprocess

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from ros_gz_interfaces.msg import Dataframe

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_SDF = os.path.join(REPO_ROOT, "worlds", "park.sdf")

ROBOT_MODEL = "a200_0000/robot"     # ground-truth comparison only, never the estimate
ROBOT_ADDRESS = "husky"
RX_TOPIC = "/husky/rx"      # the only bridged inbound topic; the other towers'
                            # rx topics exist gz-side only and cannot be
                            # subscribed to from ROS
TX_TOPIC = "/broker/msgs"
POSE_TOPIC = "rssi/pose"    # relative: resolves under the node's namespace
MAP_FRAME = "map"

# SURVEYED CONSTANT, not a simulator reading. Reading the robot's true z to
# reduce slant ranges would be consuming ground truth inside a localization
# backend. park's terrain is flat at z ~= 2.99 (CLAUDE.md: "ground z ~2.99
# (flat)"), and the A200's base_link sits ~0.13 m above the surface - the
# robot settled at z = 3.120 in the verified spawn test (gotcha #23). So the
# antenna height above the datum is taken as a fixed 3.12 m. The towers are at
# z = 3.99, giving dz = 0.87 m; an error of a few centimetres in this constant
# changes the horizontal range by well under a centimetre at the ranges
# involved (r2d = sqrt(d^2 - dz^2), and d >> dz), so a flat-terrain constant is
# sufficient here. It would NOT be on lake, whose relief spans 2.43 m.
ROBOT_Z_MAP = 3.12

PING_WAIT_S = 0.25      # per-ping wait; RFComms answers immediately. Four
                        # towers x 0.25 s = 1.0 s per cycle, which is why the
                        # default rate is 1.0 Hz - the measurement itself, not
                        # a chosen filter bandwidth, sets the ceiling.
DISCOVERY_WAIT_S = 2.0  # one bounded wait so the pub/sub pair matches up


def die(msg):
    print(f"FAIL: {msg}", file=sys.stderr)
    sys.exit(1)


def _req_text(parent, path, where):
    node = parent.find(path)
    if node is None or node.text is None or not node.text.strip():
        die(f"{WORLD_SDF}: missing <{path}> in {where}")
    return node.text.strip()


def _req_float(parent, path, where):
    text = _req_text(parent, path, where)
    try:
        return float(text)
    except ValueError:
        die(f"{WORLD_SDF}: <{path}> in {where} is not a number: {text!r}")


def _model_xyz(model, name):
    parts = _req_text(model, "pose", f"model {name}").split()
    if len(parts) < 3:
        die(f"{WORLD_SDF}: {name} <pose> has fewer than 3 components")
    try:
        return tuple(float(v) for v in parts[:3])
    except ValueError:
        die(f"{WORLD_SDF}: {name} <pose> is not numeric: {parts[:3]}")


def parse_world():
    """RFComms parameters plus every non-robot comms endpoint in the world.

    Fails loudly on anything missing - a silently defaulted tx_power or l0
    would make every derived range, and so the whole fix, meaningless. No
    tower name or count is hardcoded: add a fifth base station to the world
    and this node picks it up with no edit.
    """
    if not os.path.exists(WORLD_SDF):
        die(f"world file not found: {WORLD_SDF}")
    try:
        root = ET.parse(WORLD_SDF).getroot()
    except ET.ParseError as exc:
        die(f"cannot parse {WORLD_SDF}: {exc}")

    world = root.find("world")
    if world is None:
        die(f"{WORLD_SDF}: no <world> element")
    world_name = world.get("name")
    if not world_name:
        die(f"{WORLD_SDF}: <world> has no name attribute")

    rf = None
    for plugin in world.findall("plugin"):
        if "RFComms" in (plugin.get("name") or ""):
            rf = plugin
            break
    if rf is None:
        die(f"{WORLD_SDF}: no RFComms plugin in world <{world_name}>")

    rng = rf.find("range_config")
    if rng is None:
        die(f"{WORLD_SDF}: RFComms plugin has no <range_config>")
    radio = rf.find("radio_config")
    if radio is None:
        die(f"{WORLD_SDF}: RFComms plugin has no <radio_config>")

    params = {
        "max_range": _req_float(rng, "max_range", "range_config"),
        "fading_exponent": _req_float(rng, "fading_exponent", "range_config"),
        "l0": _req_float(rng, "l0", "range_config"),
        "sigma": _req_float(rng, "sigma", "range_config"),
        "tx_power": _req_float(radio, "tx_power", "radio_config"),
    }
    if params["fading_exponent"] == 0.0:
        die("fading_exponent is 0 - path loss carries no distance information")

    towers = []
    for model in world.findall("model"):
        name = model.get("name") or "<unnamed>"
        for plugin in model.findall("plugin"):
            if "CommsEndpoint" not in (plugin.get("name") or ""):
                continue
            address = _req_text(plugin, "address", f"CommsEndpoint of model {name}")
            if address == ROBOT_ADDRESS:
                continue
            towers.append({"name": name, "address": address,
                           "xyz": _model_xyz(model, name)})
    if not towers:
        die(f"{WORLD_SDF}: no CommsEndpoint models other than {ROBOT_ADDRESS!r} - "
            f"nothing to trilaterate from")
    return world_name, params, towers


def invert_rssi(rssi, p):
    return 10.0 ** ((p["tx_power"] - p["l0"] - rssi) / (10.0 * p["fading_exponent"]))


def trilaterate(anchors_xy, ranges_2d):
    """Least-squares 2D position from horizontal ranges to known anchors.

    Subtracting the first circle equation from the rest cancels x^2 + y^2 and
    leaves a linear system; n anchors give n-1 rows, so 3 is the minimum.
    Returns (x, y, rank); a rank-deficient (collinear) geometry is reported
    rather than fatal, so the caller can skip the cycle instead of dying.
    """
    a = np.asarray(anchors_xy, dtype=float)
    r = np.asarray(ranges_2d, dtype=float)
    x0, y0 = a[0]
    r0 = r[0]
    A = 2.0 * (a[1:] - a[0])
    b = ((a[1:, 0] ** 2 + a[1:, 1] ** 2 - r[1:] ** 2)
         - (x0 ** 2 + y0 ** 2 - r0 ** 2))
    sol, _, rank, _ = np.linalg.lstsq(A, b, rcond=None)
    return float(sol[0]), float(sol[1]), int(rank)


def ground_truth_xy():
    """The simulator's robot pose - LOGGING ONLY, never the estimate.

    `gz model -m <name> -p` is world-agnostic, so unlike the world-scoped
    services of CLAUDE.md gotcha #26 there is no world name to hardcode wrong.
    Returns None on any failure; a comparison that cannot be made is simply
    not logged.
    """
    try:
        proc = subprocess.run(["gz", "model", "-m", ROBOT_MODEL, "-p"],
                              capture_output=True, text=True, timeout=10)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    m = re.search(r"Pose[^\n]*\n\s*\[([^\]]+)\]", proc.stdout)
    if not m:
        return None
    try:
        vals = [float(v) for v in m.group(1).split()]
    except ValueError:
        return None
    return (vals[0], vals[1]) if len(vals) >= 2 else None


class RssiLocalization(Node):
    """Pings every tower once per cycle and publishes the solved 2D position.

    QoS on the comms topics: default RELIABLE, depth 10 - deliberately NOT
    qos_profile_sensor_data. CLAUDE.md recommends sensor-data QoS for sensor
    streams, but the comms broker's endpoints are RELIABLE, and a BEST_EFFORT
    publisher on /broker/msgs is rejected as incompatible ("No messages will
    be sent to it") and delivers ZERO packets, silently. Verified live.
    """

    def __init__(self, params, towers):
        super().__init__("rssi_localization")
        self.p = params
        self.towers = towers

        self.declare_parameter("rate_hz", 1.0)
        # Default 0.25 m^2 == 0.5 m one-sigma on each of x and y. park's
        # <sigma> is 0.0 so the ranges themselves are exact and the solve
        # matches truth to well under 0.1 m; the default is deliberately
        # LOOSER than that measured accuracy so the filter is not forced to
        # track every solve rigidly, and so the number stays sane if <sigma>
        # is later raised to model fading. Tune with:
        #   -p position_covariance:=<m^2>
        self.declare_parameter("position_covariance", 0.25)
        self.declare_parameter("ground_truth_compare", False)

        rate = float(self.get_parameter("rate_hz").value)
        if rate <= 0.0:
            die(f"rate_hz must be positive, got {rate}")
        self.period_s = 1.0 / rate
        self.pos_cov = float(self.get_parameter("position_covariance").value)
        if self.pos_cov <= 0.0:
            die(f"position_covariance must be positive, got {self.pos_cov}")
        self.compare = bool(self.get_parameter("ground_truth_compare").value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.rx = []
        self.create_subscription(Dataframe, RX_TOPIC, self.rx.append, qos)
        self.tx = self.create_publisher(Dataframe, TX_TOPIC, qos)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                              POSE_TOPIC, qos)

    def spin(self, seconds):
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def ping_from(self, tower_address):
        """Send one frame tower -> robot; return the packet the robot got, or None."""
        before = len(self.rx)
        msg = Dataframe()
        msg.src_address = tower_address
        msg.dst_address = ROBOT_ADDRESS
        msg.data = list(f"rssi_localization ping from {tower_address}".encode())
        self.tx.publish(msg)
        self.spin(PING_WAIT_S)
        got = [m for m in self.rx[before:] if m.src_address == tower_address]
        return got[-1] if got else None

    def measure(self):
        """One full cycle. Returns (x, y, n_used) or (None, None, n_used)."""
        used = []
        for t in self.towers:
            tx, ty, tz = t["xyz"]
            pkt = self.ping_from(t["address"])
            if pkt is None:
                # No reply means beyond <max_range>: RFComms gates delivery
                # geometrically. Expected, not an error.
                continue
            rssi = pkt.rssi  # top-level float64 on the ROS msg; NOT
                             # header.data{key: "rssi"}, which is the gz-side
                             # shape and reads as nothing here. Verified live.
            d_est = invert_rssi(rssi, self.p)
            dz = tz - ROBOT_Z_MAP
            if abs(d_est) <= abs(dz):
                self.get_logger().warn(
                    f"{t['address']}: slant range {d_est:.3f} m is not greater "
                    f"than dz {abs(dz):.3f} m - dropping this tower",
                    throttle_duration_sec=10.0)
                continue
            r_2d = math.sqrt(d_est * d_est - dz * dz)
            used.append({"xy": (tx, ty), "r2d": r_2d})

        if len(used) < 3:
            # Publish NOTHING rather than a degraded or fabricated pose: a 2D
            # fix from fewer than 3 ranges is underdetermined, and feeding the
            # EKF an invented position is worse than feeding it nothing (it
            # dead-reckons on wheel odometry across the gap).
            self.get_logger().warn(
                f"only {len(used)}/{len(self.towers)} towers in range - 2D fix "
                f"underdetermined, publishing no pose this cycle",
                throttle_duration_sec=10.0)
            return None, None, len(used)

        x, y, rank = trilaterate([u["xy"] for u in used], [u["r2d"] for u in used])
        if rank < 2:
            self.get_logger().warn(
                f"the {len(used)} in-range towers are collinear (rank {rank}) - "
                f"the 2D fix is degenerate, publishing no pose this cycle",
                throttle_duration_sec=10.0)
            return None, None, len(used)
        return x, y, len(used)

    def publish_pose(self, x, y):
        msg = PoseWithCovarianceStamped()
        # Stamped `map`, but no TF lookup and no TF broadcast: ekf_node_map is
        # the single owner of map -> odom.
        msg.header.frame_id = MAP_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        # z is the surveyed constant, not a measurement; ekf_node_map runs
        # two_d_mode and fuses only x and y from this input, so it is ignored.
        msg.pose.pose.position.z = ROBOT_Z_MAP
        msg.pose.pose.orientation.w = 1.0   # identity - see below

        cov = [0.0] * 36
        cov[0] = self.pos_cov    # x
        cov[7] = self.pos_cov    # y
        # Everything ranges cannot observe gets a huge variance. The identity
        # orientation above is NOT a heading measurement - trilateration from
        # ranges says nothing about yaw. config/rssi_localization.yaml fuses
        # only x and y (pose0_config), and this covariance is the second line
        # of defence: if anyone ever flips roll/pitch/yaw on in that config,
        # the filter still all but ignores them instead of being dragged to
        # yaw = 0.
        for i in (14, 21, 28, 35):   # z, roll, pitch, yaw
            cov[i] = 1e6
        msg.pose.covariance = cov
        self.pose_pub.publish(msg)


def main():
    world_name, p, towers = parse_world()

    rclpy.init(args=sys.argv)
    node = RssiLocalization(p, towers)
    log = node.get_logger()
    log.info(f"world {world_name} ({WORLD_SDF})")
    log.info(f"RFComms: tx_power {p['tx_power']} dBm, l0 {p['l0']} dB, "
             f"fading_exponent {p['fading_exponent']}, sigma {p['sigma']}, "
             f"max_range {p['max_range']} m")
    log.info(f"towers discovered: {len(towers)} "
             f"({', '.join(t['address'] for t in towers)})")
    log.info(f"publishing PoseWithCovarianceStamped on {POSE_TOPIC} in frame "
             f"{MAP_FRAME} at {1.0 / node.period_s:.2f} Hz, "
             f"position_covariance {node.pos_cov} m^2, no TF")

    node.spin(DISCOVERY_WAIT_S)  # one bounded wait so pub/sub match, not a poll loop
    try:
        while rclpy.ok():
            started = time.time()
            x, y, n_used = node.measure()
            if x is not None:
                node.publish_pose(x, y)
                if node.compare:
                    truth = ground_truth_xy()
                    if truth is None:
                        log.info(f"solved ({x:.3f}, {y:.3f}) from {n_used}/"
                                 f"{len(towers)} towers  [truth unavailable]")
                    else:
                        err = math.hypot(x - truth[0], y - truth[1])
                        log.info(f"solved ({x:.3f}, {y:.3f}) from {n_used}/"
                                 f"{len(towers)} towers  truth "
                                 f"({truth[0]:.3f}, {truth[1]:.3f})  "
                                 f"error {err:.3f} m")
                else:
                    log.info(f"solved ({x:.3f}, {y:.3f}) from {n_used}/"
                             f"{len(towers)} towers",
                             throttle_duration_sec=5.0)
            remaining = node.period_s - (time.time() - started)
            if remaining > 0.0:
                node.spin(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
