#!/usr/bin/env python3
"""Live RViz view of RSSI ranging and trilateration in park.

The visual sibling of tools/check_rssi_localization.py and
tools/check_rssi_route.py. Those tools print one report and exit; this one
runs continuously, pinging every tower at ~1 Hz, inverting each RSSI to a
range, solving the robot's 2D position and publishing the whole picture as a
visualization_msgs/MarkerArray so it can be watched while something else -
tools/check_rssi_route.py, nav2, teleop - drives the robot.

The measurement is inherited unchanged from the sibling tools: tower
discovery from worlds/park.sdf, the tower -> robot ping on /broker/msgs
answered on /husky/rx, the log-distance inversion

    d = 10 ** ((tx_power - l0 - rssi) / (10 * fading_exponent))

the dz reduction from slant to horizontal range, and the linearized
least-squares solve. Nothing here re-derives or re-tunes any of it.

FRAME - markers are published in `map`, and this node publishes NO TF at all.
config/gps_localization.yaml sets world_frame: map with publish_tf: true, so
ekf_node_map already owns the map -> odom edge; a second publisher of that
same edge makes TF flicker between sources with no clean error. This node
therefore DEPENDS on the GPS localization stack
(the Stage 1 GPS nodes in launch/park_stock.launch.py)
being up to supply `map`, and refuses to start without it rather than
publishing into a frame RViz cannot resolve.

Tower world coordinates go into `map` VERBATIM, with no transform arithmetic.
Under park's current datum (heading_deg 0) the Gazebo world frame IS the ENU
frame, navsat_transform's yaw_offset is 0.0 and the datum sits at the world
origin, so `map` and the world frame coincide - CLAUDE.md gotcha #32. This
would NOT hold under park's old heading_deg 90, which rotated the
world -> geodetic mapping 90 degrees about the datum; under that setting every
world x/y here would need rotating into map first.

Drawn each cycle:
  * a cylinder per discovered tower, coloured by whether it answered
  * a line from each IN-RANGE tower to the robot - a tower beyond max_range
    draws no line, so a dropout reads as a missing link
  * a text label per tower: name, measured RSSI, derived range
  * the simulator's ground-truth robot position and the trilaterated
    estimate, plus the position error in metres

With fewer than 3 in-range towers the 2D fix is underdetermined; the towers
and labels still draw, no solved-position marker is emitted, and the status
text says UNDERDETERMINED rather than showing a fabricated position.

Usage:  python3 tools/rssi_viz.py [--rate HZ] [--dry-run] [--ros-args ...]
        Bare invocation works: the node defaults its TF subscriptions to this
        stack's namespaced /a200_0000/tf and /a200_0000/tf_static. Any
        --ros-args the caller passes are forwarded to rclpy and, if they
        remap TF or set a namespace, replace that default entirely.
        --dry-run runs one cycle without requiring `map`, for checking the
        world parse, the pings and the marker construction against a bare sim.
"""

import argparse
import math
import os
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Point
from ros_gz_interfaces.msg import Dataframe
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_SDF = os.path.join(REPO_ROOT, "worlds", "park.sdf")

# The robot namespace, from `serial_number: a200-0000` (CLAUDE.md). Overridable
# without editing this file, so nothing here silently breaks if it changes.
NAMESPACE = os.environ.get("HUSKY_NAMESPACE", "a200_0000").strip("/")

ROBOT_MODEL = f"{NAMESPACE}/robot"
ROBOT_ADDRESS = "husky"
RX_TOPIC = "/husky/rx"      # the only bridged inbound topic; see check_rssi_localization.py
TX_TOPIC = "/broker/msgs"
MARKER_TOPIC = f"/{NAMESPACE}/rssi_viz"

MAP_FRAME = "map"
BASE_FRAME = "base_link"

PING_WAIT_S = 0.25      # per-ping wait; RFComms answers immediately, and a
                        # 1 Hz cycle has to fit every tower inside it
DISCOVERY_WAIT_S = 2.0  # one bounded wait so the pub/sub pair matches up
TF_WAIT_S = 3.0         # one bounded wait for map to appear, then decide

# This stack publishes TF on the NAMESPACED topics /<ns>/tf and /<ns>/tf_static -
# launch/park_stock.launch.py combines NAV2_REMAPS [("/tf","tf"),("/tf_static",
# "tf_static")] with PushRosNamespace - and global /tf does not exist at all.
# rclpy's tf2_ros.TransformListener hardcodes subscriptions to absolute /tf and
# /tf_static, so with no remap it binds to topics with zero publishers and every
# lookup fails. RViz is launched with exactly these remaps by hand; requiring
# the user to know that is the ergonomic bug, so the bare command supplies them
# itself.
#
# Chosen form: remap the two TOPICS only, as *global* ROS arguments, and leave
# the node un-namespaced. Namespacing the node instead (__ns:=) would work for
# TF but would also drag the node name and any relative name into the namespace
# for no benefit, and every topic this node actually uses (/broker/msgs,
# /husky/rx, the marker topic) is already absolute. Passing them as global args
# rather than per-node cli_args matters too: rcl resolves node-local remap rules
# BEFORE global ones, so cli_args defaults would OVERRIDE a user's explicit
# --ros-args instead of yielding to it.
#
# The defaults are injected only when the user supplied no TF remap and no
# namespace of their own, so an explicit --ros-args always wins (see
# tf_default_ros_args). This node still publishes NO TF - these are subscriber
# side remaps only; ekf_node_map keeps sole ownership of map -> odom.
DEFAULT_TF_REMAPS = [
    f"/tf:=/{NAMESPACE}/tf",
    f"/tf_static:=/{NAMESPACE}/tf_static",
]
REMAP_HINT = ("--ros-args -r __ns:=/{ns} -r /tf:=tf -r /tf_static:=tf_static"
              .format(ns=NAMESPACE))


def tf_default_ros_args(passthrough):
    """Default TF remaps, unless the caller already remapped TF themselves.

    `passthrough` is whatever argparse did not consume, i.e. the user's own
    `--ros-args ...`. If it mentions a TF remap or a namespace, we add nothing
    and their intent stands unmodified.
    """
    joined = " ".join(passthrough)
    if "/tf:=" in joined or "/tf_static:=" in joined or "__ns:=" in joined:
        return []
    args = ["--ros-args"]
    for rule in DEFAULT_TF_REMAPS:
        args += ["-r", rule]
    return args

# Tower cylinders in the world are 2.0 m tall, radius 0.1, centred at z=3.99.
TOWER_HEIGHT = 2.0
TOWER_RADIUS = 0.1

# Colours
C_IN_RANGE = (0.15, 0.90, 0.25, 1.0)    # green   - tower answered
C_OUT_RANGE = (0.55, 0.55, 0.60, 0.6)   # grey    - beyond max_range, no reply
C_LINK = (0.20, 0.80, 1.00, 0.9)        # cyan    - live RF link
C_TRUE = (1.00, 1.00, 1.00, 1.0)        # white   - ground truth
C_SOLVED = (1.00, 0.35, 0.05, 1.0)      # orange  - trilaterated estimate
C_ERR = (1.00, 0.85, 0.10, 1.0)         # yellow  - error line / status text
C_BAD = (1.00, 0.25, 0.25, 1.0)         # red     - underdetermined/degenerate


def die(msg: str) -> "NoReturn":
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
    would make every drawn range, and so the whole picture, meaningless.
    Tower names and count are never hardcoded: add a fifth tower to the world
    and it appears here with no edit.
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
    world_name = world.get("name") or die(f"{WORLD_SDF}: <world> has no name attribute")

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
            towers.append({"name": name, "address": address, "xyz": _model_xyz(model, name)})
    if not towers:
        die(f"{WORLD_SDF}: no CommsEndpoint models other than {ROBOT_ADDRESS!r} - "
            f"nothing to draw")

    return world_name, params, towers


def robot_pose():
    """Live robot XYZ from the simulator, plus the world gz reports.

    `gz model -m <name> -p` is world-agnostic, so unlike the world-scoped
    services of gotcha #26 there is nothing here to hardcode wrong.

    A one-shot read can come back empty on a transiently busy sim even while
    it is verified healthy - that aborted a live check_rssi_route.py run
    mid-route. Same family as gotchas #14/#38; retry a bounded few times
    before concluding anything is wrong. Returns (xyz, world) or (None, world)
    so a running visualisation survives a read that never came good.
    """
    attempts = 3
    out = ""
    m = None
    for attempt in range(1, attempts + 1):
        try:
            proc = subprocess.run(["gz", "model", "-m", ROBOT_MODEL, "-p"],
                                  capture_output=True, text=True, timeout=10)
        except FileNotFoundError:
            die("`gz` not found - source /opt/ros/jazzy/setup.bash first "
                "(Gazebo is vendored inside the ROS tree)")
        except subprocess.TimeoutExpired:
            return None, None

        out = proc.stdout
        # First bracketed triple after the "Pose" header is XYZ; the second is RPY.
        m = re.search(r"Pose[^\n]*\n\s*\[([^\]]+)\]", out)
        if m:
            break
        if attempt < attempts:
            time.sleep(0.2)

    live_world = None
    wm = re.search(r"Requesting state for world \[([^\]]+)\]", out)
    if wm:
        live_world = wm.group(1)
    if not m:
        return None, live_world
    try:
        xyz = tuple(float(v) for v in m.group(1).split())
    except ValueError:
        return None, live_world
    if len(xyz) != 3:
        return None, live_world
    return xyz, live_world


def invert_rssi(rssi, p):
    return 10.0 ** ((p["tx_power"] - p["l0"] - rssi) / (10.0 * p["fading_exponent"]))


def trilaterate(anchors_xy, ranges_2d):
    """Least-squares 2D position from horizontal ranges to known anchors.

    Subtracting the first circle equation from the rest cancels x^2 + y^2 and
    leaves a linear system; n anchors give n-1 rows, so 3 is the minimum.
    Returns (x, y, rank) - a rank-deficient (collinear) geometry is reported,
    not fatal, because it is a state the live view should be able to show.
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


class RssiViz(Node):
    """Pings every tower and publishes the resulting picture as markers.

    QoS on the comms topics: default RELIABLE, depth 10 - deliberately NOT
    qos_profile_sensor_data. The broker's endpoints are reliable, and a
    BEST_EFFORT publisher on /broker/msgs is rejected as incompatible ("No
    messages will be sent to it") and delivers ZERO packets, silently.
    """

    def __init__(self, params, towers, period_s):
        super().__init__("rssi_viz")
        self.p = params
        self.towers = towers
        self.period_s = period_s
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.rx = []
        self.create_subscription(Dataframe, RX_TOPIC, self.rx.append, qos)
        self.pub = self.create_publisher(Dataframe, TX_TOPIC, qos)
        self.markers = self.create_publisher(MarkerArray, MARKER_TOPIC, qos)

        # Listener only - this node publishes NO TF (see module docstring).
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Markers carry a lifetime slightly longer than one cycle so a stalled
        # publisher fades out instead of freezing a stale picture on screen;
        # everything still gets an explicit DELETE the moment it stops
        # applying (e.g. a tower going out of range must not leave its link
        # line behind).
        secs = 3.0 * period_s
        self.lifetime = DurationMsg(sec=int(secs), nanosec=int((secs % 1.0) * 1e9))

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def map_frame_available(self):
        """True once map exists in TF. One bounded wait, never a poll loop."""
        self.spin(TF_WAIT_S)
        return self.tf_buffer.can_transform(
            MAP_FRAME, BASE_FRAME, rclpy.time.Time(),
            timeout=Duration(seconds=0.0))

    def ping_from(self, tower_address, payload):
        """Send one frame tower -> robot; return the packet the robot got, or None."""
        before = len(self.rx)
        msg = Dataframe()
        msg.src_address = tower_address
        msg.dst_address = ROBOT_ADDRESS
        msg.data = list(payload.encode())
        self.pub.publish(msg)
        self.spin(PING_WAIT_S)
        got = [m for m in self.rx[before:] if m.src_address == tower_address]
        return got[-1] if got else None

    # --- measurement -----------------------------------------------------

    def measure(self, robot_xyz):
        """Ping every tower once. Returns (rows, used, solved, status).

        rows: one dict per tower - address, xyz, rssi/d_est/r2d or None.
        used: the in-range subset feeding the solve.
        """
        rows = []
        used = []
        for t in self.towers:
            tx, ty, tz = t["xyz"]
            pkt = self.ping_from(t["address"], f"rssi_viz ping from {t['address']}")
            row = {"address": t["address"], "xyz": t["xyz"],
                   "rssi": None, "d_est": None, "r2d": None, "note": ""}
            if pkt is None:
                # No reply means beyond <max_range>: RFComms gates delivery
                # geometrically, so this is the dropout the view should show.
                row["note"] = "out of range"
                rows.append(row)
                continue

            rssi = pkt.rssi  # top-level float64 on the ROS msg; NOT
                             # header.data{key:"rssi"}, which is the gz-side
                             # shape and reads as nothing here.
            d_est = invert_rssi(rssi, self.p)
            row["rssi"] = rssi
            row["d_est"] = d_est
            dz = tz - robot_xyz[2]
            if abs(d_est) <= abs(dz):
                row["note"] = "slant range below dz"
                rows.append(row)
                continue
            # Towers sit at z=3.99, the robot near z=3.12: ignoring dz would
            # bias every horizontal range long, and so bias the whole fix.
            row["r2d"] = math.sqrt(d_est * d_est - dz * dz)
            rows.append(row)
            used.append({"xy": (tx, ty), "r2d": row["r2d"]})

        if len(used) < 3:
            return rows, used, None, "UNDERDETERMINED"
        sx, sy, rank = trilaterate([u["xy"] for u in used], [u["r2d"] for u in used])
        if rank < 2:
            return rows, used, None, "DEGENERATE (in-range towers collinear)"
        return rows, used, (sx, sy), ""

    # --- markers ---------------------------------------------------------

    def _base(self, ns, mid, mtype, action=Marker.ADD):
        m = Marker()
        m.header.frame_id = MAP_FRAME  # world x/y == map x/y under heading_deg 0
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = action
        m.pose.orientation.w = 1.0
        m.lifetime = self.lifetime
        return m

    @staticmethod
    def _colour(m, rgba):
        m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])

    def build_markers(self, robot_xyz, rows, used, solved, status):
        """One MarkerArray for this cycle.

        Namespace + id are stable per tower index, so every marker updates in
        place instead of accumulating. Anything that does not apply this cycle
        is emitted as an explicit DELETE at its own ns/id, which is what stops
        an out-of-range tower's link line and label from lingering.
        """
        arr = MarkerArray()
        rx, ry, rz = robot_xyz

        for i, row in enumerate(rows):
            tx, ty, tz = row["xyz"]
            in_range = row["r2d"] is not None

            cyl = self._base("towers", i, Marker.CYLINDER)
            cyl.pose.position.x = tx
            cyl.pose.position.y = ty
            cyl.pose.position.z = tz
            cyl.scale.x = 2.0 * TOWER_RADIUS
            cyl.scale.y = 2.0 * TOWER_RADIUS
            cyl.scale.z = TOWER_HEIGHT
            self._colour(cyl, C_IN_RANGE if in_range else C_OUT_RANGE)
            arr.markers.append(cyl)

            label = self._base("labels", i, Marker.TEXT_VIEW_FACING)
            label.pose.position.x = tx
            label.pose.position.y = ty
            label.pose.position.z = tz + TOWER_HEIGHT / 2.0 + 0.6
            label.scale.z = 0.8
            if in_range:
                label.text = (f"{row['address']}\n{row['rssi']:.1f} dBm\n"
                              f"{row['r2d']:.2f} m")
                self._colour(label, C_IN_RANGE)
            else:
                label.text = f"{row['address']}\n{row['note'] or 'out of range'}"
                self._colour(label, C_OUT_RANGE)
            arr.markers.append(label)

            # One LINE_LIST per tower rather than one for all of them, so a
            # tower dropping out is a DELETE of its own marker and cannot
            # leave a stale segment inside a shared marker.
            if in_range:
                link = self._base("links", i, Marker.LINE_LIST)
                link.scale.x = 0.08
                self._colour(link, C_LINK)
                link.points = [Point(x=tx, y=ty, z=tz),
                               Point(x=rx, y=ry, z=rz)]
                arr.markers.append(link)
            else:
                arr.markers.append(self._base("links", i, Marker.LINE_LIST,
                                              Marker.DELETE))

        truth = self._base("position", 0, Marker.SPHERE)
        truth.pose.position.x = rx
        truth.pose.position.y = ry
        truth.pose.position.z = rz + 0.4
        truth.scale.x = truth.scale.y = truth.scale.z = 0.5
        self._colour(truth, C_TRUE)
        arr.markers.append(truth)

        truth_label = self._base("position", 1, Marker.TEXT_VIEW_FACING)
        truth_label.pose.position.x = rx
        truth_label.pose.position.y = ry
        truth_label.pose.position.z = rz + 1.1
        truth_label.scale.z = 0.5
        truth_label.text = "true"
        self._colour(truth_label, C_TRUE)
        arr.markers.append(truth_label)

        if solved is None:
            # No fabricated position: delete last cycle's estimate outright.
            for mid in (2, 3, 4):
                arr.markers.append(self._base("position", mid, Marker.SPHERE,
                                              Marker.DELETE))
            note = self._base("status", 0, Marker.TEXT_VIEW_FACING)
            note.pose.position.x = rx
            note.pose.position.y = ry
            note.pose.position.z = rz + 2.2
            note.scale.z = 0.7
            note.text = (f"{status}\n{len(used)}/{len(rows)} towers in range "
                         f"(3 needed)")
            self._colour(note, C_BAD)
            arr.markers.append(note)
            return arr

        sx, sy = solved
        err = math.hypot(sx - rx, sy - ry)

        # A cube against the truth sphere: different shape and colour, so the
        # two stay distinguishable even when they overlap at sub-metre error.
        est = self._base("position", 2, Marker.CUBE)
        est.pose.position.x = sx
        est.pose.position.y = sy
        est.pose.position.z = rz + 0.4
        est.scale.x = est.scale.y = est.scale.z = 0.45
        self._colour(est, C_SOLVED)
        arr.markers.append(est)

        # The error itself, drawn as the segment between the two, so a fix
        # that is merely close is still visibly separate from one that is not.
        errline = self._base("position", 3, Marker.LINE_LIST)
        errline.scale.x = 0.06
        self._colour(errline, C_ERR)
        errline.points = [Point(x=rx, y=ry, z=rz + 0.4),
                          Point(x=sx, y=sy, z=rz + 0.4)]
        arr.markers.append(errline)

        est_label = self._base("position", 4, Marker.TEXT_VIEW_FACING)
        est_label.pose.position.x = sx
        est_label.pose.position.y = sy
        est_label.pose.position.z = rz + 1.7
        est_label.scale.z = 0.6
        est_label.text = f"RSSI fix\nerror {err:.2f} m"
        self._colour(est_label, C_SOLVED)
        arr.markers.append(est_label)

        arr.markers.append(self._base("status", 0, Marker.TEXT_VIEW_FACING,
                                      Marker.DELETE))
        return arr

    def clear_all(self):
        """DELETEALL on shutdown, so nothing is left frozen in RViz."""
        arr = MarkerArray()
        m = self._base("", 0, Marker.SPHERE, Marker.DELETEALL)
        arr.markers.append(m)
        self.markers.publish(arr)
        self.spin(0.2)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--rate", type=float, default=1.0,
                    help="publish rate in Hz (default 1.0)")
    ap.add_argument("--dry-run", action="store_true",
                    help="one cycle without requiring the map frame; prints "
                         "what would be drawn. For checking against a bare sim.")
    args, passthrough = ap.parse_known_args()
    if args.rate <= 0.0:
        die(f"--rate must be positive, got {args.rate}")
    period = 1.0 / args.rate

    world_name, p, towers = parse_world()
    robot_xyz, live_world = robot_pose()
    if robot_xyz is None:
        die(f"could not read a pose for model {ROBOT_MODEL!r} - is a sim "
            f"running with the robot spawned?")
    if live_world is not None and live_world != world_name:
        die(f"the running sim is world {live_world!r} but radio parameters and "
            f"tower poses were parsed from {WORLD_SDF} (world {world_name!r}) - "
            f"refusing to visualise the wrong world (CLAUDE.md gotcha #26)")

    print("=== live RSSI trilateration markers ===")
    print(f"  world              : {world_name}"
          + ("" if live_world else "  (gz did not report a world name)"))
    print(f"  world file         : {WORLD_SDF}")
    print(f"  RFComms model      : tx_power {p['tx_power']} dBm, l0 {p['l0']} dB, "
          f"fading_exponent {p['fading_exponent']}, sigma {p['sigma']}, "
          f"max_range {p['max_range']} m")
    print(f"  towers discovered  : {len(towers)} "
          f"({', '.join(t['address'] for t in towers)})")
    for t in towers:
        print(f"      {t['address']:>15}  at [{t['xyz'][0]:>7.2f} {t['xyz'][1]:>7.2f} "
              f"{t['xyz'][2]:>6.2f}]")
    print(f"  marker topic       : {MARKER_TOPIC}  (frame {MAP_FRAME}, no TF published)")
    print(f"  rate               : {args.rate} Hz")
    print(f"  tf remaps          : "
          + (", ".join(DEFAULT_TF_REMAPS) if tf_default_ros_args(passthrough)
             else "from the caller's --ros-args"))
    print()

    ros_args = [sys.argv[0]] + tf_default_ros_args(passthrough) + passthrough
    rclpy.init(args=ros_args)
    node = RssiViz(p, towers, period)
    node.spin(DISCOVERY_WAIT_S)  # one bounded wait so pub/sub match, not a poll loop

    if args.dry_run:
        print("  --dry-run: one cycle, map frame not required.")
    elif not node.map_frame_available():
        node.destroy_node()
        rclpy.shutdown()
        die(f"no {MAP_FRAME!r} -> {BASE_FRAME!r} transform reached this node.\n"
            f"  The most likely cause is NOT a missing localization stack: it is "
            f"that TF is\n"
            f"  published under a namespace this node is not listening to. This "
            f"stack publishes\n"
            f"  on /{NAMESPACE}/tf and /{NAMESPACE}/tf_static, and global /tf "
            f"does not exist at all,\n"
            f"  while tf2_ros.TransformListener subscribes to absolute /tf. "
            f"Check which topics\n"
            f"  actually carry TF:\n"
            f"      ros2 topic info /{NAMESPACE}/tf\n"
            f"      ros2 topic info /tf\n"
            f"  and point this node at them explicitly if the namespace is not "
            f"{NAMESPACE!r}:\n"
            f"      python3 tools/rssi_viz.py {REMAP_HINT}\n"
            f"  (HUSKY_NAMESPACE=<ns> also changes the built-in default.)\n"
            f"  Confirm TF really resolves with:\n"
            f"      ros2 run tf2_ros tf2_echo {MAP_FRAME} {BASE_FRAME} "
            f"{REMAP_HINT}\n"
            f"  Only if that also fails is the localization stack itself the "
            f"problem\n"
            f"  (launch/park_stock.launch.py; verify with "
            f"tools/check_nav2_ready.py).\n"
            f"  This node publishes NO TF of its own either way - ekf_node_map "
            f"owns map -> odom.")

    try:
        while rclpy.ok():
            started = time.time()
            robot_xyz, _ = robot_pose()   # ~1 s subprocess round trip; this is a
                                          # plain loop, not a timer callback, so
                                          # nothing is starved by it
            if robot_xyz is None:
                print("  gz pose read failed after retries - skipping this cycle",
                      file=sys.stderr)
                node.spin(max(0.0, period - (time.time() - started)))
                continue

            rows, used, solved, status = node.measure(robot_xyz)
            node.markers.publish(
                node.build_markers(robot_xyz, rows, used, solved, status))

            if solved is None:
                print(f"  true ({robot_xyz[0]:8.3f}, {robot_xyz[1]:8.3f})  "
                      f"towers {len(used)}/{len(towers)}  {status}")
            else:
                err = math.hypot(solved[0] - robot_xyz[0], solved[1] - robot_xyz[1])
                print(f"  true ({robot_xyz[0]:8.3f}, {robot_xyz[1]:8.3f})  "
                      f"solved ({solved[0]:8.3f}, {solved[1]:8.3f})  "
                      f"towers {len(used)}/{len(towers)}  error {err:.3f} m")

            if args.dry_run:
                for row in rows:
                    if row["r2d"] is None:
                        print(f"  {row['address']:>15}  {row['note'] or 'out of range'}")
                    else:
                        print(f"  {row['address']:>15}  RSSI {row['rssi']:>8.3f} dBm  "
                              f"d_est {row['d_est']:>7.3f} m  r2d {row['r2d']:>7.3f} m")
                arr = node.build_markers(robot_xyz, rows, used, solved, status)
                print(f"  markers built      : {len(arr.markers)} in frame "
                      f"{arr.markers[0].header.frame_id!r}")
                break

            remaining = period - (time.time() - started)
            if remaining > 0.0:
                node.spin(remaining)
    except KeyboardInterrupt:
        print()
    finally:
        node.clear_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
