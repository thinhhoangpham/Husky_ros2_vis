#!/usr/bin/env python3
"""Single entry point for the Husky simulation.

    python3 scripts/sim.py start <world> [--config NAME] [--no-nav]
                                         [--x X --y Y --z Z --yaw YAW] [--clean-on-fail]
    python3 scripts/sim.py stop
    python3 scripts/sim.py status

Design: docs/superpowers/specs/2026-08-28-single-sim-entrypoint-design.md
Every gate is a pure function over captured text; `Shell` owns all side
effects so gates are unit-testable without a simulator.
"""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import os
import re
import subprocess
import sys
import time
import yaml
from dataclasses import dataclass
from pathlib import Path

# ----------------------------------------------------------------- constants
REPO = "/home/thinhpham/Documents/Husky_viz"
NS = "/a200_0000"
ROBOT_MODEL = "a200_0000/robot"
STATE_FILE = Path.home() / ".husky_sim" / "state.json"
SIM_LOG = "/tmp/sim.log"
NAV_LOG = "/tmp/nav.log"
NAV_DEADLINE = 60.0
ROS_SETUP = "source /opt/ros/jazzy/setup.bash"

PHASE_NAMES = ["clean", "config", "launch", "controllers", "robot", "extras", "nav2"]

SENSOR_TYPES = ("camera", "lidar2d", "lidar3d", "imu", "gps")

# ---- pure gates

EXTRA_SWEEP = ["a200_0000", "gz sim", "gz_tools_vendor"]


def parse_kill_patterns(kill_sim_text: str) -> list[str]:
    m = re.search(r"PATTERNS=\((.*?)\)", kill_sim_text, re.S)
    if not m:
        raise ValueError("no PATTERNS=( ... ) array in kill_sim.sh")
    out = []
    for line in m.group(1).splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        q = re.match(r'"([^"]+)"', line)
        if q:
            out.append(q.group(1))
    return out


def find_sim_pids(ps_lines: str, patterns: list[str], self_pid: int,
                  self_path: str) -> list[tuple[int, str]]:
    found = []
    for line in ps_lines.splitlines():
        line = line.strip()
        if not line:
            continue
        pid_s, _, cmd = line.partition(" ")
        try:
            pid = int(pid_s)
        except ValueError:
            continue  # malformed ps line (e.g. a wrapped/truncated row) — not a pid we can act on
        if pid == self_pid or "bash -c" in cmd or self_path in cmd:
            continue
        if any(p in cmd for p in patterns):
            found.append((pid, cmd))
    return found


def shm_count(ls_dev_shm: str) -> int:
    return sum(1 for n in ls_dev_shm.split() if "fastrtps" in n)


def declared_sensors(robot_yaml_text: str) -> set[str]:
    doc = yaml.safe_load(robot_yaml_text) or {}
    out = set()
    for t in SENSOR_TYPES:
        for i, s in enumerate((doc.get("sensors") or {}).get(t) or []):
            if s.get("urdf_enabled", True) or s.get("launch_enabled", True):
                out.add(f"{t}_{i}")
    return out


def parse_sdf_sensors(apply_output: str) -> set[str]:
    out, inblock = set(), False
    for line in apply_output.splitlines():
        if line.startswith("==> sensors in SDF:"):
            inblock = True
            continue
        if inblock:
            m = re.match(r"\s+(\S+)\s+->\s+\S+", line)
            if m:
                out.add(m.group(1))
            else:
                inblock = False
    return out


@dataclass
class PhaseResult:
    phase: int
    name: str
    status: str   # ok | skip | fail
    detail: str = ""


def format_line(r: PhaseResult) -> str:
    return f"[{r.phase} {r.name:<11}] {r.status:<4} {r.detail}".rstrip()


def exit_code(results: list[PhaseResult]) -> int:
    for r in results:
        if r.status == "fail":
            return 10 + r.phase
    return 0


# ---------------------------------------------------------------- Shell
class Shell:
    """Owns all real side effects (subprocess, filesystem, clock). Pure
    gates above never touch any of this directly, so they stay unit-testable
    without a simulator."""

    def __init__(self):
        self.self_pid = os.getpid()
        self.self_path = os.path.abspath(__file__)
        self.last_rc = 0

    def run(self, cmd: str, timeout: float = 30) -> str:
        full = f'set -eo pipefail; {ROS_SETUP}; {cmd}'
        try:
            p = subprocess.run(["bash", "-lc", full], capture_output=True, text=True,
                               timeout=timeout)
            self.last_rc = p.returncode
            return p.stdout + p.stderr
        except subprocess.TimeoutExpired as e:
            self.last_rc = 124
            out = e.stdout if isinstance(e.stdout, str) else ""
            err = e.stderr if isinstance(e.stderr, str) else ""
            return out + err

    def ps(self) -> str:
        return subprocess.run(["ps", "-eo", "pid,cmd", "--no-headers"],
                              capture_output=True, text=True).stdout

    def kill9(self, pids: list[int]) -> None:
        for p in pids:
            if p == self.self_pid:
                continue  # never kill our own invocation (CLAUDE.md: pkill -f matching self)
            try:
                os.kill(p, 9)
            except ProcessLookupError:
                pass

    def ls_shm(self) -> str:
        return "\n".join(os.listdir("/dev/shm"))

    def rm_shm(self) -> None:
        for n in os.listdir("/dev/shm"):
            if n.startswith("fastrtps_") or n.startswith("sem.fastrtps_"):
                try:
                    os.remove(os.path.join("/dev/shm", n))
                except OSError:
                    pass

    def daemon_stop(self) -> None:
        self.run("ros2 daemon stop >/dev/null 2>&1 || true", timeout=20)

    def read(self, path: str) -> str:
        return Path(path).read_text()

    def pid_alive(self, pid: int) -> bool:
        try:
            os.kill(pid, 0)
            return True
        except ProcessLookupError:
            return False

    def now(self) -> float:
        return time.monotonic()

    def pause(self, sec: float) -> None:
        time.sleep(sec)

    def launch(self, cmd: str, log: str) -> int:
        with open(log, "wb") as f:
            p = subprocess.Popen(["bash", "-lc", f"{ROS_SETUP}; exec {cmd}"],
                                 stdout=f, stderr=subprocess.STDOUT,
                                 stdin=subprocess.DEVNULL, start_new_session=True)
        return p.pid

    def world_stats(self, world: str) -> str:
        return self.run(f"gz topic -e -t /world/{world}/stats -n 1", timeout=5)

    def receive(self, topics: dict, deadline: float) -> dict[str, int]:
        """Deadline-bounded liveness check, NOT a rate measurement: subscribes
        to each topic with sensor-data QoS and spins until every topic has
        received at least one message or the deadline passes, returning the
        per-topic message COUNT observed in that window. The window ends as
        soon as the slowest topic arrives, so counts are not comparable
        across topics and must never be reported as Hz. For actual rates use
        the project's tools/check_*.py scripts."""
        import importlib
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        rclpy.init()
        node = Node("sim_py_receive")
        counts = {k: 0 for k in topics}
        for k, (topic, typ) in topics.items():
            pkg, _, cls = typ.split("/")
            msg_cls = getattr(importlib.import_module(f"{pkg}.msg"), cls)
            node.create_subscription(msg_cls, topic,
                                     lambda _m, k=k: counts.__setitem__(k, counts[k] + 1),
                                     qos_profile_sensor_data)
        t0 = time.monotonic()
        while time.monotonic() - t0 < deadline and not all(counts.values()):
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()
        return counts

    def gz_pose(self) -> str:
        return self.run(f"gz model -m {ROBOT_MODEL} -p", timeout=10)

    def nav_ready(self) -> list[str]:
        sys.path.append(REPO)
        from tools.check_nav2_ready import nav_ready
        with contextlib.redirect_stdout(io.StringIO()):     # its per-check prints are noise here
            return nav_ready(lambda cmd, timeout=None: self.run(cmd, timeout=timeout or 30))


def poll(shell, deadline_s: float, probe, interval: float = 0.5):
    """Call `probe()` until it returns truthy or the deadline passes.
    The only place a wait ever happens; every wait in this program is a
    deadline-bounded poll of a real signal, never a fixed sleep."""
    end = shell.now() + deadline_s
    while True:
        v = probe()
        if v:
            return v
        if shell.now() >= end:
            return None
        shell.pause(interval)


# --------------------------------------------------------------- phases
def _patterns(shell) -> list[str]:
    return parse_kill_patterns(shell.read(f"{REPO}/scripts/kill_sim.sh")) + EXTRA_SWEEP


def phase_clean(shell) -> PhaseResult:
    pats = _patterns(shell)
    victims = find_sim_pids(shell.ps(), pats, shell.self_pid, shell.self_path)
    shell.kill9([p for p, _ in victims])
    shell.daemon_stop()
    shell.rm_shm()

    # Verify survivors: poll directly on the survivor list, remembering the
    # last observed reading in `seen` so the poll's own result is
    # authoritative — no redundant duplicate ps() call after it returns.
    # (probe returns a truthy sentinel once the list is empty, since an
    # empty list itself is falsy and must not be mistaken for "still polling".)
    seen = {"left": []}

    def _survivors_probe():
        seen["left"] = find_sim_pids(shell.ps(), pats, shell.self_pid, shell.self_path)
        return True if not seen["left"] else None

    poll(shell, 15.0, _survivors_probe)
    left = seen["left"]
    if left:
        lines = "; ".join(f"{p} {c}" for p, c in left[:5])
        return PhaseResult(0, "clean", "fail", f"survivors: {lines}")

    n = shm_count(shell.ls_shm())
    if n:
        n = shm_count(shell.ls_shm())  # transient release: re-read once (CLAUDE.md #12)
    if n:
        return PhaseResult(0, "clean", "fail", f"shm {n} persists after kill (something is alive)")
    return PhaseResult(0, "clean", "ok", f"killed {len(victims)}, shm 0")


def phase_config(shell, config: str) -> PhaseResult:
    path = f"{REPO}/robot_configs/robot_{config}.yaml"
    try:
        declared = declared_sensors(shell.read(path))
    except (FileNotFoundError, KeyError):
        return PhaseResult(1, "config", "fail", f"no such config: robot_{config}.yaml")
    out = shell.run(f"{REPO}/scripts/apply_config.sh {config}", timeout=180)
    sdf = parse_sdf_sensors(out)
    missing = sorted(declared - sdf)
    if missing:
        return PhaseResult(1, "config", "fail",
                           f"{config}: declared but absent from SDF: {' '.join(missing)} (CLAUDE.md #1)")
    return PhaseResult(1, "config", "ok", f"{config}  (sensors: {' '.join(sorted(sdf))})")


LAUNCH_DEADLINE = 90.0


def parse_sim_time(stats_msg: str) -> float | None:
    m = re.search(r"sim_time \{\s*(?:sec: (\d+))?\s*(?:nsec: (\d+))?\s*\}", stats_msg)
    if not m or (m.group(1) is None and m.group(2) is None):
        return None
    return int(m.group(1) or 0) + int(m.group(2) or 0) / 1e9


def pose_args(ns) -> str:
    return " ".join(f"{k}:={getattr(ns, k)}" for k in ("x", "y", "z", "yaw")
                    if getattr(ns, k) is not None)


def phase_launch(shell, world: str, pose: str) -> tuple[PhaseResult, int]:
    cmd = f"ros2 launch {REPO}/launch/park_sim.launch.py world:={world} {pose}".strip()
    pid = shell.launch(cmd, SIM_LOG)
    t0 = shell.now()
    last = {"t": None}

    def probe():
        if not shell.pid_alive(pid):
            return "dead"
        t = parse_sim_time(shell.world_stats(world))
        prev, last["t"] = last["t"], t
        if t is not None and prev is not None and t > prev:
            return "stepping"
        return None

    v = poll(shell, LAUNCH_DEADLINE, probe, interval=1.0)
    if v == "dead":
        return PhaseResult(2, "launch", "fail",
                           f"ros2 launch (pid {pid}) is no longer running - see {SIM_LOG}"), pid
    if v != "stepping":
        return PhaseResult(2, "launch", "fail",
                           f"{world} not stepping after {LAUNCH_DEADLINE:.0f} s (sim_time={last['t']}) - see {SIM_LOG}"), pid
    return PhaseResult(2, "launch", "ok",
                       f"pid {pid}, {world} stepping after {shell.now() - t0:.1f} s"), pid


CONTROLLERS = ["joint_state_broadcaster", "platform_velocity_controller"]
CLEAN_WAIT = 10.0


def controller_states(list_output: str) -> dict[str, str]:
    return dict(re.findall(r"name='([^']+)', state='([^']+)'", list_output))


def _query_controllers(shell) -> dict[str, str]:
    out = shell.run(f"ros2 service call {NS}/controller_manager/list_controllers "
                    f"controller_manager_msgs/srv/ListControllers '{{}}' 2>/dev/null", timeout=10)
    return controller_states(out)


def _describe(states: dict[str, str]) -> str:
    return ", ".join(f"{c} {'was ' + states[c] if c in states else 'missing'}"
                     for c in CONTROLLERS if states.get(c) != "active")


def phase_controllers(shell) -> PhaseResult:
    def all_active():
        st = _query_controllers(shell)
        return st if all(st.get(c) == "active" for c in CONTROLLERS) else None
    st = poll(shell, CLEAN_WAIT, all_active)
    if st:
        return PhaseResult(3, "controllers", "ok", "clean")
    before = _query_controllers(shell)
    t0 = shell.now()
    shell.run(f"ros2 run controller_manager spawner {' '.join(CONTROLLERS)} "
              f"--controller-manager {NS}/controller_manager --switch-timeout 30", timeout=45)
    after = _query_controllers(shell)
    if all(after.get(c) == "active" for c in CONTROLLERS):
        return PhaseResult(3, "controllers", "ok",
                           f"recovered  ({_describe(before)}; respawned in {shell.now() - t0:.1f} s)")
    return PhaseResult(3, "controllers", "fail",
                       f"after spawner --switch-timeout 30: {_describe(after)} (CLAUDE.md #27)")


ROBOT_TOPICS = {
    "odom":   (f"{NS}/platform/odom",            "nav_msgs/msg/Odometry"),
    "imu":    (f"{NS}/sensors/imu_0/data",       "sensor_msgs/msg/Imu"),
    "scan":   (f"{NS}/sensors/lidar2d_0/scan",   "sensor_msgs/msg/LaserScan"),
    "points": (f"{NS}/sensors/lidar3d_0/points", "sensor_msgs/msg/PointCloud2"),
}
ROBOT_DEADLINE = 10.0
LANDED_TOL = 0.5


def parse_gz_pose(gz_model_output: str) -> tuple[float, float, float] | None:
    m = re.search(r"Pose[^\n]*\n\s*\[([^\]]+)\]", gz_model_output)
    if not m:
        return None
    try:
        v = tuple(float(x) for x in m.group(1).split())
    except ValueError:
        return None
    return v if len(v) == 3 else None


def spawn_z(world: str, override: float | None) -> float | None:
    if override is not None:
        return float(override)
    text = Path(f"{REPO}/launch/park_sim.launch.py").read_text()
    m = re.search(r"'%s': \{[^}]*'z': '([\d.\-]+)'" % re.escape(world), text)
    return float(m.group(1)) if m else 0.3


def phase_robot(shell, world: str, z_override: float | None) -> PhaseResult:
    counts = shell.receive(ROBOT_TOPICS, ROBOT_DEADLINE)
    silent = [k for k, c in counts.items() if c <= 0]
    if silent:
        return PhaseResult(4, "robot", "fail",
                           f"no messages within {ROBOT_DEADLINE:.0f} s on: {' '.join(silent)}")
    pose = parse_gz_pose(shell.gz_pose())
    if pose is None:
        return PhaseResult(4, "robot", "fail", f"gz model returned no pose for {ROBOT_MODEL}")
    zs = spawn_z(world, z_override)
    if abs(pose[2] - zs) > LANDED_TOL:
        return PhaseResult(4, "robot", "fail",
                           f"z={pose[2]:.2f} vs spawn z={zs:.2f}: fell through terrain? (CLAUDE.md #23)")
    received = sum(1 for c in counts.values() if c > 0)
    return PhaseResult(4, "robot", "ok",
                       f"pose {pose[0]:.2f} {pose[1]:.2f} {pose[2]:.2f}  "
                       f"{received}/{len(ROBOT_TOPICS)} topics receiving")


BRIDGE_LOG = "/tmp/bridge.log"


def extras_features(robot_yaml_text: str, read) -> set[str]:
    doc = yaml.safe_load(robot_yaml_text) or {}
    path = (((doc.get("platform") or {}).get("extras") or {}).get("urdf") or {}).get("path")
    if not path:
        return set()
    text = read(path)
    feats = set()
    if "compass.urdf.xacro" in text:
        feats.add("compass")
    if "comms.urdf.xacro" in text:
        feats.add("radio")
    return feats


def bridge_args(features) -> list[str]:
    a = []
    if "compass" in features:
        a.append(f"{NS}/sensors/compass_0/mag@sensor_msgs/msg/MagneticField[gz.msgs.Magnetometer")
    if "radio" in features:
        a += ["/broker/msgs@ros_gz_interfaces/msg/Dataframe]gz.msgs.Dataframe",
              "/husky/rx@ros_gz_interfaces/msg/Dataframe[gz.msgs.Dataframe",
              "/base_station/rx@ros_gz_interfaces/msg/Dataframe[gz.msgs.Dataframe"]
    return a


def phase_extras(shell, config: str, launch_pid: int):
    feats = extras_features(shell.read(f"{REPO}/robot_configs/robot_{config}.yaml"), shell.read)
    if not feats:
        return PhaseResult(5, "extras", "skip", f"{config} config has no compass/radio"), None
    args = " ".join(f"'{a}'" for a in bridge_args(feats))
    pid = shell.launch(f"ros2 run ros_gz_bridge parameter_bridge {args} "
                       f"--ros-args -r __node:=extras_gz_bridge", BRIDGE_LOG)
    ok = poll(shell, 5.0, lambda: shell.pid_alive(pid) and shell.pid_alive(launch_pid))
    if not ok:
        return PhaseResult(5, "extras", "fail", f"bridge or launch died - see {BRIDGE_LOG}"), pid
    return PhaseResult(5, "extras", "ok", f"bridged {' '.join(sorted(feats))} (pid {pid})"), pid


def nav_config(world: str):
    p = f"{REPO}/config/nav2_{world}.yaml"
    return p if os.path.exists(p) else None


def phase_nav2(shell, world: str, no_nav: bool):
    cfg = nav_config(world)
    if cfg is None:
        return PhaseResult(6, "nav2", "skip", f"no config/nav2_{world}.yaml"), None
    if no_nav:
        return PhaseResult(6, "nav2", "skip", "--no-nav"), None
    pid = shell.launch(f"ros2 launch {REPO}/launch/nav_park.launch.py", NAV_LOG)
    last = {"f": ["not checked"]}

    def probe():
        if not shell.pid_alive(pid):
            return "dead"
        last["f"] = shell.nav_ready()
        return "ready" if not last["f"] else None

    v = poll(shell, NAV_DEADLINE, probe, interval=2.0)
    if v == "dead":
        return PhaseResult(6, "nav2", "fail", f"nav launch (pid {pid}) died - see {NAV_LOG}"), pid
    if v != "ready":
        return PhaseResult(6, "nav2", "fail",
                           f"not ready after {NAV_DEADLINE:.0f} s: {'; '.join(last['f'][:3])} - see {NAV_LOG}"), pid
    return PhaseResult(6, "nav2", "ok", "map->odom present, all lifecycle nodes active"), pid


# ----------------------------------------------------------------------- CLI
def parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(prog="sim.py")
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("start")
    s.add_argument("world")
    s.add_argument("--config", default="default")
    s.add_argument("--no-nav", action="store_true")
    s.add_argument("--clean-on-fail", action="store_true")
    for k in ("x", "y", "z", "yaw"):
        s.add_argument(f"--{k}", type=float, default=None)
    sub.add_parser("stop")
    sub.add_parser("status")
    return p.parse_args(argv)


def save_state(path: Path, d: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(d, indent=2))


def load_state(path: Path):
    try:
        return json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return None


def cmd_start(shell, args, out=print) -> int:
    results: list[PhaseResult] = []
    state = {"world": args.world, "config": args.config, "launch_pid": None,
             "bridge_pid": None, "nav_pid": None, "started_at": time.time(), "phase_reached": -1}

    def record(r: PhaseResult) -> bool:
        results.append(r)
        out(format_line(r))
        state["phase_reached"] = r.phase
        return r.status != "fail"

    def finish() -> int:
        save_state(STATE_FILE, state)
        rc = exit_code(results)
        if rc:
            f = results[-1]
            out(f"FAIL {f.phase} {f.name}: {f.detail}")
            if getattr(args, "clean_on_fail", False):
                record(phase_clean(shell))
        else:
            nav = " nav" if results[6].status == "ok" else ""
            out(f"READY {args.world} {args.config}{nav}")
        return rc

    if not record(phase_clean(shell)):
        return finish()
    if not record(phase_config(shell, args.config)):
        return finish()
    r, pid = phase_launch(shell, args.world, pose_args(args))
    state["launch_pid"] = pid
    save_state(STATE_FILE, state)
    if not record(r):
        return finish()
    if not record(phase_controllers(shell)):
        return finish()
    if not record(phase_robot(shell, args.world, args.z)):
        return finish()
    r, bpid = phase_extras(shell, args.config, pid)
    state["bridge_pid"] = bpid
    if not record(r):
        return finish()
    r, npid = phase_nav2(shell, args.world, args.no_nav)
    state["nav_pid"] = npid
    record(r)
    return finish()


def cmd_stop(shell, out=print) -> int:
    r = phase_clean(shell)
    out(format_line(r))
    try:
        STATE_FILE.unlink()
    except FileNotFoundError:
        pass
    out("CLEAN" if r.status == "ok" else f"FAIL 0 clean: {r.detail}")
    return exit_code([r])


def cmd_status(shell, out=print) -> int:
    st = load_state(STATE_FILE) or {}
    if not st:
        out("no state file - probing anyway")
    world, config = st.get("world", "?"), st.get("config", "?")
    results = []
    lp = st.get("launch_pid")
    alive = bool(lp) and shell.pid_alive(lp)
    t1 = parse_sim_time(shell.world_stats(world)) if world != "?" else None
    t2 = parse_sim_time(shell.world_stats(world)) if t1 is not None else None
    stepping = t1 is not None and t2 is not None and t2 > t1
    results.append(PhaseResult(2, "launch", "ok" if alive and stepping else "fail",
                               f"pid {lp} alive={alive} stepping={stepping}"))
    cs = _query_controllers(shell)
    ok = all(cs.get(c) == "active" for c in CONTROLLERS)
    results.append(PhaseResult(3, "controllers", "ok" if ok else "fail", _describe(cs) or "both active"))
    if world != "?":
        results.append(phase_robot(shell, world, None))
    bp = st.get("bridge_pid")
    if bp:
        results.append(PhaseResult(5, "extras", "ok" if shell.pid_alive(bp) else "fail", f"bridge pid {bp}"))
    np_ = st.get("nav_pid")
    if np_:
        f = shell.nav_ready() if shell.pid_alive(np_) else ["nav launch dead"]
        results.append(PhaseResult(6, "nav2", "ok" if not f else "fail", "; ".join(f[:3]) or "ready"))
    for r in results:
        out(format_line(r))
    rc = exit_code(results)
    out(f"{'READY' if rc == 0 else 'NOT READY'} {world} {config}")
    return rc


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    shell = Shell()
    if args.cmd == "start":
        return cmd_start(shell, args)
    if args.cmd == "stop":
        return cmd_stop(shell)
    return cmd_status(shell)


if __name__ == "__main__":
    sys.exit(main())
