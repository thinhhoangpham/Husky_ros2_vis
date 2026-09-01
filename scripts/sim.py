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
NAV_DEADLINE = 180.0
# NAV_DEADLINE is wall-clock but nav2 bring-up is paced by simulated time, so
# under a slow RTF (CLAUDE.md: full config's camera drags park to 0.10-0.14)
# the raw deadline is denominated in the wrong clock. Scale it by the
# measured real_time_factor instead.
#
# RTF_FLOOR = 0.05: half the worst measured RTF (0.10), so a genuinely
# stalled/near-zero RTF still produces a large-but-finite budget rather than
# a division blow-up toward infinity.
# NAV_DEADLINE_MAX = 1500.0: at the worst observed RTF (0.12) the scaled
# budget is 180/0.12 = 1500 s, so the cap is set to exactly cover that case
# (comfortably above the ~20 s of simulated time 180 s of wall clock buys at
# RTF 0.12) while still bounding a genuinely dead nav2 stack to 25 minutes
# rather than letting RTF_FLOOR blow the budget out further.
RTF_FLOOR = 0.05
NAV_DEADLINE_MAX = 1500.0
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


def gz_world_name(world: str, worlds_dir: str = f"{REPO}/worlds") -> str:
    """Resolve the gz world NAME declared inside worlds/<world>.sdf.

    warehouse_ext.sdf and warehouse_ramp.sdf both declare
    `<world name='warehouse'>` -- NOT their file basename -- so any
    /world/<name>/... gz topic or service must be addressed by this value,
    not by the file basename. Falls back to the basename if the file is
    missing or carries no `<world name=...>` attribute (park.sdf and
    lake.sdf both match their basename already, so the fallback is a no-op
    for them). Does NOT change what is passed as the launch file's
    `world:=` argument or the spawn-pose lookup key -- those correctly use
    the basename.
    """
    path = Path(worlds_dir) / f"{world}.sdf"
    try:
        text = path.read_text()
    except OSError:
        return world
    m = re.search(r"<world\s+name=['\"]([^'\"]+)['\"]", text)
    return m.group(1) if m else world


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
        # Popen objects for children THIS process spawned, keyed by pid, so
        # pid_alive can consult Popen.poll() instead of os.kill(pid, 0) -
        # the latter reports True for a zombie (exited, not yet reaped),
        # which is exactly the false-positive that let a dead bridge/launch
        # read as "alive" (CLAUDE.md review finding 1).
        self._children: dict[int, subprocess.Popen] = {}

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
        # A child we spawned ourselves: ask the OS for its real exit status
        # via Popen.poll(), which reaps it and returns non-None once it has
        # exited. `status` reads pids back from the state file across
        # process boundaries, where there is no Popen to consult, so that
        # case falls back to the os.kill(pid, 0) probe.
        p = self._children.get(pid)
        if p is not None:
            return p.poll() is None
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
        self._children[p.pid] = p
        return p.pid

    def world_stats(self, world: str) -> str:
        return self.run(f"gz topic -e -t /world/{gz_world_name(world)}/stats -n 1", timeout=5)

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
        node = None
        try:
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
            return counts
        finally:
            # If subscribing/spinning raises, rclpy must still be shut down
            # here - otherwise a later rclpy.init() (e.g. in nav_ready())
            # fails too, poisoning the rest of the same run (CLAUDE.md
            # review finding 3).
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()

    def gz_pose(self) -> str:
        return self.run(f"gz model -m {ROBOT_MODEL} -p", timeout=10)

    def scene_info(self, world: str) -> str:
        return self.run(
            f"gz service -s /world/{gz_world_name(world)}/scene/info "
            f"--reqtype gz.msgs.Empty --reptype gz.msgs.Scene --timeout 30000 --req ''",
            timeout=35)

    def gui_alive(self) -> bool:
        # A `gz sim` process that is NOT the server, per sim-operator.md's
        # documented form. `|| true` absorbs grep's nonzero exit when no
        # matching line is left, which `set -eo pipefail` would otherwise
        # turn into a hard failure of the whole pipeline.
        out = self.run('pgrep -af "gz sim" | grep -v "bash -c" | grep -v server || true',
                       timeout=10)
        return bool(out.strip())

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

# Per-world spawn delay (seconds) passed to park_sim.launch.py's `spawn_delay`
# argument. Sequences the robot spawn after the world's GUI has had time to
# load its scene, so gz-sim's GUI cannot miss the spawn's model-creation
# event mid-load (CLAUDE.md; .claude/agents/sim-operator.md). Only park needs
# it: it is the heaviest world (~97 models, ~221 MB of textures including a
# 46 MB normal map used 16x) and the only one observed losing the robot's
# visual ~1/3 of runs; warehouse and lake are far lighter and load reliably,
# so leaving them at 0.0 costs them nothing. park itself reaches "stepping"
# (server ready) at ~6.2-6.3 s; 15.0 s is a conservative multiple of that,
# chosen because no GUI-side "scene finished loading" signal exists to
# measure the real completion time against (see park_sim.launch.py) - it is
# an engineering estimate, not a measured value, and cheap against the 90 s
# LAUNCH_DEADLINE budget.
SPAWN_DELAY_S = {"park": 15.0}


def spawn_delay_for(world: str) -> float:
    return SPAWN_DELAY_S.get(world, 0.0)


def parse_sim_time(stats_msg: str) -> float | None:
    m = re.search(r"sim_time \{\s*(?:sec: (\d+))?\s*(?:nsec: (\d+))?\s*\}", stats_msg)
    if not m or (m.group(1) is None and m.group(2) is None):
        return None
    return int(m.group(1) or 0) + int(m.group(2) or 0) / 1e9


def parse_rtf(stats_msg: str) -> float | None:
    m = re.search(r"real_time_factor:\s*([\d.eE+-]+)", stats_msg)
    if not m:
        return None
    try:
        return float(m.group(1))
    except ValueError:
        return None


def nav_deadline(rtf: float | None) -> float:
    if rtf is None:
        return NAV_DEADLINE
    return min(NAV_DEADLINE / max(rtf, RTF_FLOOR), NAV_DEADLINE_MAX)


def pose_args(ns) -> str:
    return " ".join(f"{k}:={getattr(ns, k)}" for k in ("x", "y", "z", "yaw")
                    if getattr(ns, k) is not None)


def phase_launch(shell, world: str, pose: str) -> tuple[PhaseResult, int]:
    delay = spawn_delay_for(world)
    cmd = (f"ros2 launch {REPO}/launch/park_sim.launch.py world:={world} "
           f"spawn_delay:={delay} {pose}").strip()
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


def phase_controllers(shell, spawn_delay: float = 0.0) -> PhaseResult:
    # The robot (and so the controller_manager) does not exist until
    # spawn_delay has elapsed inside the launch, so the "clean" wait budget
    # has to cover that delay on top of the ordinary spawn/activation time -
    # otherwise a delayed-but-healthy spawn would be misdiagnosed as needing
    # the phase-3 recovery spawner.
    def all_active():
        st = _query_controllers(shell)
        return st if all(st.get(c) == "active" for c in CONTROLLERS) else None
    st = poll(shell, CLEAN_WAIT + spawn_delay, all_active)
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


RENDERER_FAIL_MARKER = "GUI missed the spawn"


def scene_robot_count(scene_info_text: str) -> int:
    """Pure count of `ROBOT_MODEL` occurrences in a `scene/info` service
    reply, mirroring the documented renderer gate
    (.claude/agents/sim-operator.md, "It is up" means the renderer too):
    `gz service .../scene/info ... | grep -c 'a200_0000/robot'`.

    CAVEAT (documented honestly, not hidden): this reflects the SERVER's
    scene graph, not what the GUI actually rendered. The user's own notes
    treat count==1 as the renderer check, but on the failing runs that
    prompted this gate the count was reported as 1 while the robot was
    still invisible in the GUI window - so this check, run alone, is not
    guaranteed to catch every renderer-miss. It is implemented exactly as
    documented because that is the only check `sim-operator.md` specifies;
    no more reliable GUI-side signal (a GUI-specific scene/state topic) was
    found to exist for gz-sim Harmonic 8.11 during this fix - see the
    session report for what was searched.
    """
    return sum(1 for line in scene_info_text.splitlines() if ROBOT_MODEL in line)


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
    scene_count = scene_robot_count(shell.scene_info(world))
    gui_up = shell.gui_alive()
    if scene_count != 1 or not gui_up:
        return PhaseResult(4, "robot", "fail",
                           f"robot in physics but NOT in the renderer ({RENDERER_FAIL_MARKER}); "
                           f"scene count {scene_count}, GUI {'alive' if gui_up else 'none'} "
                           "- remedy is a full CLEAN_SIM.md + RUN_SIM.md restart")

    received = sum(1 for c in counts.values() if c > 0)
    return PhaseResult(4, "robot", "ok",
                       f"pose {pose[0]:.2f} {pose[1]:.2f} {pose[2]:.2f}  "
                       f"{received}/{len(ROBOT_TOPICS)} topics receiving  renderer ok")


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

    # A bridge advertises its topics whether or not anything is behind them
    # (CLAUDE.md: run_husky_sim.sh waited 12 s for exactly this reason), so
    # confirming liveness at t=0 is not a check at all - `poll` returning on
    # the FIRST truthy probe made this an instantaneous check, not "still
    # alive after a few seconds" (CLAUDE.md review finding 1c). Instead,
    # fail fast if death is detected during the window, but only confirm
    # success by checking liveness again once the window has fully elapsed.
    def dead_probe():
        return "dead" if not (shell.pid_alive(pid) and shell.pid_alive(launch_pid)) else None

    if poll(shell, 5.0, dead_probe) == "dead":
        return PhaseResult(5, "extras", "fail", f"bridge or launch died - see {BRIDGE_LOG}"), pid
    if not (shell.pid_alive(pid) and shell.pid_alive(launch_pid)):
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

    rtf = parse_rtf(shell.world_stats(world))
    deadline = nav_deadline(rtf)
    rtf_note = (f"rtf {rtf:.2f}, budget scaled from {NAV_DEADLINE:.0f} s"
                if rtf is not None else
                f"rtf unreadable, budget not scaled from {NAV_DEADLINE:.0f} s")

    def probe():
        if not shell.pid_alive(pid):
            return "dead"
        last["f"] = shell.nav_ready()
        return "ready" if not last["f"] else None

    v = poll(shell, deadline, probe, interval=2.0)
    if v == "dead":
        return PhaseResult(6, "nav2", "fail", f"nav launch (pid {pid}) died - see {NAV_LOG}"), pid
    if v != "ready":
        return PhaseResult(6, "nav2", "fail",
                           f"not ready after {deadline:.0f} s ({rtf_note}): "
                           f"{'; '.join(last['f'][:3])} - see {NAV_LOG}"), pid
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
    s.add_argument("--no-retry", action="store_true")
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


# The renderer gate (phase_robot's scene/GUI check) is the one intermittent,
# environment-timing failure in this pipeline - the documented remedy is a
# full clean+restart, and it was observed failing ~1 in 3 starts. 3 total
# attempts (1 initial + 2 retries) pushes the odds of exhausting all
# attempts down to roughly (1/3)^3 ~= 3.7% while keeping a hung retry loop
# bounded and cheap - each attempt is a full CLEAN_SIM.md-equivalent cycle,
# not a quick recheck.
RETRY_ATTEMPTS = 3


def _run_start_attempt(shell, args, out) -> tuple[list[PhaseResult], dict]:
    """One full clean -> ... -> nav2 cycle. No retry logic in here - that
    lives in cmd_start, which decides whether the whole cycle is worth
    repeating."""
    results: list[PhaseResult] = []
    state = {"world": args.world, "config": args.config, "no_nav": args.no_nav,
             "launch_pid": None, "bridge_pid": None, "nav_pid": None,
             "started_at": time.time(), "phase_reached": -1}

    def record(r: PhaseResult) -> bool:
        results.append(r)
        out(format_line(r))
        state["phase_reached"] = r.phase
        return r.status != "fail"

    if not record(phase_clean(shell)):
        return results, state
    if not record(phase_config(shell, args.config)):
        return results, state
    r, pid = phase_launch(shell, args.world, pose_args(args))
    state["launch_pid"] = pid
    save_state(STATE_FILE, state)
    if not record(r):
        return results, state
    if not record(phase_controllers(shell, spawn_delay_for(args.world))):
        return results, state
    if not record(phase_robot(shell, args.world, args.z)):
        return results, state
    r, bpid = phase_extras(shell, args.config, pid)
    state["bridge_pid"] = bpid
    if not record(r):
        return results, state
    r, npid = phase_nav2(shell, args.world, args.no_nav)
    state["nav_pid"] = npid
    record(r)
    return results, state


def _renderer_gate_failure(results: list[PhaseResult]) -> bool:
    """True only when the run's SOLE failure is phase 4 (robot) failing for
    the renderer-gate reason - never for a silent topic or a robot that fell
    through the terrain, and never when an earlier phase also failed. Only
    this case is worth burning a retry on, per the task's retry scope."""
    fails = [r for r in results if r.status == "fail"]
    return (len(fails) == 1 and fails[0].phase == 4
            and RENDERER_FAIL_MARKER in fails[0].detail)


def cmd_start(shell, args, out=print) -> int:
    max_attempts = 1 if getattr(args, "no_retry", False) else RETRY_ATTEMPTS
    attempt = 1
    while True:
        results, state = _run_start_attempt(shell, args, out)
        rc = exit_code(results)

        if rc == 0:
            nav = " nav" if results[6].status == "ok" else ""
            suffix = f" (attempt {attempt} of {max_attempts})" if attempt > 1 else ""
            out(f"READY {args.world} {args.config}{nav}{suffix}")
            save_state(STATE_FILE, state)
            return 0

        if _renderer_gate_failure(results) and attempt < max_attempts:
            out(f"retrying (attempt {attempt + 1} of {max_attempts}) after renderer gate failure - "
                f"restarting the whole cycle clean")
            attempt += 1
            continue

        f = results[-1]
        out(f"FAIL {f.phase} {f.name}: {f.detail}")
        if getattr(args, "clean_on_fail", False):
            r = phase_clean(shell)
            results.append(r)
            out(format_line(r))
            state["phase_reached"] = r.phase
        save_state(STATE_FILE, state)
        return rc


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

    # What must be running is derived from CONFIGURATION, not from what a
    # past `start` happened to record - a start that failed at phase 3/4
    # leaves bridge_pid/nav_pid as None forever, and checking only `if bp` /
    # `if np_` let a later status skip those phases entirely and print
    # READY with no bridge and no nav2 running (CLAUDE.md review finding 2).
    try:
        required_feats = (extras_features(shell.read(f"{REPO}/robot_configs/robot_{config}.yaml"), shell.read)
                          if config != "?" else set())
    except (FileNotFoundError, KeyError):
        required_feats = set()

    bp = st.get("bridge_pid")
    bp_alive = bool(bp) and shell.pid_alive(bp)
    if required_feats:
        results.append(PhaseResult(5, "extras", "ok" if bp_alive else "fail",
                                   f"bridge pid {bp}" if bp_alive else
                                   f"{config} requires {' '.join(sorted(required_feats))} "
                                   "but the extras bridge is not running"))
    elif bp:
        results.append(PhaseResult(5, "extras", "ok" if bp_alive else "fail", f"bridge pid {bp}"))

    nav_required = world != "?" and nav_config(world) is not None and not st.get("no_nav", False)
    np_ = st.get("nav_pid")
    np_alive = bool(np_) and shell.pid_alive(np_)
    if nav_required:
        if np_alive:
            f = shell.nav_ready()
            results.append(PhaseResult(6, "nav2", "ok" if not f else "fail", "; ".join(f[:3]) or "ready"))
        else:
            results.append(PhaseResult(6, "nav2", "fail",
                           f"nav2 is required for {world} but the nav launch is not running"))
    elif np_:
        f = shell.nav_ready() if np_alive else ["nav launch dead"]
        results.append(PhaseResult(6, "nav2", "ok" if not f else "fail", "; ".join(f[:3]) or "ready"))

    for r in results:
        out(format_line(r))
    rc = exit_code(results)
    nav_ok = any(r.phase == 6 and r.status == "ok" for r in results)
    out(f"{'READY' if rc == 0 else 'NOT READY'} {world} {config}{' nav' if nav_ok else ''}")
    return rc


def check_ros_env() -> str | None:
    """Return an error message if ROS is not sourced in this process, else None.

    Every phase eventually shells out to `ros2`/rclpy; without ROS sourced
    the first such use dies with a bare ModuleNotFoundError traceback and no
    verdict line, breaking this tool's "last line is the verdict" contract.
    Detect it up front via importlib.util.find_spec, which only resolves the
    module location and never imports rclpy - so the ROS-free unit suite
    stays untouched.
    """
    import importlib.util
    if importlib.util.find_spec("rclpy") is None:
        return ("ROS 2 is not sourced in this shell - "
                "run: source /opt/ros/jazzy/setup.bash")
    return None


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    err = check_ros_env()
    if err:
        # "FAIL env: ..." (not "FAIL 0 env: ...") - a bare phase-0 prefix
        # would collide with the real phase 0 (clean), which has its own
        # documented exit code 10 (CLAUDE.md review finding 5).
        print(f"FAIL env: {err}")
        return 2
    shell = Shell()
    try:
        if args.cmd == "start":
            return cmd_start(shell, args)
        if args.cmd == "stop":
            return cmd_stop(shell)
        return cmd_status(shell)
    except Exception as e:
        # Every phase eventually does something that can raise outside the
        # gates' own error handling (spawn_z's regex over a moved file,
        # Shell.receive, Shell.read via _patterns, ...). Without this, that
        # raises a bare traceback and an undocumented exit code, breaking
        # the "last line is the verdict" contract (CLAUDE.md review
        # finding 3).
        print(f"FAIL {args.cmd}: {e}")
        return 2


if __name__ == "__main__":
    sys.exit(main())
