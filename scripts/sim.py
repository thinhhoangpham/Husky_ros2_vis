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


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    print(f"sim.py: {args.cmd} not implemented yet", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
