# Single Sim Entry Point (`scripts/sim.py`) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** One command, `python3 scripts/sim.py start <world>`, that cleans the machine, applies the robot config, launches Gazebo, guarantees active controllers (recovering the spawner race), verifies the robot, bridges extras, and brings up nav2 where a config exists — exiting 0 only when every applicable gate passed.

**Architecture:** `scripts/sim.py` is a phase state machine. Every gate is a **pure function over captured text** (`ps` output, `gz topic` output, service replies, yaml) so it is unit-testable without a sim; a thin `Shell` class does the subprocess/rclpy work and is replaced by a fake in tests. Phases run in order; the first failure ends the run with exit `10 + phase`. `stop` and `status` reuse the same gates read-only.

**Tech Stack:** Python 3.12 stdlib, `pyyaml` (already present via ROS), `rclpy` (only inside `Shell`), pytest. Sourced `/opt/ros/jazzy/setup.bash` for every subprocess.

**Spec:** `docs/superpowers/specs/2026-08-28-single-sim-entrypoint-design.md`

## Global Constraints

- Repo root is `/home/thinhpham/Documents/Husky_viz`; absolute paths into it are the project convention.
- No fixed `sleep` waits — every wait is a deadline-bounded poll of a real signal (spec "Phases and gates"). No `timeout` wrapping a `ros2` CLI (CLAUDE.md #8) — use `subprocess.run(..., timeout=)` on a `bash -lc` child only.
- Source ROS with `set -eo pipefail`, never `-u` (CLAUDE.md #10).
- Never `pkill -f` a pattern that matches our own command line (#9); skip pids whose cmdline contains `bash -c` or our own script path.
- Launch with `setsid nohup ... &` (#22).
- `--config` defaults to `default`; nav2 iff `config/nav2_<world>.yaml` exists and not `--no-nav`.
- Phase 5 never bridges `gps_0` or `imu_enu` (#7, #15).
- Exit codes: 0 READY/CLEAN; `10 + phase` for a gate failure; 2 for usage errors.
- Nothing under `~/clearpath/` is edited; no Clearpath launch file is forked.
- Run tests with `python3 -m pytest tests/test_sim_py.py -q` (rclpy is not imported by the pure functions, so pytest runs without sourcing ROS).
- Commit after every task. Commit messages end with the project trailer:
  ```
  Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01XueZncyUN3mJnbd3AV73fP
  ```

---

## File Structure

| File | Responsibility |
|---|---|
| `scripts/sim.py` | CLI, `Shell` (all side effects), phase functions, pure gate functions, state file. Single file by project convention (`tools/check_*.py` are single files); sections separated by banner comments in the order: constants → pure gates → `Shell` → phases → commands → `main`. |
| `tests/test_sim_py.py` | Pure-gate tests on captured text; phase-ordering tests with `FakeShell`. |
| `tools/check_nav2_ready.py` | Modified: gains `nav_ready(shell_fn) -> list[str]` so `sim.py` imports it instead of shelling out. |
| `CLEAN_SIM.md`, `RUN_SIM.md`, `NAV_PARK.md`, `CLAUDE.md`, `.claude/agents/sim-operator.md`, `.claude/skills/husky-sim/SKILL.md` | Docs/agent pointed at `sim.py`. |

`tests/` imports with `from scripts.sim import ...` — add `scripts/__init__.py` (empty) in Task 1, matching how `tests/test_sdf_geometry.py` imports `tools.sdf_geometry`.

---

### Task 1: Skeleton — result types, output format, exit codes, CLI

**Files:**
- Create: `scripts/__init__.py` (empty), `scripts/sim.py`
- Test: `tests/test_sim_py.py`

**Interfaces:**
- Produces:
  - `@dataclass PhaseResult(phase: int, name: str, status: str, detail: str)` — `status ∈ {"ok","skip","fail"}`
  - `format_line(r: PhaseResult) -> str` → `"[3 controllers] ok  recovered ..."` (phase number, name padded to 11, status padded to 4, detail)
  - `exit_code(results: list[PhaseResult]) -> int` → 0 if no `fail`, else `10 + phase` of the first fail
  - `parse_args(argv: list[str]) -> argparse.Namespace` with subcommands `start|stop|status`
  - `REPO = "/home/thinhpham/Documents/Husky_viz"`, `NS = "/a200_0000"`, `ROBOT_MODEL = "a200_0000/robot"`, `STATE_FILE = Path.home()/".husky_sim"/"state.json"`, `SIM_LOG = "/tmp/sim.log"`, `NAV_LOG = "/tmp/nav.log"`

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_sim_py.py
from scripts.sim import PhaseResult, format_line, exit_code, parse_args


def test_format_line_pads_name_and_status():
    r = PhaseResult(3, "controllers", "ok", "recovered (jsb timed out)")
    assert format_line(r) == "[3 controllers] ok   recovered (jsb timed out)"


def test_exit_code_zero_when_no_fail():
    rs = [PhaseResult(0, "clean", "ok", ""), PhaseResult(6, "nav2", "skip", "")]
    assert exit_code(rs) == 0


def test_exit_code_is_ten_plus_first_failed_phase():
    rs = [PhaseResult(0, "clean", "ok", ""), PhaseResult(2, "launch", "fail", "x"),
          PhaseResult(3, "controllers", "fail", "y")]
    assert exit_code(rs) == 12


def test_parse_args_start_defaults():
    a = parse_args(["start", "park"])
    assert a.cmd == "start" and a.world == "park" and a.config == "default"
    assert a.no_nav is False and a.clean_on_fail is False
    assert a.x is None and a.yaw is None


def test_parse_args_stop_and_status():
    assert parse_args(["stop"]).cmd == "stop"
    assert parse_args(["status"]).cmd == "status"
```

- [ ] **Step 2: Run to verify it fails**

Run: `python3 -m pytest tests/test_sim_py.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'scripts.sim'`

- [ ] **Step 3: Write the skeleton**

```python
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
import sys
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
```

Also `touch scripts/__init__.py` and `chmod +x scripts/sim.py`.

- [ ] **Step 4: Run to verify it passes**

Run: `python3 -m pytest tests/test_sim_py.py -q`
Expected: 5 passed

- [ ] **Step 5: Commit**

```bash
git add scripts/__init__.py scripts/sim.py tests/test_sim_py.py
git commit -m "sim.py: skeleton with phase results, exit codes and CLI"
```

---

### Task 2: Phase 0 — clean (pure gates)

**Files:**
- Modify: `scripts/sim.py` (pure-gates section)
- Test: `tests/test_sim_py.py`

**Interfaces:**
- Produces:
  - `parse_kill_patterns(kill_sim_text: str) -> list[str]` — the quoted entries of the `PATTERNS=( ... )` array in `scripts/kill_sim.sh`, comments ignored
  - `EXTRA_SWEEP = ["a200_0000", "gz sim", "gz_tools_vendor"]`
  - `find_sim_pids(ps_lines: str, patterns: list[str], self_pid: int, self_path: str) -> list[tuple[int, str]]` — `ps_lines` is `ps -eo pid,cmd --no-headers`; returns `(pid, cmd)` whose cmd contains any pattern, excluding `self_pid`, lines containing `bash -c`, and lines containing `self_path`
  - `shm_count(ls_dev_shm: str) -> int` — count of names containing `fastrtps`

- [ ] **Step 1: Write the failing tests**

```python
from scripts.sim import parse_kill_patterns, find_sim_pids, shm_count, EXTRA_SWEEP

KILL_SH = '''
PATTERNS=(
  "gz sim"
  "ros2 launch"
  # Clearpath's teleop stack
  "marker_server"
)
SELF=$$
'''

PS = """\
  101 /opt/ros/jazzy/lib/gz_tools_vendor/bin/gz sim -r park.sdf
  102 bash -c source /opt/ros/jazzy/setup.bash && ros2 launch x
  103 python3 /home/thinhpham/Documents/Husky_viz/scripts/sim.py start park
  104 /usr/bin/python3 /opt/ros/jazzy/lib/nav2_map_server/map_server --ros-args -r __ns:=/a200_0000
  105 /usr/lib/firefox/firefox
"""


def test_parse_kill_patterns_reads_quoted_entries_ignoring_comments():
    assert parse_kill_patterns(KILL_SH) == ["gz sim", "ros2 launch", "marker_server"]


def test_find_sim_pids_matches_patterns_and_namespace_sweep():
    pats = ["gz sim", "ros2 launch"] + EXTRA_SWEEP
    got = find_sim_pids(PS, pats, self_pid=999, self_path="/scripts/sim.py")
    assert [p for p, _ in got] == [101, 104]


def test_find_sim_pids_skips_bash_c_wrapper_and_self():
    pats = ["ros2 launch", "sim.py"]
    got = find_sim_pids(PS, pats, self_pid=103, self_path="/scripts/sim.py")
    assert got == []


def test_shm_count_counts_fastrtps_only():
    assert shm_count("fastrtps_port7412\nsem.fastrtps_port7412\nfoo\n") == 2
    assert shm_count("") == 0
```

- [ ] **Step 2: Run to verify it fails**

Run: `python3 -m pytest tests/test_sim_py.py -q`
Expected: FAIL — `ImportError: cannot import name 'parse_kill_patterns'`

- [ ] **Step 3: Implement**

Add under a `# ---- pure gates` banner after the constants:

```python
import re

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
        pid = int(pid_s)
        if pid == self_pid or "bash -c" in cmd or self_path in cmd:
            continue
        if any(p in cmd for p in patterns):
            found.append((pid, cmd))
    return found


def shm_count(ls_dev_shm: str) -> int:
    return sum(1 for n in ls_dev_shm.split() if "fastrtps" in n)
```

- [ ] **Step 4: Run to verify it passes** — `python3 -m pytest tests/test_sim_py.py -q` → 9 passed

- [ ] **Step 5: Commit** — `git commit -am "sim.py: pure gates for the clean phase"`

---

### Task 3: `Shell` and Phase 0 execution

**Files:**
- Modify: `scripts/sim.py` (Shell + phases sections)
- Test: `tests/test_sim_py.py`

**Interfaces:**
- Produces:
  - `class Shell` with methods (all real side effects):
    - `run(cmd: str, timeout: float = 30) -> str` — runs `bash -lc "set -eo pipefail; source /opt/ros/jazzy/setup.bash; <cmd>"`, returns stdout+stderr, never raises on nonzero (returns output; `.last_rc` holds rc); on `TimeoutExpired` returns partial output and sets `.last_rc = 124`
    - `ps() -> str` — `ps -eo pid,cmd --no-headers`
    - `kill9(pids: list[int]) -> None`
    - `ls_shm() -> str` — `ls /dev/shm`
    - `rm_shm() -> None` — `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*`
    - `daemon_stop() -> None` — `ros2 daemon stop`
    - `read(path: str) -> str`
    - `pid_alive(pid: int) -> bool`
    - `now() -> float` (monotonic), `pause(sec: float)` (the only sleep; used **inside deadline polls only**, 0.5 s)
    - `self_pid: int`, `self_path: str`
  - `poll(shell, deadline_s, probe, interval=0.5) -> Any | None` — calls `probe()` until it returns truthy or the deadline passes; returns the truthy value or `None`
  - `phase_clean(shell) -> PhaseResult` — kill → daemon stop → rm shm → verify (survivors + shm; shm re-read once if nonzero)
  - `class FakeShell` in tests: records calls, returns scripted outputs

- [ ] **Step 1: Write the failing tests**

```python
from scripts.sim import phase_clean, poll


class FakeShell:
    """Scripted side effects. `ps_seq` / `shm_seq` are consumed per call."""
    def __init__(self, ps_seq=("",), shm_seq=("",), files=None, run_out=None):
        self.ps_seq = list(ps_seq); self.shm_seq = list(shm_seq)
        self.files = files or {}; self.run_out = run_out or {}
        self.killed = []; self.calls = []; self.last_rc = 0
        self.self_pid = 1; self.self_path = "/scripts/sim.py"; self.t = 0.0
    def _next(self, seq):
        return seq.pop(0) if len(seq) > 1 else seq[0]
    def ps(self): return self._next(self.ps_seq)
    def ls_shm(self): return self._next(self.shm_seq)
    def kill9(self, pids): self.killed += pids
    def rm_shm(self): self.calls.append("rm_shm")
    def daemon_stop(self): self.calls.append("daemon_stop")
    def read(self, path): return self.files[path]
    def run(self, cmd, timeout=30):
        self.calls.append(cmd)
        for k, v in self.run_out.items():
            if k in cmd:
                return v
        return ""
    def pid_alive(self, pid): return pid in self.run_out.get("alive", ())
    def now(self): return self.t
    def pause(self, s): self.t += s


KILL_FILE = {"/home/thinhpham/Documents/Husky_viz/scripts/kill_sim.sh":
             'PATTERNS=(\n  "gz sim"\n)\n'}


def test_phase_clean_kills_then_reports_ok_when_nothing_remains():
    sh = FakeShell(ps_seq=("  50 gz sim -r park.sdf\n", ""), shm_seq=("fastrtps_1\n", ""),
                   files=KILL_FILE)
    r = phase_clean(sh)
    assert sh.killed == [50]
    assert sh.calls[:2] == ["daemon_stop", "rm_shm"]
    assert r.status == "ok" and r.detail == "killed 1, shm 0"


def test_phase_clean_transient_shm_passes_on_second_read():
    sh = FakeShell(ps_seq=("",), shm_seq=("fastrtps_a fastrtps_b", ""), files=KILL_FILE)
    assert phase_clean(sh).status == "ok"


def test_phase_clean_fails_listing_survivors():
    sh = FakeShell(ps_seq=("  50 gz sim -r park.sdf\n",), files=KILL_FILE)
    r = phase_clean(sh)
    assert r.status == "fail" and "50 gz sim -r park.sdf" in r.detail


def test_phase_clean_fails_on_persistent_shm():
    sh = FakeShell(ps_seq=("",), shm_seq=("fastrtps_a",), files=KILL_FILE)
    r = phase_clean(sh)
    assert r.status == "fail" and "shm 1" in r.detail


def test_poll_returns_first_truthy_before_deadline():
    sh = FakeShell(); seq = iter([None, None, "yes"])
    assert poll(sh, 5.0, lambda: next(seq)) == "yes"


def test_poll_returns_none_at_deadline():
    sh = FakeShell()
    assert poll(sh, 1.0, lambda: None) is None
```

- [ ] **Step 2: Run to verify it fails** — expected `ImportError: cannot import name 'phase_clean'`

- [ ] **Step 3: Implement**

```python
import os, subprocess, time

# ---------------------------------------------------------------- Shell
class Shell:
    def __init__(self):
        self.self_pid = os.getpid()
        self.self_path = os.path.abspath(__file__)
        self.last_rc = 0

    def run(self, cmd: str, timeout: float = 30) -> str:
        full = f"set -eo pipefail; {ROS_SETUP}; {cmd}"
        try:
            p = subprocess.run(["bash", "-lc", full], capture_output=True, text=True,
                               timeout=timeout)
            self.last_rc = p.returncode
            return p.stdout + p.stderr
        except subprocess.TimeoutExpired as e:
            self.last_rc = 124
            return (e.stdout or "") if isinstance(e.stdout, str) else ""

    def ps(self) -> str:
        return subprocess.run(["ps", "-eo", "pid,cmd", "--no-headers"],
                              capture_output=True, text=True).stdout

    def kill9(self, pids: list[int]) -> None:
        for p in pids:
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
    # verify: survivors, bounded by a short deadline for teardown
    survivors = poll(shell, 15.0, lambda: [] if not find_sim_pids(
        shell.ps(), pats, shell.self_pid, shell.self_path) else None)
    left = find_sim_pids(shell.ps(), pats, shell.self_pid, shell.self_path)
    if left:
        lines = "; ".join(f"{p} {c}" for p, c in left[:5])
        return PhaseResult(0, "clean", "fail", f"survivors: {lines}")
    n = shm_count(shell.ls_shm())
    if n:
        n = shm_count(shell.ls_shm())        # transient release: re-read once (CLAUDE.md #12)
    if n:
        return PhaseResult(0, "clean", "fail", f"shm {n} persists after kill (something is alive)")
    return PhaseResult(0, "clean", "ok", f"killed {len(victims)}, shm 0")
```

(`survivors` is intentionally unused beyond forcing the poll; the final `left` is the authoritative read.)

- [ ] **Step 4: Run** — 15 passed
- [ ] **Step 5: Commit** — `git commit -am "sim.py: Shell, poll and the clean phase"`

---

### Task 4: Phase 1 — config

**Files:**
- Modify: `scripts/sim.py`
- Test: `tests/test_sim_py.py`

**Interfaces:**
- Produces:
  - `declared_sensors(robot_yaml_text: str) -> set[str]` — names of sensors under `sensors:` whose `urdf_enabled` or `launch_enabled` is true, as `<type>_<index>` (e.g. `imu_0`, `lidar2d_0`, `gps_0`); type keys map: `camera→camera, lidar2d→lidar2d, lidar3d→lidar3d, imu→imu, gps→gps`
  - `parse_sdf_sensors(apply_output: str) -> set[str]` — names from the `==> sensors in SDF:` block lines `    <name>  ->  <type>`
  - `phase_config(shell, config: str) -> PhaseResult` — runs `apply_config.sh <config>`, requires `declared ⊆ sdf`, **except** `gps_0` when declared with `urdf_enabled: false` (custom xacro supplies it under the same name — it *is* in SDF, so no exception needed; keep the plain subset check)

- [ ] **Step 1: Failing tests**

```python
from scripts.sim import declared_sensors, parse_sdf_sensors, phase_config

ROBOT_YAML = """
sensors:
  camera:
  - model: intel_realsense
    urdf_enabled: true
    launch_enabled: true
  imu:
  - model: microstrain_imu
    urdf_enabled: true
    launch_enabled: true
  gps:
  - model: garmin_18x
    urdf_enabled: false
    launch_enabled: true
  lidar2d:
  - model: hokuyo_ust
    urdf_enabled: false
    launch_enabled: false
"""

APPLY_OUT = """==> applied robot_default.yaml
==> regenerated URDF, params and launch files
==> sensors in SDF:
    imu_0  ->  imu
    gps_0  ->  navsat
    camera_0  ->  rgbd_camera
"""


def test_declared_sensors_indexes_by_type_and_skips_fully_disabled():
    assert declared_sensors(ROBOT_YAML) == {"camera_0", "imu_0", "gps_0"}


def test_parse_sdf_sensors():
    assert parse_sdf_sensors(APPLY_OUT) == {"imu_0", "gps_0", "camera_0"}


def test_phase_config_ok_when_declared_subset_of_sdf():
    sh = FakeShell(files={"/home/thinhpham/Documents/Husky_viz/robot_configs/robot_default.yaml": ROBOT_YAML},
                   run_out={"apply_config.sh default": APPLY_OUT})
    r = phase_config(sh, "default")
    assert r.status == "ok" and "camera_0 gps_0 imu_0" in r.detail


def test_phase_config_fails_naming_missing_sensor():
    sh = FakeShell(files={"/home/thinhpham/Documents/Husky_viz/robot_configs/robot_default.yaml": ROBOT_YAML},
                   run_out={"apply_config.sh default": APPLY_OUT.replace("    camera_0  ->  rgbd_camera\n", "")})
    r = phase_config(sh, "default")
    assert r.status == "fail" and "camera_0" in r.detail


def test_phase_config_fails_on_unknown_config():
    sh = FakeShell(files={})
    r = phase_config(sh, "nope")
    assert r.status == "fail" and "robot_nope.yaml" in r.detail
```

- [ ] **Step 2: Run** — ImportError
- [ ] **Step 3: Implement**

```python
import yaml

SENSOR_TYPES = ("camera", "lidar2d", "lidar3d", "imu", "gps")


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
```

Note `FakeShell.read` raises `KeyError` for a missing file; the real `Shell.read` raises `FileNotFoundError` — both are caught.

- [ ] **Step 4: Run** — 20 passed
- [ ] **Step 5: Commit** — `git commit -am "sim.py: config phase with declared-vs-SDF sensor check"`

---

### Task 5: Phase 2 — launch and "world stepping" gate

**Files:**
- Modify: `scripts/sim.py`
- Test: `tests/test_sim_py.py`

**Interfaces:**
- Produces:
  - `parse_sim_time(stats_msg: str) -> float | None` — from `gz topic -e -t /world/<w>/stats -n 1` output, whose `sim_time { sec: 12 nsec: 345000000 }` block yields `12.345`; `None` if absent
  - `Shell.launch(cmd: str, log: str) -> int` — `setsid nohup bash -lc "<ROS_SETUP>; <cmd>" > log 2>&1 < /dev/null &`, returns the pid of the `bash` (recorded in state); implemented with `subprocess.Popen(..., start_new_session=True)`
  - `Shell.world_stats(world) -> str` — `gz topic -e -t /world/<world>/stats -n 1` with `timeout=5`
  - `pose_args(ns) -> str` — `x:=.. y:=..` only for flags the user set
  - `phase_launch(shell, world, pose: str) -> tuple[PhaseResult, int]` — launches, then polls (90 s) for pid alive **and** two `parse_sim_time` reads with the second strictly greater

- [ ] **Step 1: Failing tests**

```python
from scripts.sim import parse_sim_time, pose_args, phase_launch, parse_args

STATS = "sim_time {\n  sec: 12\n  nsec: 345000000\n}\nreal_time {\n  sec: 30\n}\niterations: 4115\n"


def test_parse_sim_time():
    assert parse_sim_time(STATS) == 12.345
    assert parse_sim_time("") is None


def test_pose_args_only_for_set_flags():
    a = parse_args(["start", "park", "--z", "4.0", "--yaw", "3.05"])
    assert pose_args(a) == "z:=4.0 yaw:=3.05"
    assert pose_args(parse_args(["start", "park"])) == ""


class LaunchShell(FakeShell):
    def __init__(self, stats_seq, alive=True):
        super().__init__(); self.stats_seq = list(stats_seq); self.alive = alive; self.launched = []
    def launch(self, cmd, log): self.launched.append((cmd, log)); return 4242
    def pid_alive(self, pid): return self.alive
    def world_stats(self, world): return self.stats_seq.pop(0) if len(self.stats_seq) > 1 else self.stats_seq[0]


def test_phase_launch_ok_when_sim_time_advances():
    s1 = STATS; s2 = STATS.replace("sec: 12", "sec: 13")
    sh = LaunchShell([ "", s1, s1, s2 ])
    r, pid = phase_launch(sh, "park", "")
    assert r.status == "ok" and pid == 4242 and "stepping" in r.detail
    assert "world:=park" in sh.launched[0][0] and sh.launched[0][1] == "/tmp/sim.log"


def test_phase_launch_fails_when_launch_dies():
    sh = LaunchShell([""], alive=False)
    r, _ = phase_launch(sh, "park", "")
    assert r.status == "fail" and "no longer running" in r.detail


def test_phase_launch_fails_when_sim_time_stalls():
    sh = LaunchShell([STATS])       # same value forever
    sh.t = 0.0
    r, _ = phase_launch(sh, "park", "")
    assert r.status == "fail" and "not stepping" in r.detail
```

- [ ] **Step 2: Run** — ImportError
- [ ] **Step 3: Implement**

```python
LAUNCH_DEADLINE = 90.0


def parse_sim_time(stats_msg: str) -> float | None:
    m = re.search(r"sim_time \{\s*(?:sec: (\d+))?\s*(?:nsec: (\d+))?\s*\}", stats_msg)
    if not m or (m.group(1) is None and m.group(2) is None):
        return None
    return int(m.group(1) or 0) + int(m.group(2) or 0) / 1e9


def pose_args(ns) -> str:
    return " ".join(f"{k}:={getattr(ns, k)}" for k in ("x", "y", "z", "yaw")
                    if getattr(ns, k) is not None)


# in class Shell:
    def launch(self, cmd: str, log: str) -> int:
        with open(log, "ab") as f:
            p = subprocess.Popen(["bash", "-lc", f"{ROS_SETUP}; exec {cmd}"],
                                 stdout=f, stderr=subprocess.STDOUT,
                                 stdin=subprocess.DEVNULL, start_new_session=True)
        return p.pid

    def world_stats(self, world: str) -> str:
        return self.run(f"gz topic -e -t /world/{world}/stats -n 1", timeout=5)


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
```

`start_new_session=True` is the `setsid` of CLAUDE.md #22; `exec` makes the launch replace the bash so `pid` is `ros2 launch` itself.

- [ ] **Step 4: Run** — 26 passed
- [ ] **Step 5: Commit** — `git commit -am "sim.py: launch phase gated on sim_time advancing"`

---

### Task 6: Phase 3 — controllers with spawner recovery

**Files:**
- Modify: `scripts/sim.py`
- Test: `tests/test_sim_py.py`

**Interfaces:**
- Produces:
  - `CONTROLLERS = ["joint_state_broadcaster", "platform_velocity_controller"]`
  - `controller_states(list_output: str) -> dict[str, str]` — parses `ros2 service call .../list_controllers` reply: each `name='X'` … `state='Y'` pair
  - `phase_controllers(shell) -> PhaseResult` — poll 10 s for both active → `ok clean`; else run spawner with `--switch-timeout 30` (timeout 45 s), re-query → `ok recovered (<who> was <state>; respawned in N s)`; else `fail`

- [ ] **Step 1: Failing tests**

```python
from scripts.sim import controller_states, phase_controllers

LIST_OK = ("response:\ncontroller_manager_msgs.srv.ListControllers_Response(controller=["
           "ControllerState(name='joint_state_broadcaster', state='active', type='x'), "
           "ControllerState(name='platform_velocity_controller', state='active', type='y')])\n")
LIST_HALF = LIST_OK.replace("name='platform_velocity_controller', state='active'",
                            "name='platform_velocity_controller', state='inactive'")
LIST_NONE = "response:\ncontroller_manager_msgs.srv.ListControllers_Response(controller=[])\n"


def test_controller_states_parses_name_state_pairs():
    assert controller_states(LIST_HALF) == {"joint_state_broadcaster": "active",
                                            "platform_velocity_controller": "inactive"}
    assert controller_states(LIST_NONE) == {}


def test_phase_controllers_clean():
    sh = FakeShell(run_out={"list_controllers": LIST_OK})
    r = phase_controllers(sh)
    assert r.status == "ok" and r.detail.startswith("clean")
    assert not any("spawner" in c for c in sh.calls)


def test_phase_controllers_recovers_with_switch_timeout_30():
    class Sh(FakeShell):
        def run(self, cmd, timeout=30):
            self.calls.append(cmd)
            if "spawner" in cmd:
                self.run_out["list_controllers"] = LIST_OK
                return "Successfully switched controllers"
            return self.run_out.get("list_controllers", LIST_NONE)
    sh = Sh(run_out={"list_controllers": LIST_HALF})
    r = phase_controllers(sh)
    assert r.status == "ok" and r.detail.startswith("recovered")
    assert "platform_velocity_controller was inactive" in r.detail
    spawn = [c for c in sh.calls if "spawner" in c][0]
    assert "--switch-timeout 30" in spawn and "joint_state_broadcaster platform_velocity_controller" in spawn
    assert f"--controller-manager {NS}/controller_manager" in spawn


def test_phase_controllers_fails_when_recovery_fails():
    sh = FakeShell(run_out={"list_controllers": LIST_NONE, "spawner": "timed out"})
    r = phase_controllers(sh)
    assert r.status == "fail" and "joint_state_broadcaster missing" in r.detail
```

- [ ] **Step 2: Run** — ImportError
- [ ] **Step 3: Implement**

```python
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
```

- [ ] **Step 4: Run** — 30 passed
- [ ] **Step 5: Commit** — `git commit -am "sim.py: controllers phase with spawner-race recovery"`

---

### Task 7: Phase 4 — robot publishing and landed

**Files:**
- Modify: `scripts/sim.py`
- Test: `tests/test_sim_py.py`

**Interfaces:**
- Produces:
  - `ROBOT_TOPICS = {"odom": (f"{NS}/platform/odom", "nav_msgs/msg/Odometry"), "imu": (f"{NS}/sensors/imu_0/data", "sensor_msgs/msg/Imu"), "scan": (f"{NS}/sensors/lidar2d_0/scan", "sensor_msgs/msg/LaserScan"), "points": (f"{NS}/sensors/lidar3d_0/points", "sensor_msgs/msg/PointCloud2")}`
  - `parse_gz_pose(gz_model_output: str) -> tuple[float, float, float] | None` — same regex as `tools/check_rssi_ranging.py:robot_pose`
  - `spawn_z(world: str, override: float | None) -> float | None` — from `launch.park_sim.WORLD_SPAWN_POSES` (import) else `0.3`
  - `Shell.receive(topics: dict, deadline: float) -> dict[str, float]` — one rclpy node, sensor-data QoS subscribers, spin until every topic has ≥1 msg or deadline; returns per-key Hz estimate (msgs / elapsed) — 0.0 for none. rclpy is imported *inside* this method.
  - `Shell.gz_pose() -> str` — `gz model -m a200_0000/robot -p`, timeout 10
  - `phase_robot(shell, world, z_override) -> PhaseResult` — all four topics received, and `|z - spawn_z| <= 0.5`

- [ ] **Step 1: Failing tests**

```python
from scripts.sim import parse_gz_pose, spawn_z, phase_robot

GZ_MODEL = """Requesting state for world [park]...
Model: [a200_0000/robot]
  - Pose [ XYZ (m) ] [ RPY (rad) ]:
    [45.640000 0.021000 3.120000]
    [0.002000 0.001000 2.613200]
"""


def test_parse_gz_pose():
    assert parse_gz_pose(GZ_MODEL) == (45.64, 0.021, 3.12)
    assert parse_gz_pose("no model") is None


def test_spawn_z_from_world_table_and_override():
    assert spawn_z("park", None) == 3.3
    assert spawn_z("warehouse", None) == 0.3
    assert spawn_z("park", 9.0) == 9.0


class RobotShell(FakeShell):
    def __init__(self, rates, pose_out):
        super().__init__(); self.rates = rates; self.pose_out = pose_out
    def receive(self, topics, deadline): return {k: self.rates.get(k, 0.0) for k in topics}
    def gz_pose(self): return self.pose_out


def test_phase_robot_ok():
    sh = RobotShell({"odom": 33.0, "imu": 67.0, "scan": 24.0, "points": 13.0}, GZ_MODEL)
    r = phase_robot(sh, "park", None)
    assert r.status == "ok" and "pose 45.64 0.02 3.12" in r.detail and "odom 33" in r.detail


def test_phase_robot_fails_on_silent_topic():
    sh = RobotShell({"odom": 0.0, "imu": 67.0, "scan": 24.0, "points": 13.0}, GZ_MODEL)
    r = phase_robot(sh, "park", None)
    assert r.status == "fail" and "odom" in r.detail


def test_phase_robot_fails_when_fallen_through_terrain():
    sh = RobotShell({"odom": 33.0, "imu": 67.0, "scan": 24.0, "points": 13.0},
                    GZ_MODEL.replace("3.120000", "-12116.000000"))
    r = phase_robot(sh, "park", None)
    assert r.status == "fail" and "#23" in r.detail
```

- [ ] **Step 2: Run** — ImportError
- [ ] **Step 3: Implement**

```python
ROBOT_TOPICS = {
    "odom":   (f"{NS}/platform/odom",            "nav_msgs/msg/Odometry"),
    "imu":    (f"{NS}/sensors/imu_0/data",       "sensor_msgs/msg/Imu"),
    "scan":   (f"{NS}/sensors/lidar2d_0/scan",   "sensor_msgs/msg/LaserScan"),
    "points": (f"{NS}/sensors/lidar3d_0/points", "sensor_msgs/msg/PointCloud2"),
}
ROBOT_DEADLINE = 10.0
LANDED_TOL = 0.5


def parse_gz_pose(gz_model_output: str):
    m = re.search(r"Pose[^\n]*\n\s*\[([^\]]+)\]", gz_model_output)
    if not m:
        return None
    try:
        v = tuple(float(x) for x in m.group(1).split())
    except ValueError:
        return None
    return v if len(v) == 3 else None


def spawn_z(world: str, override):
    if override is not None:
        return float(override)
    sys.path.insert(0, REPO)
    from launch.park_sim import WORLD_SPAWN_POSES, STOCK_POSE_DEFAULTS   # noqa: E402
    return float(WORLD_SPAWN_POSES.get(world, STOCK_POSE_DEFAULTS)["z"])


# in class Shell:
    def receive(self, topics: dict, deadline: float) -> dict[str, float]:
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
        elapsed = max(time.monotonic() - t0, 1e-3)
        node.destroy_node()
        rclpy.shutdown()
        return {k: c / elapsed for k, c in counts.items()}

    def gz_pose(self) -> str:
        return self.run(f"gz model -m {ROBOT_MODEL} -p", timeout=10)


def phase_robot(shell, world: str, z_override) -> PhaseResult:
    rates = shell.receive(ROBOT_TOPICS, ROBOT_DEADLINE)
    silent = [k for k, hz in rates.items() if hz <= 0.0]
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
    hz = " ".join(f"{k} {rates[k]:.0f} Hz" for k in ROBOT_TOPICS)
    return PhaseResult(4, "robot", "ok", f"pose {pose[0]:.2f} {pose[1]:.2f} {pose[2]:.2f}  {hz}")
```

`spawn_z` imports `launch/park_sim.py`, which imports `launch` ROS modules — importing it requires ROS sourced. The unit test therefore must run under `bash -lc "source /opt/ros/jazzy/setup.bash && python3 -m pytest ..."`; **update the Global Constraints test command to that form from this task on.** (The `launch/` directory name shadows the ROS `launch` package when `REPO` is on `sys.path` first — use `sys.path.append(REPO)` instead of `insert(0, ...)` and import as `importlib.import_module("launch.park_sim")`; if that still collides, read `WORLD_SPAWN_POSES` by regex from the file text: `re.search(r"'%s': \{[^}]*'z': '([\d.\-]+)'" % world, text)`. Prefer the regex — it needs no ROS at test time. Implement the regex form.)

Final `spawn_z` (regex form, replaces the import form above):

```python
def spawn_z(world: str, override):
    if override is not None:
        return float(override)
    text = Path(f"{REPO}/launch/park_sim.launch.py").read_text()
    m = re.search(r"'%s': \{[^}]*'z': '([\d.\-]+)'" % re.escape(world), text)
    return float(m.group(1)) if m else 0.3
```

- [ ] **Step 4: Run** — 36 passed
- [ ] **Step 5: Commit** — `git commit -am "sim.py: robot phase - topics received and robot landed"`

---

### Task 8: Phase 5 — extras bridge

**Files:**
- Modify: `scripts/sim.py`
- Test: `tests/test_sim_py.py`

**Interfaces:**
- Produces:
  - `extras_features(robot_yaml_text: str, read) -> set[str]` — reads `platform.extras.urdf.path`, then `read(path)`; returns `{"compass"}` if that xacro text contains `compass.urdf.xacro`, `{"radio"}` if it contains `comms.urdf.xacro`; empty if no extras
  - `bridge_args(features) -> list[str]` — compass → `f"{NS}/sensors/compass_0/mag@sensor_msgs/msg/MagneticField[gz.msgs.Magnetometer"`; radio → the three Dataframe entries from `run_husky_sim.sh` (`/broker/msgs@...]gz.msgs.Dataframe`, `/husky/rx@...[`, `/base_station/rx@...[`)
  - `phase_extras(shell, config, launch_pid) -> tuple[PhaseResult, int | None]` — `skip` when no features; else launch `ros2 run ros_gz_bridge parameter_bridge <args> --ros-args -r __node:=extras_gz_bridge` to `/tmp/bridge.log`, poll 5 s that bridge pid and launch pid are both alive

- [ ] **Step 1: Failing tests**

```python
from scripts.sim import extras_features, bridge_args, phase_extras

FULL_YAML = "platform:\n  extras:\n    urdf:\n      path: /x/extras.urdf.xacro\n"
FILES = {"/x/extras.urdf.xacro": '<xacro:include filename="/y/compass.urdf.xacro"/>\n<xacro:include filename="/y/comms.urdf.xacro"/>',
         "/x/extras_radio.urdf.xacro": '<xacro:include filename="/y/comms.urdf.xacro"/>'}


def test_extras_features_full_and_radio_and_none():
    assert extras_features(FULL_YAML, FILES.__getitem__) == {"compass", "radio"}
    assert extras_features(FULL_YAML.replace("extras.urdf", "extras_radio.urdf"), FILES.__getitem__) == {"radio"}
    assert extras_features("platform: {}\n", FILES.__getitem__) == set()


def test_bridge_args_never_include_gps_or_imu_enu():
    a = bridge_args({"compass", "radio"})
    assert any("compass_0/mag" in x for x in a) and any("/husky/rx" in x for x in a)
    assert not any("gps" in x or "imu_enu" in x for x in a)
    assert bridge_args(set()) == []


def test_phase_extras_skips_for_default():
    sh = FakeShell(files={f"{REPO}/robot_configs/robot_default.yaml": "platform: {}\n"})
    r, pid = phase_extras(sh, "default", 4242)
    assert r.status == "skip" and pid is None


def test_phase_extras_bridges_for_full():
    class Sh(FakeShell):
        def launch(self, cmd, log): self.calls.append(cmd); return 777
        def pid_alive(self, pid): return True
    sh = Sh(files={f"{REPO}/robot_configs/robot_full.yaml": FULL_YAML, **FILES})
    r, pid = phase_extras(sh, "full", 4242)
    assert r.status == "ok" and pid == 777
    assert "parameter_bridge" in sh.calls[-1] and "extras_gz_bridge" in sh.calls[-1]
```

(`REPO` must be imported in the test module: `from scripts.sim import REPO`.)

- [ ] **Step 2: Run** — ImportError
- [ ] **Step 3: Implement**

```python
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
```

- [ ] **Step 4: Run** — 40 passed
- [ ] **Step 5: Commit** — `git commit -am "sim.py: extras bridge phase for compass/radio configs"`

---

### Task 9: Phase 6 — nav2, reusing `check_nav2_ready.py`

**Files:**
- Modify: `tools/check_nav2_ready.py:100-157` (extract `main`'s checks into `nav_ready(sh) -> list[str]`), `scripts/sim.py`
- Test: `tests/test_sim_py.py`

**Interfaces:**
- Produces:
  - `tools.check_nav2_ready.nav_ready(sh, tf_check=check_transform) -> list[str]` — the failure list `main()` builds today, with `sh` injectable (same signature as its `sh(cmd, timeout)`) and the tf check injectable; prints unchanged; `main()` becomes `return 1 if nav_ready(sh) else 0` plus the READY/NOT READY print
  - `nav_config(world) -> str | None` — `f"{REPO}/config/nav2_{world}.yaml"` if it exists
  - `phase_nav2(shell, world, no_nav) -> tuple[PhaseResult, int | None]` — `skip` if no config or `--no-nav`; else launch `nav_park.launch.py` to `/tmp/nav.log`; poll 60 s until `nav_ready(shell.run, tf_check)` returns `[]`

- [ ] **Step 1: Failing tests**

```python
from scripts.sim import nav_config, phase_nav2


def test_nav_config_only_where_file_exists():
    assert nav_config("park") == f"{REPO}/config/nav2_park.yaml"
    assert nav_config("lake") is None


class NavShell(FakeShell):
    def __init__(self, ready_seq):
        super().__init__(); self.ready_seq = list(ready_seq); self.launched = []
    def launch(self, cmd, log): self.launched.append((cmd, log)); return 900
    def pid_alive(self, pid): return True
    def nav_ready(self): return self.ready_seq.pop(0) if len(self.ready_seq) > 1 else self.ready_seq[0]


def test_phase_nav2_skips_without_config_or_with_flag():
    assert phase_nav2(NavShell([[]]), "lake", False)[0].status == "skip"
    assert phase_nav2(NavShell([[]]), "park", True)[0].status == "skip"


def test_phase_nav2_ok_when_failures_drain():
    sh = NavShell([["map -> odom not published"], []])
    r, pid = phase_nav2(sh, "park", False)
    assert r.status == "ok" and pid == 900 and sh.launched[0][1] == "/tmp/nav.log"
    assert "nav_park.launch.py" in sh.launched[0][0]


def test_phase_nav2_fails_reporting_last_failures():
    sh = NavShell([["planner_server is not active"]])
    r, _ = phase_nav2(sh, "park", False)
    assert r.status == "fail" and "planner_server" in r.detail
```

- [ ] **Step 2: Run** — ImportError
- [ ] **Step 3: Refactor `check_nav2_ready.py`**

Replace `main()` (lines 100–157) with:

```python
def nav_ready(sh=sh, tf_check=check_transform) -> list[str]:
    """Run every readiness check; return the list of failures (empty == READY)."""
    failures = []
    print("== transforms")
    if not tf_check():
        failures.append("map -> odom not published (GPS localization not up?)")
    print("== lifecycle nodes")
    for n in LIFECYCLE:
        out = sh(f"ros2 service call {NS}/{n}/get_state lifecycle_msgs/srv/GetState "
                 f"'{{}}' 2>/dev/null", timeout=5.0)
        good = "label='active'" in out
        print(f"  {n:30s}: {'active' if good else 'NOT ACTIVE'}")
        if not good:
            reason = out.strip()[:80] or "no response (service unavailable within 5s)"
            failures.append(f"{n} is not active (get_state: {reason})")
    print("== action servers")
    acts = sh("ros2 action list 2>/dev/null")
    for a in ("navigate_to_pose", "follow_waypoints"):
        ok = f"{NS}/{a}" in acts
        print(f"  {a:30s}: {'OK' if ok else 'MISSING'}")
        if not ok:
            failures.append(f"action {a} not advertised")
    print("== cmd_vel contract (gotcha #3)")
    info = sh(f"ros2 topic info -v {NS}/cmd_vel 2>/dev/null")
    stamped = "geometry_msgs/msg/TwistStamped" in info
    plain = "geometry_msgs/msg/Twist\n" in info or "geometry_msgs/msg/Twist " in info
    print(f"  TwistStamped : {'yes' if stamped else 'NO'}")
    if plain and not stamped:
        print("  !! nav2 publishes plain Twist; the Husky will never move.")
        failures.append("cmd_vel type mismatch: nav2 publishes Twist, robot needs TwistStamped")
    elif not stamped:
        failures.append("cmd_vel has no TwistStamped publisher")
    print("== costmaps")
    for t in (f"{NS}/global_costmap/costmap", f"{NS}/local_costmap/costmap"):
        n = sh(f"ros2 topic info {t} 2>/dev/null | grep -c 'Publisher count: 1'").strip()
        ok = n == "1"
        print(f"  {t.split('/')[-2]:30s}: {'OK' if ok else 'no publisher'}")
        if not ok:
            failures.append(f"{t} has no publisher")
    return failures


def main() -> int:
    failures = nav_ready()
    if failures:
        print("\nNOT READY:")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("\nREADY")
    return 0
```

Behaviour of `python3 tools/check_nav2_ready.py` is unchanged; verify by running it against no sim: it must print `NOT READY` with the same bullets as before the edit.

Then in `sim.py`:

```python
import contextlib, io

NAV_DEADLINE = 60.0


def nav_config(world: str):
    p = f"{REPO}/config/nav2_{world}.yaml"
    return p if os.path.exists(p) else None


# in class Shell:
    def nav_ready(self) -> list[str]:
        sys.path.append(REPO)
        from tools.check_nav2_ready import nav_ready
        with contextlib.redirect_stdout(io.StringIO()):     # its per-check prints are noise here
            return nav_ready(lambda cmd, timeout=None: self.run(cmd, timeout=timeout or 30))


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
```

- [ ] **Step 4: Run** — 45 passed; also `bash -lc "source /opt/ros/jazzy/setup.bash && python3 tools/check_nav2_ready.py"` with no sim → exits 1, prints `NOT READY`
- [ ] **Step 5: Commit** — `git commit -am "sim.py: nav2 phase; check_nav2_ready exposes nav_ready()"`

---

### Task 10: `start` orchestration, state file, `stop`, `status`

**Files:**
- Modify: `scripts/sim.py` (commands + main)
- Test: `tests/test_sim_py.py`

**Interfaces:**
- Produces:
  - `save_state(path, d: dict)`, `load_state(path) -> dict | None`
  - `cmd_start(shell, args, out=print) -> int` — runs phases 0–6 in order, printing each line as it completes; stops at first `fail`; on fail with `--clean-on-fail` runs `phase_clean` afterwards; writes state `{world, config, launch_pid, bridge_pid, nav_pid, started_at, phase_reached}` after phase 2 and again at the end; prints `READY <world> <config> [nav]` or `FAIL <phase>: <detail>`; returns `exit_code`
  - `cmd_stop(shell, out=print) -> int` — `phase_clean`, delete state file, prints `CLEAN` or `FAIL 0: ...`
  - `cmd_status(shell, out=print) -> int` — loads state (prints `no state file` + still probes); runs gates 2–6 read-only: launch pid alive + stepping, `controller_states` (no spawner), `phase_robot`, bridge pid alive if any, `nav_ready` if nav pid; prints lines + `READY`/`NOT READY`

- [ ] **Step 1: Failing tests**

```python
import json
from scripts.sim import cmd_start, cmd_stop, cmd_status, save_state, load_state


def _patched(monkeypatch, results):
    """Replace every phase with a stub returning the scripted PhaseResult."""
    import scripts.sim as S
    monkeypatch.setattr(S, "phase_clean", lambda sh: results[0])
    monkeypatch.setattr(S, "phase_config", lambda sh, c: results[1])
    monkeypatch.setattr(S, "phase_launch", lambda sh, w, p: (results[2], 4242))
    monkeypatch.setattr(S, "phase_controllers", lambda sh: results[3])
    monkeypatch.setattr(S, "phase_robot", lambda sh, w, z: results[4])
    monkeypatch.setattr(S, "phase_extras", lambda sh, c, lp: (results[5], None))
    monkeypatch.setattr(S, "phase_nav2", lambda sh, w, n: (results[6], 900))


OK7 = [PhaseResult(i, PHASE_NAMES[i], "ok", "d") for i in range(7)]


def test_cmd_start_prints_every_phase_and_ready(monkeypatch, tmp_path):
    import scripts.sim as S
    monkeypatch.setattr(S, "STATE_FILE", tmp_path / "state.json")
    _patched(monkeypatch, OK7)
    lines = []
    rc = cmd_start(FakeShell(), parse_args(["start", "park"]), out=lines.append)
    assert rc == 0 and lines[-1] == "READY park default nav"
    assert lines[0].startswith("[0 clean") and lines[6].startswith("[6 nav2")
    st = json.loads((tmp_path / "state.json").read_text())
    assert st["world"] == "park" and st["launch_pid"] == 4242 and st["phase_reached"] == 6


def test_cmd_start_stops_at_first_fail_and_leaves_sim_running(monkeypatch, tmp_path):
    import scripts.sim as S
    monkeypatch.setattr(S, "STATE_FILE", tmp_path / "state.json")
    rs = list(OK7); rs[3] = PhaseResult(3, "controllers", "fail", "boom")
    _patched(monkeypatch, rs)
    cleaned = []
    monkeypatch.setattr(S, "phase_clean", lambda sh: cleaned.append(1) or OK7[0])
    lines = []
    rc = cmd_start(FakeShell(), parse_args(["start", "park"]), out=lines.append)
    assert rc == 13 and lines[-1] == "FAIL 3 controllers: boom"
    assert len(lines) == 5 and cleaned == [1]          # phase 0 only, no clean-on-fail


def test_cmd_start_clean_on_fail(monkeypatch, tmp_path):
    import scripts.sim as S
    monkeypatch.setattr(S, "STATE_FILE", tmp_path / "state.json")
    rs = list(OK7); rs[2] = PhaseResult(2, "launch", "fail", "dead")
    _patched(monkeypatch, rs)
    cleaned = []
    monkeypatch.setattr(S, "phase_clean", lambda sh: cleaned.append(1) or OK7[0])
    rc = cmd_start(FakeShell(), parse_args(["start", "park", "--clean-on-fail"]), out=lambda s: None)
    assert rc == 12 and cleaned == [1, 1]


def test_cmd_start_ready_line_omits_nav_when_skipped(monkeypatch, tmp_path):
    import scripts.sim as S
    monkeypatch.setattr(S, "STATE_FILE", tmp_path / "state.json")
    rs = list(OK7); rs[6] = PhaseResult(6, "nav2", "skip", "no config")
    _patched(monkeypatch, rs)
    lines = []
    cmd_start(FakeShell(), parse_args(["start", "lake"]), out=lines.append)
    assert lines[-1] == "READY lake default"


def test_cmd_stop_prints_clean_and_removes_state(monkeypatch, tmp_path):
    import scripts.sim as S
    p = tmp_path / "state.json"; p.write_text("{}")
    monkeypatch.setattr(S, "STATE_FILE", p)
    monkeypatch.setattr(S, "phase_clean", lambda sh: OK7[0])
    lines = []
    assert cmd_stop(FakeShell(), out=lines.append) == 0
    assert lines[-1] == "CLEAN" and not p.exists()


def test_save_and_load_state_roundtrip(tmp_path):
    p = tmp_path / "s.json"
    save_state(p, {"world": "park"})
    assert load_state(p) == {"world": "park"}
    assert load_state(tmp_path / "missing.json") is None
```

- [ ] **Step 2: Run** — ImportError
- [ ] **Step 3: Implement**

```python
import json


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


def main(argv=None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    shell = Shell()
    if args.cmd == "start":
        return cmd_start(shell, args)
    if args.cmd == "stop":
        return cmd_stop(shell)
    return cmd_status(shell)
```

Remove the Task 1 placeholder `main`.

- [ ] **Step 4: Run** — 51 passed
- [ ] **Step 5: Commit** — `git commit -am "sim.py: start/stop/status orchestration and state file"`

---

### Task 11: Runbooks, CLAUDE.md, agent and skill point at `sim.py`

**Files:**
- Modify: `CLEAN_SIM.md`, `RUN_SIM.md`, `NAV_PARK.md:16-33` (Steps 1–4), `CLAUDE.md` (Workflow section + gotcha #27), `.claude/agents/sim-operator.md:13-21,24-30`, `.claude/skills/husky-sim/SKILL.md`

- [ ] **Step 1: Rewrite `RUN_SIM.md`** to exactly:

````markdown
# Running the simulation

One command. Rationale lives in `CLAUDE.md`; design in
`docs/superpowers/specs/2026-08-28-single-sim-entrypoint-design.md`.

Worlds: `park`, `lake`, `warehouse_ext`, `warehouse_ramp`, and Clearpath's
six stock worlds (`warehouse`, `construction`, `office`, `orchard`,
`pipeline`, `solar_farm`).

## Step 1 — Start

```bash
cd ~/Documents/Husky_viz
python3 scripts/sim.py start <world>            # config: default
python3 scripts/sim.py start <world> --config full
```

It cleans first (no separate `CLEAN_SIM.md` pass is needed), applies the
config, launches, ensures both controllers are active, verifies the robot,
bridges compass/radio when the config has them, and brings up nav2 when
`config/nav2_<world>.yaml` exists (`--no-nav` to skip). Pose overrides:
`--x --y --z --yaw`.

## Step 2 — Read the output

Expected shape (park):

```
[0 clean      ] ok   killed 0, shm 0
[1 config     ] ok   default  (sensors: gps_0 imu_0 lidar2d_0 lidar3d_0)
[2 launch     ] ok   pid 41233, park stepping after 6.8 s
[3 controllers] ok   clean            <- or: recovered (...)
[4 robot      ] ok   pose 45.64 0.02 3.12  odom 33 Hz imu 67 Hz scan 24 Hz points 13 Hz
[5 extras     ] skip default config has no compass/radio
[6 nav2       ] ok   map->odom present, all lifecycle nodes active
READY park default nav
```

The last line is the verdict. `READY` means every gate passed.
`FAIL <n> <phase>: <observation>` means it stopped there; the sim is left
running for inspection (`--clean-on-fail` to tear it down instead).

| Exit | Meaning |
|---|---|
| 0 | READY |
| 10–16 | phase 0–6 failed (10 clean, 11 config, 12 launch, 13 controllers, 14 robot, 15 extras, 16 nav2) |
| 2 | usage |

Logs: `/tmp/sim.log`, `/tmp/bridge.log`, `/tmp/nav.log`.

## Step 3 — Later checks

```bash
python3 scripts/sim.py status     # re-runs the gates read-only
python3 scripts/sim.py stop       # must print CLEAN
```

`scripts/run_husky_sim.sh` remains as a manual entry point; do not run it
alongside `sim.py` (two bridges on the same topics).
````

- [ ] **Step 2: Rewrite `CLEAN_SIM.md`** to:

````markdown
# Cleaning up before a new simulation

```bash
cd ~/Documents/Husky_viz
python3 scripts/sim.py stop
```

**Required: last line `CLEAN`.** `sim.py start` runs this itself, so a
separate clean pass is only needed to stop a sim without starting another.

## What it does

Kills every pid matching `scripts/kill_sim.sh`'s pattern list plus the
`a200_0000` / `gz sim` / `gz_tools_vendor` sweep (skipping `bash -c`
wrappers and itself), stops the `ros2` daemon, removes
`/dev/shm/fastrtps_*` and `sem.fastrtps_*`, then verifies no survivors and
a `fastrtps` count of 0 — re-reading once because a nonzero first read is
usually a transient release (CLAUDE.md #12).

## If it prints FAIL

It lists the survivors by full command line. A survivor means a node type
the pattern list does not cover — add it to `scripts/kill_sim.sh`'s
`PATTERNS` (the user decides), never kill it by hand and move on.
````

- [ ] **Step 3: Edit `NAV_PARK.md`** — replace Steps 1–4 with a single `## Step 1 - start park` whose body is `python3 scripts/sim.py start park` and "must end with `READY park default nav`"; renumber the remaining steps 2–7. Keep their content verbatim.

- [ ] **Step 4: Edit `.claude/agents/sim-operator.md`** — replace the "Load your skills first" list with the single `husky-sim` skill; replace the runbook table rows for `CLEAN_SIM.md`/`RUN_SIM.md` with "both are now one command, `python3 scripts/sim.py start|stop|status`"; add: "Run the command once, relay every phase line and the verdict verbatim, and stop. On `FAIL`, report the line and the named log's last 30 lines; do not re-run, do not investigate." Fix the "completed through Step 5" reference in the Demos section to "printed READY".

- [ ] **Step 5: Edit `.claude/skills/husky-sim/SKILL.md`** — wherever it enumerates CLEAN_SIM/RUN_SIM steps, replace with the `sim.py` command and the READY/CLEAN verdict rule; keep NAV_PARK/DEMO guidance.

- [ ] **Step 6: Edit `CLAUDE.md`** — in Workflow, under the runbook table, add: "`scripts/sim.py start|stop|status` is the single entry point (2026-08-28); `CLEAN_SIM.md` and `RUN_SIM.md` now wrap it." In gotcha #27, append: "**Handled by `scripts/sim.py` Phase 3** — it queries `list_controllers` once the world is stepping and re-runs the spawner with `--switch-timeout 30` only when a controller is not active, logging `clean` or `recovered`." In the Workflow "Never wait on a sim launch with a fixed sleep" paragraph, add one sentence: "`sim.py` polls a real signal (sim_time advancing, a received message, a service reply) under a per-phase deadline — that is the sanctioned form."

- [ ] **Step 7: Verify docs** — `grep -rn "husky-sim-restart\|husky-run-sim" .claude CLAUDE.md *.md` → no hits. `grep -n "Step 5" .claude/agents/sim-operator.md` → no hits.

- [ ] **Step 8: Commit** — `git add -A CLEAN_SIM.md RUN_SIM.md NAV_PARK.md CLAUDE.md .claude && git commit -m "Point runbooks, CLAUDE.md and sim-operator at scripts/sim.py"`

---

### Task 12: Live acceptance (executed by the `sim-operator` agent, not in unit tests)

**Files:** none modified unless a run exposes a bug (then fix under the relevant task and re-run).

- [ ] **Step 1: Five park starts.** For i in 1..5: `python3 scripts/sim.py start park` → record the verdict and the Phase 3 word (`clean`/`recovered`); `python3 scripts/sim.py stop` → must print `CLEAN`. Acceptance: 5/5 `READY park default nav`.
- [ ] **Step 2: Lake.** `start lake` → `READY lake default` with `[6 nav2] skip no config/nav2_lake.yaml`; `stop` → `CLEAN`.
- [ ] **Step 3: Full config on warehouse_ext.** `start warehouse_ext --config full` → Phase 5 `ok bridged compass radio`; confirm `ros2 topic info -v /a200_0000/sensors/compass_0/mag` shows `Publisher count: 1`; `stop`.
- [ ] **Step 4: Negative.** `start park`, then `kill -9 $(pgrep -f ros2_control_node | head -1)` (controller_manager), then `python3 scripts/sim.py status` → `[3 controllers] fail ...` and `NOT READY`; `stop`.
- [ ] **Step 5: Independent clean check.** After the last `stop`, run CLEAN_SIM's former manual check: `ps -eo cmd --no-headers | grep -c '^/opt/ros'` → `0`; `ls /dev/shm | grep -c fastrtps` → `0`.
- [ ] **Step 6: Record results** in the PR/commit message: the 5-run tally (`clean` vs `recovered` count) and each step's verdict line, then `git commit --allow-empty -m "sim.py: live acceptance — <tally>"`.

---

## Self-review

- **Spec coverage:** CLI (T1, T10) · phases 0–6 with their gates and deadlines (T3–T9) · exit codes (T1) · state file / status / stop (T10) · extras never bridging gps/imu_enu (T8 test) · nav2 iff config exists / `--no-nav` (T9) · spawner recovery option (b) with `clean`/`recovered` (T6) · docs & agent updates (T11) · integration list (T12). No gaps.
- **Placeholders:** none; every step carries code or an exact command.
- **Type consistency:** `phase_launch/phase_extras/phase_nav2` return `(PhaseResult, pid)`; the others return `PhaseResult`; `cmd_start` unpacks accordingly. `FakeShell` is defined once in T3 and extended by subclass in T5/T7/T9. `_query_controllers`/`_describe` (T6) are reused by `cmd_status` (T10). `spawn_z` final form is the regex version (T7 note).
