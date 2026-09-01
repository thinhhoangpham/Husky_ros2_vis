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


def test_find_sim_pids_skips_malformed_ps_line():
    # ruling 2: a non-numeric first token must not crash parsing
    lines = "garbage line with no pid\n  50 gz sim -r park.sdf\n"
    found = find_sim_pids(lines, ["gz sim"], self_pid=1, self_path="/scripts/sim.py")
    assert found == [(50, "gz sim -r park.sdf")]


from scripts.sim import gz_world_name


def test_gz_world_name_resolves_mismatched_sdf_name(tmp_path):
    (tmp_path / "warehouse_ext.sdf").write_text("<sdf><world name='warehouse'></world></sdf>")
    assert gz_world_name("warehouse_ext", worlds_dir=str(tmp_path)) == "warehouse"


def test_gz_world_name_matches_basename(tmp_path):
    (tmp_path / "park.sdf").write_text("<sdf><world name='park'></world></sdf>")
    assert gz_world_name("park", worlds_dir=str(tmp_path)) == "park"


def test_gz_world_name_falls_back_to_basename_when_file_missing(tmp_path):
    assert gz_world_name("nope", worlds_dir=str(tmp_path)) == "nope"


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


from scripts.sim import parse_sim_time, pose_args, phase_launch

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


from scripts.sim import controller_states, phase_controllers, NS

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
    assert r.status == "ok" and "pose 45.64 0.02 3.12" in r.detail and "4/4 topics receiving" in r.detail


def test_phase_robot_fails_on_silent_topic():
    sh = RobotShell({"odom": 0.0, "imu": 67.0, "scan": 24.0, "points": 13.0}, GZ_MODEL)
    r = phase_robot(sh, "park", None)
    assert r.status == "fail" and "odom" in r.detail


def test_phase_robot_fails_when_fallen_through_terrain():
    sh = RobotShell({"odom": 33.0, "imu": 67.0, "scan": 24.0, "points": 13.0},
                    GZ_MODEL.replace("3.120000", "-12116.000000"))
    r = phase_robot(sh, "park", None)
    assert r.status == "fail" and "#23" in r.detail


def test_phase_robot_ok_detail_reports_counts_not_rates():
    sh = RobotShell({"odom": 1, "imu": 40, "scan": 3, "points": 1}, GZ_MODEL)
    r = phase_robot(sh, "park", None)
    assert r.status == "ok" and "4/4 topics receiving" in r.detail and "Hz" not in r.detail


from scripts.sim import extras_features, bridge_args, phase_extras, REPO

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


def test_shell_launch_truncates_log(tmp_path, monkeypatch):
    import os as _os
    import time as _t
    import scripts.sim as sim
    monkeypatch.setattr(sim, "ROS_SETUP", ":")
    log = tmp_path / "log.txt"
    log.write_text("STALE MARKER FROM PREVIOUS RUN\n")
    sh = sim.Shell()
    pid = sh.launch("true", str(log))
    end = _t.monotonic() + 5
    alive = True
    while alive and _t.monotonic() < end:
        try:
            reaped_pid, _ = _os.waitpid(pid, _os.WNOHANG)
            alive = reaped_pid == 0  # 0 means still running; nonzero means reaped
        except ChildProcessError:
            alive = False
        if alive:
            _t.sleep(0.02)
    assert "STALE MARKER" not in log.read_text()


from scripts.sim import nav_config, phase_nav2


def test_nav_config_only_where_file_exists():
    assert nav_config("park") == f"{REPO}/config/nav2_park.yaml"
    assert nav_config("lake") is None


class NavShell(FakeShell):
    def __init__(self, ready_seq, stats=""):
        super().__init__(); self.ready_seq = list(ready_seq); self.launched = []
        self.stats = stats
    def launch(self, cmd, log): self.launched.append((cmd, log)); return 900
    def pid_alive(self, pid): return True
    def nav_ready(self): return self.ready_seq.pop(0) if len(self.ready_seq) > 1 else self.ready_seq[0]
    def world_stats(self, world): return self.stats


def test_phase_nav2_skips_without_config_or_with_flag():
    assert phase_nav2(NavShell([[]]), "lake", False)[0].status == "skip"
    assert phase_nav2(NavShell([[]]), "park", True)[0].status == "skip"


def test_phase_nav2_ok_when_failures_drain():
    sh = NavShell([["map -> odom not published"], []])
    r, pid = phase_nav2(sh, "park", False)
    assert r.status == "ok" and pid == 900 and sh.launched[0][1] == "/tmp/nav.log"
    assert "park_stock.launch.py" in sh.launched[0][0]
    # the seam: the navigation stages only, no second Gazebo (see phase_nav2).
    assert "world_and_robot:=false" in sh.launched[0][0]


def test_phase_nav2_fails_reporting_last_failures():
    sh = NavShell([["planner_server is not active"]])
    r, _ = phase_nav2(sh, "park", False)
    assert r.status == "fail" and "planner_server" in r.detail


def test_phase_nav2_retries_across_several_slow_passes():
    """Defect 1 regression: a fake whose nav_ready() reports failures for the
    first several polls (as a get_state timeout would under load) and then
    succeeds must still yield 'ok' - proving phase_nav2 retries within
    NAV_DEADLINE rather than giving up after a single starved pass."""
    sh = NavShell([["velocity_smoother is not active (get_state timeout)"]] * 5 + [[]])
    r, pid = phase_nav2(sh, "park", False)
    assert r.status == "ok" and pid == 900
    # the fake clock only advances via shell.pause(interval) between polls,
    # so reaching "ok" here proves multiple probe() calls actually ran.
    assert sh.t > 0


from scripts.sim import parse_rtf, nav_deadline, NAV_DEADLINE, NAV_DEADLINE_MAX, RTF_FLOOR

RTF_STATS_FULL = "real_time_factor: 0.12\nsim_time {\n  sec: 12\n}\n"
RTF_STATS_HEALTHY = "real_time_factor: 1.0\nsim_time {\n  sec: 12\n}\n"


def test_parse_rtf():
    assert parse_rtf(RTF_STATS_FULL) == 0.12
    assert parse_rtf(RTF_STATS_HEALTHY) == 1.0
    assert parse_rtf("") is None


def test_nav_deadline_unscaled_at_full_rtf():
    assert nav_deadline(1.0) == NAV_DEADLINE


def test_nav_deadline_scaled_but_capped_at_slow_rtf():
    d = nav_deadline(0.12)
    assert d > NAV_DEADLINE
    assert d <= NAV_DEADLINE_MAX


def test_nav_deadline_falls_back_when_rtf_unreadable():
    assert nav_deadline(None) == NAV_DEADLINE


def test_phase_nav2_scales_deadline_and_names_rtf_in_fail_detail():
    """A 'full'-config-speed sim (rtf 0.12) that never becomes ready must be
    given the scaled budget, not the raw 180 s, and the FAIL detail must
    name both the measured rtf and the effective deadline used."""
    sh = NavShell([["planner_server is not active"]], stats=RTF_STATS_FULL)
    r, _ = phase_nav2(sh, "park", False)
    assert r.status == "fail"
    assert "rtf 0.12" in r.detail
    assert f"{nav_deadline(0.12):.0f} s" in r.detail
    assert "planner_server" in r.detail


def test_phase_nav2_falls_back_to_raw_deadline_when_rtf_unreadable():
    sh = NavShell([["planner_server is not active"]], stats="")
    r, _ = phase_nav2(sh, "park", False)
    assert r.status == "fail"
    assert "rtf unreadable" in r.detail
    assert f"{NAV_DEADLINE:.0f} s" in r.detail


import json
from scripts.sim import cmd_start, cmd_stop, cmd_status, save_state, load_state
from scripts.sim import PHASE_NAMES


def _patched(monkeypatch, results):
    """Replace every phase with a stub returning the scripted PhaseResult."""
    import scripts.sim as S
    monkeypatch.setattr(S, "phase_clean", lambda sh: results[0])
    monkeypatch.setattr(S, "phase_config", lambda sh, c: results[1])
    monkeypatch.setattr(S, "phase_launch", lambda sh, w, p: (results[2], 4242))
    monkeypatch.setattr(S, "phase_controllers", lambda sh: results[3])
    monkeypatch.setattr(S, "phase_robot", lambda sh, w, z: results[4])
    monkeypatch.setattr(S, "phase_extras", lambda sh, c, lp: (results[5], None))
    monkeypatch.setattr(S, "phase_nav2", lambda sh, w, n, loc=None: (results[6], 900))
    monkeypatch.setattr(S, "phase_slam", lambda sh, w: (results[6], 900))


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


def test_cmd_status_no_state_file_does_not_crash(monkeypatch, tmp_path):
    import scripts.sim as S
    monkeypatch.setattr(S, "STATE_FILE", tmp_path / "state.json")
    lines = []
    rc = cmd_status(FakeShell(run_out={"list_controllers": ""}), out=lines.append)
    assert any(l.startswith("no state file") for l in lines)
    assert lines[-1].startswith("NOT READY") or lines[-1].startswith("READY")
    assert rc == 12


class LiveStatusShell(FakeShell):
    """Alive launch pid, stepping sim time, active controllers, alive bridge."""
    def __init__(self, alive=True, stats_seq=None, list_out=LIST_OK, ready=None):
        super().__init__(run_out={"list_controllers": list_out})
        self.alive = alive
        self.stats_seq = list(stats_seq) if stats_seq is not None else [STATS, STATS.replace("sec: 12", "sec: 13")]
        self.ready = [] if ready is None else ready
    def pid_alive(self, pid): return self.alive
    def world_stats(self, world): return self.stats_seq.pop(0) if len(self.stats_seq) > 1 else self.stats_seq[0]
    def nav_ready(self): return self.ready


def _write_state(path, **overrides):
    st = {"world": "park", "config": "default", "no_nav": False, "launch_pid": 4242,
          "bridge_pid": 5000, "nav_pid": 900, "started_at": 0, "phase_reached": 6}
    st.update(overrides)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(st))
    return st


def test_cmd_status_live_all_good_invokes_phase_robot(monkeypatch, tmp_path):
    import scripts.sim as S
    p = tmp_path / "state.json"; _write_state(p)
    monkeypatch.setattr(S, "STATE_FILE", p)
    calls = []
    monkeypatch.setattr(S, "phase_robot", lambda sh, w, z: (calls.append((w, z)) or PhaseResult(4, "robot", "ok", "d")))
    sh = LiveStatusShell()
    lines = []
    rc = cmd_status(sh, out=lines.append)
    assert rc == 0
    # park has a nav2 config and this state's nav_pid is alive+ready, so
    # status - like start - must require and confirm it, not merely note it
    # if present (CLAUDE.md review finding 2).
    assert lines[-1] == "READY park default nav"
    assert calls == [("park", None)]


def test_cmd_status_live_robot_silent_is_not_ready(monkeypatch, tmp_path):
    import scripts.sim as S
    p = tmp_path / "state.json"; _write_state(p)
    monkeypatch.setattr(S, "STATE_FILE", p)
    monkeypatch.setattr(S, "phase_robot", lambda sh, w, z: PhaseResult(4, "robot", "fail", "silent"))
    sh = LiveStatusShell()
    lines = []
    rc = cmd_status(sh, out=lines.append)
    assert rc == 14
    assert lines[-1].startswith("NOT READY")


class ReadOnlyStatusShell(LiveStatusShell):
    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.launched = []
    def launch(self, cmd, log):
        self.launched.append((cmd, log))
        return 9999


def test_cmd_status_is_read_only(monkeypatch, tmp_path):
    import scripts.sim as S
    p = tmp_path / "state.json"; _write_state(p)
    monkeypatch.setattr(S, "STATE_FILE", p)
    monkeypatch.setattr(S, "phase_robot", lambda sh, w, z: PhaseResult(4, "robot", "ok", "d"))
    sh = ReadOnlyStatusShell()
    cmd_status(sh, out=lambda s: None)
    assert sh.launched == []
    assert sh.killed == []
    assert not any("spawner" in c for c in sh.calls)


from scripts.sim import check_ros_env, main


def test_check_ros_env_none_when_rclpy_importable(monkeypatch):
    import importlib.util
    monkeypatch.setattr(importlib.util, "find_spec", lambda name: object())
    assert check_ros_env() is None


def test_check_ros_env_message_when_rclpy_missing(monkeypatch):
    import importlib.util
    monkeypatch.setattr(importlib.util, "find_spec", lambda name: None)
    err = check_ros_env()
    assert err is not None and "source /opt/ros/jazzy/setup.bash" in err


def test_main_exits_2_with_fail_line_when_ros_not_sourced(monkeypatch, capsys):
    import scripts.sim as S
    monkeypatch.setattr(S, "check_ros_env", lambda: "ROS 2 is not sourced in this shell - run: source /opt/ros/jazzy/setup.bash")
    rc = main(["start", "park"])
    out = capsys.readouterr().out
    assert rc == 2
    # "FAIL env:" not "FAIL 0 env:" - the latter collided with the real
    # phase 0 (clean), which has its own documented exit code 10
    # (CLAUDE.md review finding 5).
    assert out.strip().startswith("FAIL env:")


from tools.check_nav2_ready import _lifecycle_states_from_clients


class _FakeFuture:
    def __init__(self, result):
        self._result = result

    def result(self):
        return self._result


class _FakeState:
    def __init__(self, label):
        self.label = label


class _FakeGetStateResult:
    def __init__(self, label):
        self.current_state = _FakeState(label)


class _FakeClient:
    """Stand-in for an rclpy Client: no ROS import required."""

    def __init__(self, service_available, response_label=None):
        self._service_available = service_available
        self._response_label = response_label
        self.wait_for_service_calls = []
        self.call_async_calls = 0

    def wait_for_service(self, timeout_sec):
        self.wait_for_service_calls.append(timeout_sec)
        return self._service_available

    def call_async(self, request):
        self.call_async_calls += 1
        result = _FakeGetStateResult(self._response_label) if self._response_label else None
        return _FakeFuture(result)


def _noop_spin(future, timeout_sec):
    pass


def test_lifecycle_states_labels_unavailable_when_wait_for_service_fails():
    """Defect C regression: a service never discovered must be labelled
    "unavailable", exactly as an unresponsive call_async was labelled before
    this fix, so failure text and printed lines keep the same shape."""
    client = _FakeClient(service_available=False)
    clients = {"map_server": client}
    states = _lifecycle_states_from_clients(
        ["map_server"], clients, spin_until_future_complete=_noop_spin,
        make_request=object, first_service_timeout=10.0, per_service_timeout=3.0)
    assert states == {"map_server": "unavailable"}
    assert client.call_async_calls == 0


def test_lifecycle_states_calls_client_once_available_and_returns_state():
    """A client that becomes available (wait_for_service True) must actually
    be called, and its reported state returned verbatim."""
    client = _FakeClient(service_available=True, response_label="active")
    clients = {"map_server": client}
    states = _lifecycle_states_from_clients(
        ["map_server"], clients, spin_until_future_complete=_noop_spin,
        make_request=object, first_service_timeout=10.0, per_service_timeout=3.0)
    assert states == {"map_server": "active"}
    assert client.call_async_calls == 1


def test_lifecycle_states_first_node_gets_the_generous_discovery_budget():
    """The first service pays the DDS discovery cost for the whole node;
    later services reuse that discovered graph and get the short budget."""
    c1 = _FakeClient(service_available=True, response_label="active")
    c2 = _FakeClient(service_available=True, response_label="active")
    clients = {"map_server": c1, "planner_server": c2}
    _lifecycle_states_from_clients(
        ["map_server", "planner_server"], clients,
        spin_until_future_complete=_noop_spin, make_request=object,
        first_service_timeout=10.0, per_service_timeout=3.0)
    assert c1.wait_for_service_calls == [10.0]
    assert c2.wait_for_service_calls == [3.0]


def test_lifecycle_states_unresponsive_call_async_still_labelled_unavailable():
    """Service is discovered (wait_for_service True) but the GetState call
    itself never completes - future.result() is None - must still read
    "unavailable", matching the pre-fix label for this case."""
    client = _FakeClient(service_available=True, response_label=None)
    clients = {"map_server": client}
    states = _lifecycle_states_from_clients(
        ["map_server"], clients, spin_until_future_complete=_noop_spin,
        make_request=object, first_service_timeout=10.0, per_service_timeout=3.0)
    assert states == {"map_server": "unavailable"}


# ------------------------------------------------------- review fix wave

def test_pid_alive_detects_exited_child_not_a_zombie(tmp_path, monkeypatch):
    """Finding 1: os.kill(pid, 0) reports True for a zombie (exited but not
    reaped), so a bridge/launch that dies instantly used to read as alive
    forever. Prove pid_alive is now truthful for a REAL short-lived process
    this Shell actually spawned - bounded, hermetic, no ROS."""
    import time as _t
    import scripts.sim as sim
    monkeypatch.setattr(sim, "ROS_SETUP", ":")
    sh = sim.Shell()
    log = tmp_path / "log.txt"
    pid = sh.launch("true", str(log))
    end = _t.monotonic() + 5
    while sh.pid_alive(pid) and _t.monotonic() < end:
        _t.sleep(0.02)
    assert sh.pid_alive(pid) is False


def test_pid_alive_falls_back_to_kill_probe_for_foreign_pid():
    """`status` reads pids back from the state file across process
    boundaries, where there is no Popen to consult - that path must still
    work via the os.kill(pid, 0) probe."""
    import scripts.sim as sim
    sh = sim.Shell()
    assert sh.pid_alive(os.getpid()) is True
    assert sh.pid_alive(999999) is False


import os  # noqa: E402 (kept local to this section, mirrors sim.py's own import)


def test_phase_extras_confirms_liveness_at_end_of_window_not_at_start():
    """Finding 1c: poll used to return on the FIRST truthy probe, so a
    bridge that dies at t=1s (within the 5s window) still passed if it was
    alive when first checked. Prove the fail path is reached when the
    fake shell reports dead partway through the window."""
    class Sh(FakeShell):
        def __init__(self):
            super().__init__(files={f"{REPO}/robot_configs/robot_full.yaml": FULL_YAML, **FILES})
            self.polls = 0
        def launch(self, cmd, log): return 777
        def pid_alive(self, pid):
            self.polls += 1
            return self.polls < 3  # alive on first checks, dies mid-window
    sh = Sh()
    r, pid = phase_extras(sh, "full", 4242)
    assert r.status == "fail" and "died" in r.detail


def test_cmd_status_fails_when_nav_required_but_pid_missing(monkeypatch, tmp_path):
    """Finding 2: a start that never reached phase 6 (spawner race, etc.)
    leaves nav_pid None forever. status must FAIL nav2 for a world that has
    a nav2 config, not silently skip the phase and print READY."""
    import scripts.sim as S
    p = tmp_path / "state.json"
    _write_state(p, nav_pid=None)
    monkeypatch.setattr(S, "STATE_FILE", p)
    monkeypatch.setattr(S, "phase_robot", lambda sh, w, z: PhaseResult(4, "robot", "ok", "d"))
    sh = LiveStatusShell()
    lines = []
    rc = cmd_status(sh, out=lines.append)
    assert rc == 16
    assert any(l.startswith("[6 nav2") and "fail" in l for l in lines)
    assert lines[-1].startswith("NOT READY")


def test_cmd_status_skips_nav_requirement_when_no_nav_recorded(monkeypatch, tmp_path):
    """A deliberate --no-nav start must not be misread as 'nav2 never got
    there' - status must distinguish the two via the recorded no_nav flag."""
    import scripts.sim as S
    p = tmp_path / "state.json"
    _write_state(p, nav_pid=None, no_nav=True)
    monkeypatch.setattr(S, "STATE_FILE", p)
    monkeypatch.setattr(S, "phase_robot", lambda sh, w, z: PhaseResult(4, "robot", "ok", "d"))
    sh = LiveStatusShell()
    lines = []
    rc = cmd_status(sh, out=lines.append)
    assert rc == 0
    assert lines[-1] == "READY park default"


def test_cmd_start_records_no_nav_flag(monkeypatch, tmp_path):
    import scripts.sim as S
    monkeypatch.setattr(S, "STATE_FILE", tmp_path / "state.json")
    _patched(monkeypatch, OK7)
    cmd_start(FakeShell(), parse_args(["start", "park", "--no-nav"]), out=lambda s: None)
    st = json.loads((tmp_path / "state.json").read_text())
    assert st["no_nav"] is True


def test_main_wraps_unhandled_exception_in_a_verdict_line(monkeypatch, capsys):
    """Finding 3: an unhandled exception anywhere in dispatch must still
    produce a single FAIL line and a documented exit code, not a bare
    traceback."""
    import scripts.sim as S
    monkeypatch.setattr(S, "check_ros_env", lambda: None)

    class BoomShell:
        pass

    monkeypatch.setattr(S, "Shell", lambda: BoomShell())

    def boom(sh, args, out=print):
        raise RuntimeError("kaboom")
    monkeypatch.setattr(S, "cmd_start", boom)
    rc = S.main(["start", "park"])
    out = capsys.readouterr().out
    assert rc == 2
    assert "FAIL start: kaboom" in out


def test_main_env_failure_uses_non_phase_prefix(monkeypatch, capsys):
    """Finding 5: 'FAIL 0 env: ...' collided with the real phase 0 (clean),
    which has its own documented exit code 10. Must not share the '0'
    prefix with a real phase number."""
    import scripts.sim as S
    monkeypatch.setattr(S, "check_ros_env", lambda: "ROS 2 is not sourced")
    rc = S.main(["status"])
    out = capsys.readouterr().out
    assert rc == 2
    assert "FAIL env: ROS 2 is not sourced" in out
    assert "FAIL 0" not in out


# ---------------------------------------------------------------- --slam / --localization

from scripts.sim import LOCALIZATION_CHOICES


def test_parse_args_localization_defaults_to_gps():
    a = parse_args(["start", "park"])
    assert a.localization == "gps" and a.slam is False


def test_parse_args_localization_explicit_choice():
    a = parse_args(["start", "park", "--localization", "rssi"])
    assert a.localization == "rssi"


def test_parse_args_localization_rejects_unknown_backend():
    import pytest
    with pytest.raises(SystemExit):
        parse_args(["start", "park", "--localization", "bogus"])


def test_parse_args_localization_accepts_amcl_choice_early():
    """amcl is accepted by the CLI ahead of park_stock.launch.py supporting it
    (a separate, later task) - argparse must not reject it."""
    a = parse_args(["start", "park", "--localization", "amcl"])
    assert a.localization == "amcl"


def test_parse_args_slam_sets_flag_and_leaves_localization_unset():
    a = parse_args(["start", "park", "--slam"])
    assert a.slam is True
    assert a.localization is None


def test_parse_args_slam_with_localization_errors():
    import pytest
    with pytest.raises(SystemExit):
        parse_args(["start", "park", "--slam", "--localization", "rssi"])


def test_parse_args_slam_with_no_nav_errors():
    import pytest
    with pytest.raises(SystemExit):
        parse_args(["start", "park", "--slam", "--no-nav"])


def test_parse_args_slam_alone_is_fine():
    a = parse_args(["start", "park", "--slam"])
    assert a.slam is True and a.no_nav is False


from scripts.sim import phase_slam, SLAM_LOG


class SlamShell(NavShell):
    def slam_ready(self):
        return self.ready_seq.pop(0) if len(self.ready_seq) > 1 else self.ready_seq[0]


def test_phase_slam_launches_park_slam_and_reports_ok_when_ready():
    sh = SlamShell(ready_seq=[[]])
    r, pid = phase_slam(sh, "park")
    assert r.status == "ok" and r.name == "slam" and pid == 900
    assert sh.launched[0][1] == SLAM_LOG
    assert "park_slam.launch.py" in sh.launched[0][0]


def test_phase_slam_fails_reporting_last_failures():
    sh = SlamShell(ready_seq=[["map -> odom not published"]])
    r, _ = phase_slam(sh, "park")
    assert r.status == "fail" and "map -> odom" in r.detail


def test_phase_nav2_passes_localization_through_to_launch_args():
    sh = NavShell([[]])
    phase_nav2(sh, "park", False, "rssi")
    assert "localization:=rssi" in sh.launched[0][0]


def test_phase_nav2_defaults_localization_to_gps():
    sh = NavShell([[]])
    phase_nav2(sh, "park", False)
    assert "localization:=gps" in sh.launched[0][0]


def test_cmd_start_slam_calls_phase_slam_not_phase_nav2(monkeypatch, tmp_path):
    import scripts.sim as S
    monkeypatch.setattr(S, "STATE_FILE", tmp_path / "state.json")
    _patched(monkeypatch, OK7)
    calls = []
    monkeypatch.setattr(S, "phase_slam", lambda sh, w: (calls.append((w,)) or PhaseResult(6, "slam", "ok", "d"), 900))

    def boom_nav2(*a, **k):
        raise AssertionError("phase_nav2 must not run under --slam")
    monkeypatch.setattr(S, "phase_nav2", boom_nav2)

    lines = []
    rc = cmd_start(FakeShell(), parse_args(["start", "park", "--slam"]), out=lines.append)
    assert rc == 0 and calls == [("park",)]
    assert lines[-1] == "READY park default slam"
    st = json.loads((tmp_path / "state.json").read_text())
    assert st["slam"] is True


def test_cmd_start_gps_default_reports_nav_tag(monkeypatch, tmp_path):
    import scripts.sim as S
    monkeypatch.setattr(S, "STATE_FILE", tmp_path / "state.json")
    _patched(monkeypatch, OK7)
    lines = []
    rc = cmd_start(FakeShell(), parse_args(["start", "park"]), out=lines.append)
    assert rc == 0 and lines[-1] == "READY park default nav"
    st = json.loads((tmp_path / "state.json").read_text())
    assert st["slam"] is False and st["localization"] == "gps"


class SlamStatusShell(LiveStatusShell):
    def __init__(self, *a, ready=None, **kw):
        super().__init__(*a, ready=ready, **kw)
    def slam_ready(self):
        return self.ready


def test_cmd_status_slam_uses_slam_ready_and_slam_tag(monkeypatch, tmp_path):
    import scripts.sim as S
    p = tmp_path / "state.json"
    _write_state(p, slam=True)
    monkeypatch.setattr(S, "STATE_FILE", p)
    monkeypatch.setattr(S, "phase_robot", lambda sh, w, z: PhaseResult(4, "robot", "ok", "d"))
    sh = SlamStatusShell(ready=[])

    def boom_nav_ready():
        raise AssertionError("nav_ready must not be called in slam status")
    sh.nav_ready = boom_nav_ready

    lines = []
    rc = cmd_status(sh, out=lines.append)
    assert rc == 0
    assert lines[-1] == "READY park default slam"


def test_cmd_status_slam_not_ready_reports_fail(monkeypatch, tmp_path):
    import scripts.sim as S
    p = tmp_path / "state.json"
    _write_state(p, slam=True)
    monkeypatch.setattr(S, "STATE_FILE", p)
    monkeypatch.setattr(S, "phase_robot", lambda sh, w, z: PhaseResult(4, "robot", "ok", "d"))
    sh = SlamStatusShell(ready=["map -> odom not published"])
    sh.nav_ready = lambda: (_ for _ in ()).throw(AssertionError("must not call nav_ready"))
    lines = []
    rc = cmd_status(sh, out=lines.append)
    assert rc != 0
    assert lines[-1].startswith("NOT READY")


from tools.check_nav2_ready import map_topic_published


def test_map_topic_published_true_only_with_one_publisher():
    assert map_topic_published("Type: nav_msgs/msg/OccupancyGrid\nPublisher count: 1\n")
    assert not map_topic_published("Publisher count: 0\n")
    assert not map_topic_published("")


from tools.check_nav2_ready import slam_ready as _slam_ready_impl


def test_slam_ready_reports_map_odom_failure():
    fails = _slam_ready_impl(sh=lambda cmd, timeout=None: "", tf_check=lambda: False)
    assert any("map -> odom" in f for f in fails)
    assert any("/a200_0000/map" in f for f in fails)


def test_slam_ready_empty_when_transform_and_map_topic_ok():
    fails = _slam_ready_impl(
        sh=lambda cmd, timeout=None: "Publisher count: 1\n",
        tf_check=lambda: True)
    assert fails == []
