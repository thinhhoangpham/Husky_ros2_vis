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
