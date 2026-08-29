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
