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
