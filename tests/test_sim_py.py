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
