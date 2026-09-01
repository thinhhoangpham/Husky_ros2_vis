"""Unit tests for tools/drive_geometry.py - the pure logic behind
tools/drive_segment.py.

ROS-free by construction: drive_geometry imports nothing from rclpy, so these
run without a sourced ROS environment and without a sim.

The two bug families they pin down are the ones hit during the 2026-08-31
warehouse SLAM run (DEMO.md):
  Bug A - a whole-scan clearance rule stranded the robot, because a forward
          obstacle vetoed the reverse and the turn that would escape it.
  Bug B - turns of pi or more never terminated, because progress was measured
          with shortest-angle wrapping.
"""

import math

import pytest

from tools.drive_geometry import (
    CONTACT_CLEARANCE,
    CORRIDOR_HALF_WIDTH,
    ROTATION_RADIUS,
    STRAIGHT_CLEARANCE,
    TURN_CLEARANCE,
    TurnProgress,
    clearance_check,
    corridor_clearance,
    omni_clearance,
    scan_points,
    wrap_pi,
)


# --------------------------------------------------------------- Bug B: turns

def test_wrap_pi_folds_into_plus_minus_pi():
    assert wrap_pi(0.0) == 0.0
    assert math.isclose(wrap_pi(math.pi), math.pi, abs_tol=1e-12)
    assert math.isclose(wrap_pi(3 * math.pi / 2), -math.pi / 2, abs_tol=1e-12)
    # atan2's range is closed, so the sign at the +/-pi boundary is not
    # specified; only the magnitude is.
    assert math.isclose(abs(wrap_pi(-3 * math.pi)), math.pi, abs_tol=1e-12)


def _sweep(start, amount, step=0.02, overshoot=0.2):
    """Headings as an encoder would report them: wrapped into [-pi, pi].

    Runs `overshoot` radians past the commanded amount, as a real robot does -
    the controller stops when progress crosses the target, it does not land on
    it exactly.
    """
    sign = 1.0 if amount >= 0 else -1.0
    n = int((abs(amount) + overshoot) / step)
    return [wrap_pi(start + sign * step * i) for i in range(1, n + 1)]


def test_turn_of_exactly_pi_terminates():
    """The regression: shortest-angle progress reads pi as ~0 and never fires."""
    start = 0.3
    turn = TurnProgress(start)
    fired_at = None
    for h in _sweep(start, math.pi):
        turn.update(h)
        if turn.magnitude >= math.pi and fired_at is None:
            fired_at = turn.magnitude
    assert fired_at is not None
    assert math.isclose(fired_at, math.pi, abs_tol=0.05)


def test_turn_greater_than_pi_terminates_at_the_commanded_angle():
    """3.1416 rad read as 1.31 rad of progress in the real run."""
    start = -2.9
    target = 4.0
    turn = TurnProgress(start)
    fired_at = None
    for h in _sweep(start, target):
        turn.update(h)
        if turn.magnitude >= target and fired_at is None:
            fired_at = turn.magnitude
            break
    assert fired_at is not None
    assert math.isclose(fired_at, target, abs_tol=0.05)


def test_shortest_angle_progress_would_have_failed():
    """Pin the old behaviour so the fix cannot silently regress to it."""
    start, target = 0.3, math.pi
    headings = _sweep(start, target)
    wrapped_max = max(abs(wrap_pi(h - start)) for h in headings)
    # The wrapped measure peaks at pi and folds back, so it never *exceeds* the
    # target and the segment runs on to its timeout: the original bug.
    assert wrapped_max <= target
    assert abs(wrap_pi(headings[-1] - start)) < target - 0.15


def test_multi_revolution_turn_accumulates_without_bound():
    start = 1.0
    amount = 3 * 2 * math.pi + 0.5
    turn = TurnProgress(start)
    fired_at = None
    for h in _sweep(start, amount):
        turn.update(h)
        if turn.magnitude >= amount:
            fired_at = turn.total
            break
    assert fired_at is not None
    assert math.isclose(fired_at, amount, abs_tol=0.05)


def test_turn_progress_is_signed_and_negative_for_clockwise():
    start = 0.0
    target = math.pi + 1.0
    turn = TurnProgress(start)
    for h in _sweep(start, -target):
        turn.update(h)
        if turn.magnitude >= target:
            break
    assert turn.total < 0
    assert math.isclose(turn.magnitude, target, abs_tol=0.05)


def test_turn_progress_starts_at_zero():
    turn = TurnProgress(2.0)
    assert turn.total == 0.0
    assert turn.update(2.0) == 0.0


# ------------------------------------------------- Bug A: direction awareness

def obstacle(distance, bearing_deg):
    return (distance, math.radians(bearing_deg))


FORWARD_085 = [obstacle(0.85, 15.0)]      # the real segment-11 abort: front-right


def test_forward_obstacle_blocks_forward():
    allowed, clearance, why = clearance_check(FORWARD_085, "forward")
    assert allowed is False
    assert clearance < STRAIGHT_CLEARANCE
    assert "corridor blocked" in why


def test_forward_obstacle_permits_reversing():
    """Bug A: this was the segment the old rule refused, stranding the robot."""
    allowed, _clearance, _why = clearance_check(FORWARD_085, "reverse")
    assert allowed is True


def test_forward_obstacle_permits_both_turns():
    assert clearance_check(FORWARD_085, "turn_left")[0] is True
    assert clearance_check(FORWARD_085, "turn_right")[0] is True


def test_rear_obstacle_permits_forward_and_blocks_reverse():
    rear = [obstacle(0.6, 180.0)]
    assert clearance_check(rear, "forward")[0] is True
    assert clearance_check(rear, "reverse")[0] is False


def test_obstacle_beside_the_corridor_does_not_block_straight_motion():
    """Just outside the half-width: a wall the robot drives past, not into."""
    lateral = CORRIDOR_HALF_WIDTH + 0.05
    beside = [(math.hypot(0.5, lateral), math.atan2(lateral, 0.5))]
    assert clearance_check(beside, "forward")[0] is True


def test_obstacle_inside_the_corridor_blocks_straight_motion():
    lateral = CORRIDOR_HALF_WIDTH - 0.05
    inside = [(math.hypot(0.5, lateral), math.atan2(lateral, 0.5))]
    assert clearance_check(inside, "forward")[0] is False


def test_corridor_clearance_is_along_track_not_slant_range():
    lateral = 0.3
    along = 0.9
    pts = [(math.hypot(along, lateral), math.atan2(lateral, along))]
    assert math.isclose(corridor_clearance(pts), along, abs_tol=1e-9)


def test_turn_blocked_inside_the_rotation_footprint():
    near = [obstacle(TURN_CLEARANCE - 0.05, 90.0)]
    allowed, _c, why = clearance_check(near, "turn_left")
    assert allowed is False
    assert "rotation footprint" in why
    assert TURN_CLEARANCE > ROTATION_RADIUS


def test_contact_imminent_vetoes_every_mode():
    touching = [obstacle(CONTACT_CLEARANCE - 0.05, 200.0)]
    for mode in ("forward", "reverse", "turn_left", "turn_right"):
        allowed, _c, why = clearance_check(touching, mode)
        assert allowed is False
        assert "contact imminent" in why


def test_contact_threshold_is_much_smaller_than_the_motion_thresholds():
    assert CONTACT_CLEARANCE < TURN_CLEARANCE < STRAIGHT_CLEARANCE


def test_empty_scan_is_clear_in_every_mode():
    for mode in ("forward", "reverse", "turn_left", "turn_right"):
        allowed, clearance, _why = clearance_check([], mode)
        assert allowed is True
        assert math.isinf(clearance)


def test_unknown_mode_raises():
    with pytest.raises(ValueError):
        clearance_check([], "sideways")


# ------------------------------------------------------------- scan filtering

def test_scan_points_drops_inf_nan_and_out_of_band_returns():
    ranges = [float("inf"), float("nan"), 0.05, 30.0, 2.0]
    pts = list(scan_points(ranges, angle_min=0.0, angle_increment=0.1,
                           range_min=0.1, range_max=25.0))
    assert pts == [(2.0, 0.4)]


def test_scan_points_bearings_follow_angle_min_and_increment():
    pts = list(scan_points([1.0, 1.0], angle_min=-math.pi, angle_increment=0.5,
                           range_min=0.1, range_max=25.0))
    assert [round(t, 6) for _r, t in pts] == [round(-math.pi, 6),
                                              round(-math.pi + 0.5, 6)]


def test_omni_clearance_is_the_nearest_return_in_any_direction():
    pts = [obstacle(3.0, 0.0), obstacle(0.7, 175.0), obstacle(1.2, -90.0)]
    assert math.isclose(omni_clearance(pts), 0.7)
    assert math.isinf(omni_clearance([]))
