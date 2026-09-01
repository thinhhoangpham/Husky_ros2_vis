"""Bounded-segment drive geometry: clearance gating and turn progress.

Pure python: no ROS, no Gazebo, so it is unit-testable without a sim (same
split as tools/sdf_geometry.py). tools/drive_segment.py is the ROS wrapper.

Two rules live here, each written against a failure observed during the
2026-08-31 warehouse SLAM run (DEMO.md, "Demo: warehouse SLAM mapping run"):

**Direction-aware clearance.** The original rule aborted a segment whenever the
minimum lidar range over the *whole* 360 deg scan fell below 1.0 m. Once an
obstacle sat 0.85 m off the front-right, every following segment aborted at its
first check - including the reverse and the turn that would have escaped - and
the run had to be hand-flown out. The rule here evaluates only the region the
robot is about to sweep:

  * straight motion sweeps a corridor, not a cone. A return at range r and
    bearing theta (0 = +x = forward, CCW positive) lies in the corridor iff its
    lateral offset |r*sin(theta)| <= CORRIDOR_HALF_WIDTH and it is on the side
    being driven toward (r*cos(theta) > 0 forward, < 0 reverse). Clearance is
    the along-track distance |r*cos(theta)| of the nearest such return, because
    that - not slant range - is what the robot closes on.
    CORRIDOR_HALF_WIDTH is the robot half-width 0.34 m plus a 0.10 m margin.
    Expressed as an angle it is +/- asin(0.44 / 1.00) = +/- 26.1 deg at the
    STRAIGHT_CLEARANCE stop distance, widening as the obstacle gets nearer -
    which is the point: a fixed cone is either too narrow up close or too wide
    far out.
  * an in-place turn sweeps the robot's circumscribed circle in every
    direction, so it is gated on the minimum *slant* range over the full scan
    against TURN_CLEARANCE = rotation radius + 0.10 m. The A200 footprint is
    0.99 x 0.67 m; using the same 0.34 m half-width as the corridor, that radius
    is hypot(0.495, 0.34) = 0.600 m.

  * CONTACT_CLEARANCE is a separate, much smaller omnidirectional veto: below
    it the robot is about to touch something and no motion of any kind is
    allowed. This is the only rule that can strand the robot, and at 0.35 m
    (roughly the half-width) being stranded is the correct outcome.

An 0.85 m frontal obstacle therefore vetoes driving forward while still
permitting reverse and either turn - the escape the original rule denied.

**Unwrapped turn progress.** Progress was measured as the shortest-angle
difference between the current and the starting heading, which wraps at pi: a
commanded 3.1416 rad turn read 1.31 rad of progress, never terminated, and
over-rotated to ~5.0 rad. TurnProgress accumulates the per-sample delta
instead, so total rotation is unbounded and a turn of exactly pi, more than pi,
or several revolutions all terminate at the commanded angle.
"""

from __future__ import annotations

import math
from typing import Iterable, Iterator, Sequence

# --- robot geometry (Clearpath A200, footprint 0.99 x 0.67 m) ---------------
ROBOT_HALF_WIDTH = 0.34
ROBOT_HALF_LENGTH = 0.495
CLEARANCE_MARGIN = 0.10

CORRIDOR_HALF_WIDTH = ROBOT_HALF_WIDTH + CLEARANCE_MARGIN          # 0.44 m
ROTATION_RADIUS = math.hypot(ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH)  # 0.600 m

# --- clearance thresholds ---------------------------------------------------
STRAIGHT_CLEARANCE = 1.00                       # along-track, in the corridor
TURN_CLEARANCE = ROTATION_RADIUS + CLEARANCE_MARGIN   # 0.700 m, omnidirectional
CONTACT_CLEARANCE = 0.35                        # omnidirectional, vetoes all motion

MODES = ("forward", "reverse", "turn_left", "turn_right")


def wrap_pi(angle: float) -> float:
    """Fold an angle into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class TurnProgress:
    """Unwrapped accumulated heading change (Bug B).

    Feed every heading sample; `total` is the signed rotation since the first
    sample and grows without bound, so `abs(total) >= target` is a correct
    termination test for any target, including >= pi.

    Correct only while consecutive samples are less than pi apart - true at any
    sane control rate (0.4 rad/s at 20 Hz is 0.02 rad per sample).
    """

    def __init__(self, start: float) -> None:
        self.start = start
        self.previous = start
        self.total = 0.0

    def update(self, heading: float) -> float:
        self.total += wrap_pi(heading - self.previous)
        self.previous = heading
        return self.total

    @property
    def magnitude(self) -> float:
        return abs(self.total)


def scan_points(ranges: Sequence[float], angle_min: float, angle_increment: float,
                range_min: float, range_max: float) -> Iterator[tuple[float, float]]:
    """Yield (range, bearing) for the finite, in-band returns of a LaserScan.

    inf/nan and out-of-band values are no-returns, not obstacles: treating a
    max-range reading as a hit is the same mistake as CLAUDE.md gotcha #30 in
    the opposite direction.
    """
    for i, r in enumerate(ranges):
        if r is None or math.isnan(r) or math.isinf(r):
            continue
        if r < range_min or r > range_max:
            continue
        yield r, angle_min + i * angle_increment


def corridor_clearance(points: Iterable[tuple[float, float]], reverse: bool = False) -> float:
    """Nearest along-track distance inside the swept corridor, or inf if clear."""
    best = math.inf
    sign = -1.0 if reverse else 1.0
    for r, theta in points:
        along = sign * r * math.cos(theta)
        if along <= 0.0:
            continue
        if abs(r * math.sin(theta)) > CORRIDOR_HALF_WIDTH:
            continue
        best = min(best, along)
    return best


def omni_clearance(points: Iterable[tuple[float, float]]) -> float:
    """Nearest slant range in any direction, or inf if the scan is empty."""
    best = math.inf
    for r, _theta in points:
        best = min(best, r)
    return best


def clearance_check(points: Iterable[tuple[float, float]], mode: str
                    ) -> tuple[bool, float, str]:
    """Decide whether `mode` may proceed.

    Returns (allowed, measured clearance in metres, reason). The clearance
    reported is the one the decision was made on: along-track for straight
    motion, slant range for a turn or a contact veto.
    """
    if mode not in MODES:
        raise ValueError(f"unknown mode {mode!r}; expected one of {MODES}")
    points = list(points)

    nearest = omni_clearance(points)
    if nearest < CONTACT_CLEARANCE:
        return False, nearest, (f"contact imminent: {nearest:.2f} m < "
                                f"{CONTACT_CLEARANCE:.2f} m in some direction")

    if mode in ("turn_left", "turn_right"):
        if nearest < TURN_CLEARANCE:
            return False, nearest, (f"rotation footprint blocked: {nearest:.2f} m < "
                                    f"{TURN_CLEARANCE:.2f} m")
        return True, nearest, "clear to turn"

    reverse = mode == "reverse"
    ahead = corridor_clearance(points, reverse=reverse)
    if ahead < STRAIGHT_CLEARANCE:
        return False, ahead, (f"{mode} corridor blocked: {ahead:.2f} m < "
                              f"{STRAIGHT_CLEARANCE:.2f} m")
    return True, ahead, f"clear to go {mode}"
