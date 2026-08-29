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
