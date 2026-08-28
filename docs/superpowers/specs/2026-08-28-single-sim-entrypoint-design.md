# Single sim entry point — `scripts/sim.py`

Date: 2026-08-28. Status: approved design, not yet implemented.

## Problem

Starting the simulation today spans three runbooks (`CLEAN_SIM.md`,
`RUN_SIM.md`, `NAV_PARK.md` Steps 1–4), three launch entry points with
different config semantics (`apply_config.sh`, `run_husky_sim.sh`,
`park_sim.launch.py`), and ~15 manual gate commands executed by an agent.
Failures come from three independent sources:

1. dirty state — orphan nodes and stale `/dev/shm` segments (CLAUDE.md #11, #12);
2. wrong config — each entry point applies (or does not apply) the robot config differently (#13);
3. the controller spawner race — 42 % of park starts lose it and no script recovers (#27, #37).

The gates themselves produce false results (#14, #38), and the
`sim-operator` agent still references skills deleted on 2026-08-27.

## Goal

One command that, when it exits 0, guarantees: machine was cleaned, the
requested config is live, Gazebo is stepping, both controllers are active,
the robot is publishing and standing at its spawn pose, and — where a nav2
config exists for the world — nav2 is up with `map -> odom` present.
Acceptance: 5 consecutive `start park` runs → 5/5 READY.

## CLI

```
python3 scripts/sim.py start <world> [--config NAME] [--no-nav]
                                     [--x X --y Y --z Z --yaw YAW] [--clean-on-fail]
python3 scripts/sim.py stop      # clean + verify; exit 0 only when CLEAN
python3 scripts/sim.py status    # re-run the gates read-only, one line each
```

- `--config` defaults to `default`. Any `robot_configs/robot_<NAME>.yaml` is valid.
- nav2 runs iff `config/nav2_<world>.yaml` exists (park only today); `--no-nav` opts out.
- `start` always runs the clean phase first; there is no "already clean" shortcut.
- Pose flags forward to `park_sim.launch.py`; unset ones use `WORLD_SPAWN_POSES`.
- Dependencies: Python stdlib + rclpy (sourced `/opt/ros/jazzy`). No new packages.

## Phases and gates

Phases run strictly in order. The first failing gate ends the run with exit
code `10 + phase`, one line `FAIL <phase>: <observation>`, and a pointer to
`/tmp/sim.log` (and `/tmp/nav.log`). Success prints one line per phase and a
final `READY <world> <config> [nav]`.

All waiting is deadline-bounded polling of a concrete signal with a short
probe interval — never a fixed sleep, never `timeout` on a `ros2` CLI (#8).

| # | Phase | Action | Gate | Deadline |
|---|---|---|---|---|
| 0 | clean | kill by `kill_sim.sh` pattern list (read from that file — one list) plus the `a200_0000\|gz sim\|gz_tools_vendor` sweep, skipping `bash -c` wrappers and self; `ros2 daemon stop`; `rm /dev/shm/fastrtps_* sem.fastrtps_*` | zero matching pids; `fastrtps` shm count 0 — if nonzero, re-read once (transient release, CLAUDE.md #12); FAIL lists survivors by full cmdline | 15 s |
| 1 | config | `scripts/apply_config.sh <config>`; parse its "sensors in SDF" block | every sensor declared in `robot_<config>.yaml` appears in SDF (#1) | — |
| 2 | launch | `setsid nohup ros2 launch launch/park_sim.launch.py world:=<w> [pose] > /tmp/sim.log` (#22) | launch pid alive **and** `/world/<w>/stats` `sim_time` strictly increases across two reads (world is stepping) | 90 s |
| 3 | controllers | query `/a200_0000/controller_manager/list_controllers` (rclpy) | both `joint_state_broadcaster` and `platform_velocity_controller` `active` → `clean`; otherwise run `ros2 run controller_manager spawner <both> --controller-manager /a200_0000/controller_manager --switch-timeout 30`, re-query → `recovered`; else FAIL | 40 s |
| 4 | robot | rclpy subscribers, sensor-data QoS, on `platform/odom`, `sensors/imu_0/data`, `sensors/lidar2d_0/scan`, `sensors/lidar3d_0/points`; Gazebo pose of `a200_0000/robot` via `/world/<w>/...` | ≥1 message on each within the deadline (bounded receive, not a topic listing — #14, #38); z within ±0.5 m of spawn z (#23) | 10 s |
| 5 | extras | only if the config yaml declares compass and/or radio: `ros_gz_bridge parameter_bridge` for `compass_0/mag`, `/broker/msgs`, `/husky/rx`, `/base_station/rx`. **Not** gps (generated bridge, #7/#15) and **not** `imu_enu` (owned by `gps_localization.launch.py`, #7) | bridge pid alive and launch pid still alive; `skip` line when the config has neither | 5 s |
| 6 | nav2 | if `config/nav2_<world>.yaml` exists and not `--no-nav`: `setsid nohup ros2 launch launch/nav_park.launch.py > /tmp/nav.log`; then the readiness checks from `tools/check_nav2_ready.py` imported as functions (single source of truth) | `map -> odom` transform present; all `LIFECYCLE` nodes active; `skip` otherwise | 60 s |

Design rule: the spawner race is *recovered* (option b), not prevented —
preventing it requires forking four Clearpath launch files (#37). The
`clean`/`recovered` word in the Phase 3 line is deliberately kept as a
measurement of how often the race is lost.

## State and failure behaviour

- `~/.husky_sim/state.json`: world, config, launch/bridge/nav pids, start
  time, highest phase reached. `status` and `stop` read it; `stop` still
  sweeps by pattern so it works without it.
- A FAIL leaves everything running for inspection unless `--clean-on-fail`.
  Only `stop` (or the Phase 0 of the next `start`) kills.
- `status` re-runs gates 2–6 read-only (no spawner, no launches) and prints
  the same phase lines; exit code as for `start`.

## Surrounding changes

| File | Change |
|---|---|
| `scripts/sim.py` | new |
| `tests/test_sim_py.py` | new — gate functions tested on captured inputs, no sim |
| `scripts/run_husky_sim.sh` | untouched; remains the manual entry point. Retirement is a separate decision |
| `scripts/kill_sim.sh` | untouched; its `PATTERNS` array is parsed by `sim.py` |
| `CLEAN_SIM.md` | body becomes `python3 scripts/sim.py stop` → must print `CLEAN`; manual steps move under "What it does / if it fails" |
| `RUN_SIM.md` | body becomes `python3 scripts/sim.py start <world> [--config …]`, the expected READY output, the exit-code table |
| `NAV_PARK.md` | Steps 1–4 → the single `start park` line; Steps 5–10 unchanged |
| `.claude/agents/sim-operator.md` | drop dead `husky-sim-restart`/`husky-run-sim` refs → `husky-sim`; "run the one command, relay its phase lines verbatim, stop on FAIL" |
| `.claude/skills/husky-sim/SKILL.md` | aligned to the above |
| `CLAUDE.md` | Workflow section points at `sim.py`; #27 notes it is handled by Phase 3 |

Nothing under `~/clearpath/` is edited; no Clearpath launch file is forked.

## Testing

Unit (`tests/test_sim_py.py`, runs without a sim): gate functions take data
(stats text, controller list, shm listing, SDF sensor list, yaml) — cases:
sim_time advancing vs stalled; controllers active/inactive/missing;
transient vs persistent shm; config-vs-SDF mismatch; exit-code and
phase-ordering with a fake runner.

Integration (live sim, run by the user or `sim-operator`):
1. `start park` × 5 → 5/5 READY; record how many Phase 3 lines say `recovered`.
2. `start lake` → READY, Phase 6 `skip`.
3. `start warehouse_ext --config full` → Phase 5 bridge up, compass topic receives.
4. Negative: start park, `kill -9` `controller_manager`, `status` → `FAIL 3`, never READY.
5. `stop` after each → `CLEAN`, and `CLEAN_SIM.md`'s manual check agrees (0 / 0).

Success criterion: an agent asked "start park" issues one command and
relays one output; the 5-run loop is 5/5.
