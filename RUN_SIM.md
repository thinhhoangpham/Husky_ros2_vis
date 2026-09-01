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

Phase 4 (`robot`) also checks the renderer, not just the ROS graph and
physics server: the Gazebo GUI can finish loading its scene while the robot
is being spawned and silently miss it, leaving physics/topics fine but
nothing visible in the window (see `.claude/agents/sim-operator.md`). It
queries `gz service .../scene/info` for the robot's model count and checks
that a GUI process (not the server) is alive; either being wrong fails phase
4 with `... GUI missed the spawn ...`. This failure is intermittent
(~1 in 3 park starts, observed 2026-08-31) and the only remedy is a full
clean+restart, so `sim.py start` retries the *entire* cycle automatically,
up to 3 total attempts, whenever phase 4 fails for this reason specifically
(never for a silent topic or a robot that fell through the terrain — see
`--z`/gotcha #23). Pass `--no-retry` to disable this and fail on the first
attempt.

To prevent the race rather than only retry after it, `park_sim.launch.py`
takes a `spawn_delay` argument that sequences the robot spawn after a fixed
launch-time delay, so the GUI cannot miss the spawn's creation event mid-load
(park is the heaviest world — ~97 models, ~221 MB of textures including a
46 MB normal map used 16x — which is why only park shows this failure).
`sim.py` sets it automatically per world (`SPAWN_DELAY_S` in `scripts/sim.py`,
currently 15.0 s for park, 0.0 elsewhere) — this is launch-time sequencing,
not a readiness poll, since gz-sim exposes no GUI-side "scene finished
loading" signal to actually wait on; the retry above remains the backstop
for whatever the delay does not catch.

`--config full` on a world with a nav2 config (park) currently fails phase 6:
nav2's lifecycle bring-up stalls under `full` (`filter_mask_server`,
`costmap_filter_info_server`, `controller_server` never activate). This is a
known open nav2 bring-up issue, not a `sim.py` regression; `default` is the
verified path.

## Step 2 — Read the output

Expected shape (park):

```
[0 clean      ] ok   killed 0, shm 0
[1 config     ] ok   default  (sensors: gps_0 imu_0 lidar2d_0 lidar3d_0)
[2 launch     ] ok   pid 41233, park stepping after 6.8 s
[3 controllers] ok   clean            <- or: recovered  (...)
[4 robot      ] ok   pose 45.64 0.02 3.12  4/4 topics receiving  renderer ok
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
| 2 | usage, ROS not sourced (`FAIL env: ...`), or an unhandled exception (`FAIL <cmd>: ...`) |

Logs: `/tmp/sim.log`, `/tmp/bridge.log`, `/tmp/nav.log`.

## Step 3 — Later checks

```bash
python3 scripts/sim.py status     # re-runs the gates read-only
python3 scripts/sim.py stop       # must print CLEAN
```

`scripts/run_husky_sim.sh` remains as a manual entry point; do not run it
alongside `sim.py` (two bridges on the same topics).
