# Running the simulation

Steps only. Rationale lives in `CLAUDE.md` (gotcha #6 for the stock park
chain, gotcha #39 for why its stages are separate commands); `sim.py` design
in
`docs/superpowers/specs/2026-08-28-single-sim-entrypoint-design.md`.

| World | Path |
|---|---|
| `park` | **Part A — stock Clearpath launchers** |
| `lake`, `warehouse_ext`, `warehouse_ramp`, and Clearpath's six stock worlds (`warehouse`, `construction`, `office`, `orchard`, `pipeline`, `solar_farm`) | **Part B — `scripts/sim.py`** |

Stopping is the same for both: `python3 scripts/sim.py stop` (Part C).

---

# Part A — park (stock launchers)

## A1 — Clean first

Run `CLEAN_SIM.md` in full. **This path does not clean itself.**

```bash
cd ~/Documents/Husky_viz
source /opt/ros/jazzy/setup.bash
python3 scripts/sim.py stop
```

Required last line: `CLEAN`. Do not continue without it.

## A2 — Apply the config

```bash
cd ~/Documents/Husky_viz
source /opt/ros/jazzy/setup.bash
./scripts/apply_config.sh default
```

Required: `imu_enu` appears in the printed SDF sensor list.

## A3 — Stage 1: the world

Each stage below runs detached (gotcha #22) and shell state does not persist
between them, so **every** `bash -c` repeats the `source` and the
`AMENT_PREFIX_PATH` export. **Run the stages as separate commands, in order,
and do not continue past a stage until its gates pass** (gotcha #39).

```bash
setsid nohup bash -c 'source /opt/ros/jazzy/setup.bash && \
  export AMENT_PREFIX_PATH=/home/thinhpham/Documents/Husky_viz/gz:$AMENT_PREFIX_PATH && \
  ros2 launch clearpath_gz gz_sim.launch.py \
    world:=/home/thinhpham/Documents/Husky_viz/worlds/park' \
  > /tmp/gz_sim.log 2>&1 & disown
```

The world path is **absolute and without `.sdf`**.

Gates — all must hold before A4. A first reading taken immediately after
launch can be transient (gotcha #14 — RTF read `0.0011` at `iterations: 2`
before `1.0000`): re-read once with another one-shot command. Never sleep,
never poll in a loop.

```bash
# world stepping — expect real_time_factor ~1.0
gz topic -e -t /world/park/stats -n 1

# meshes resolved — expect 0
grep -c -E "Error Code 14|Error Code 9|Failed to load a world" /tmp/gz_sim.log

# GUI up — expect /gui/copy listed (17 /gui/* services)
gz service -l | grep -c "^/gui/"

# GUI renderer started — expect both nonzero
grep -c "MinimalScene" /tmp/gz_sim.log
grep -c -i "render thread" /tmp/gz_sim.log
```

## A4 — Stage 2: the robot

Only after every A3 gate passed.

```bash
setsid nohup bash -c 'source /opt/ros/jazzy/setup.bash && \
  export AMENT_PREFIX_PATH=/home/thinhpham/Documents/Husky_viz/gz:$AMENT_PREFIX_PATH && \
  ros2 launch clearpath_gz robot_spawn.launch.py world:=park \
    x:=45.64 y:=0.02 z:=3.3 yaw:=2.6132' \
  > /tmp/robot_spawn.log 2>&1 & disown
```

`robot_spawn`'s `world` is the **bare name** `park`. The `x/y/z/yaw`
arguments are mandatory — stock has no `WORLD_SPAWN_POSES` and omitting them
drops the robot through the terrain (gotcha #23).

Gates:

```bash
# robot pose — expect ~[45.640, 0.021, 3.120], yaw 2.6132
# re-read once if mid-settle (z read 3.2999 before 3.1196)
gz model -m a200_0000/robot -p

# sensors — expect "Publisher count: 1" on each
for t in platform/odom sensors/imu_0/data sensors/lidar2d_0/scan sensors/gps_0/fix; do
  echo "== $t"; ros2 topic info -v /a200_0000/$t | grep -m1 "Publisher count"
done
```

Then A5 (controllers) and A6 (GUI presence).

## A5 — Fix the controllers if the spawner lost the race

park loses it in 42% of runs (gotcha #27). Nothing on this path recovers
automatically. Check:

```bash
ros2 service call /a200_0000/controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers "{}"
```

Required: `joint_state_broadcaster` and `platform_velocity_controller` both
`active` (the `ros2 control` CLI is not installed — gotcha #33). If either
is missing or inactive, re-run the spawner by hand:

```bash
ros2 run controller_manager spawner \
  joint_state_broadcaster platform_velocity_controller \
  --controller-manager /a200_0000/controller_manager \
  --switch-timeout 30
```

Then re-check with the service call above before continuing.

## A6 — Confirm the robot is in the GUI

`gz service -s /world/park/scene/info` does **not** answer this — it is served
by the server, not the GUI (gotcha #39). `/gui/screenshot` returns
`data: true` and writes no file. `/gui/move_to`'s own boolean is meaningless
(gotcha #4 family). Read the camera pose instead:

```bash
gz service -s /gui/move_to --reqtype gz.msgs.GUICamera \
  --reptype gz.msgs.Boolean --timeout 5000 --req 'name: "a200_0000/robot"'
gz service -s /gui/camera/pose --reqtype gz.msgs.Empty \
  --reptype gz.msgs.Pose --timeout 5000 --req ''
```

Required: the camera pose is near the robot's A4 pose (measured 1.46 m from
it). Negative control — repeat both with a bogus `name:`; the camera must
**not** move. If the camera does not move for `a200_0000/robot`, the GUI
missed the spawn: stop (Part C) and restart from A1.

## A7 — Stage 3: choose ONE localization path

**A7a and A7b are alternatives, not steps.** slam_toolbox and
`ekf_node_map` both publish `map -> odom`; running both puts two producers on
that edge and fails silently (gotcha #34 family). Pick one.

| Path | Use when |
|---|---|
| **A7a — nav2 + GPS** | autonomous navigation in park; the stack `NAV_PARK.md` verifies |
| **A7b — SLAM** | building a map with slam_toolbox; no GPS localization |

### A7a — nav2 + GPS localization

```bash
setsid nohup bash -c 'source /opt/ros/jazzy/setup.bash && \
  export AMENT_PREFIX_PATH=/home/thinhpham/Documents/Husky_viz/gz:$AMENT_PREFIX_PATH && \
  ros2 launch /home/thinhpham/Documents/Husky_viz/launch/park_stock.launch.py \
    world_and_robot:=false' \
  > /tmp/nav.log 2>&1 & disown
```

`world_and_robot:=false` is mandatory — the default `true` makes this file
launch the world and robot itself, and that path is unreliable (gotcha #39).

Gate:

```bash
python3 tools/check_nav2_ready.py
```

Required: `READY`, 8 lifecycle nodes active, both action servers, both
costmaps OK.

### A7b — SLAM

```bash
setsid nohup bash -c 'source /opt/ros/jazzy/setup.bash && \
  export AMENT_PREFIX_PATH=/home/thinhpham/Documents/Husky_viz/gz:$AMENT_PREFIX_PATH && \
  ros2 launch clearpath_nav2_demos slam.launch.py \
    use_sim_time:=true setup_path:=$HOME/clearpath/' \
  > /tmp/slam.log 2>&1 & disown
```

```bash
setsid nohup bash -c 'source /opt/ros/jazzy/setup.bash && \
  export AMENT_PREFIX_PATH=/home/thinhpham/Documents/Husky_viz/gz:$AMENT_PREFIX_PATH && \
  ros2 launch clearpath_viz view_navigation.launch.py \
    namespace:=a200_0000 use_sim_time:=true' \
  > /tmp/viz.log 2>&1 & disown
```

Gate:

```bash
# expect "Publisher count: 1", and map -> odom present
ros2 topic info -v /a200_0000/map | grep -m1 "Publisher count"
python3 tools/check_nav2_ready.py    # read only its "== transforms" result
```

## A8 — Checklist

There is no phase table and no `READY` verdict on this path. All must hold:

| Gate | Required | Step |
|---|---|---|
| stepping | `real_time_factor` ~1.0 | A3 |
| meshes | `0` matches in the gz log | A3 |
| GUI | `/gui/copy` listed, `MinimalScene` + render thread in the log | A3 |
| pose | ≈ `[45.640, 0.021, 3.120]` yaw `2.6132`; a large negative z means it fell through the terrain | A4 |
| sensors | `Publisher count: 1` on all four topics | A4 |
| controllers | both `active` | A5 |
| GUI has the robot | `/gui/camera/pose` moved to it; bogus name did not move it | A6 |
| navigation | A7a `READY` **or** A7b `Publisher count: 1` on `/a200_0000/map` with `map -> odom` | A7 |

Never conclude a sensor is missing from `ros2 topic list` (gotcha #38) —
only `ros2 topic info -v` on the specific topic counts.

## A9 — `sim.py status` does not apply here

`scripts/sim.py status` reads `~/.husky_sim/state.json`, which only
`sim.py start` writes. Against a stock-launched park it either finds no
state (world `?`, no launch pid → phase 2 `fail`, robot and nav gates
skipped, verdict `NOT READY`) or finds a *stale* file from an earlier
`sim.py` run and reports dead pids. Its phase-6 gate also runs
`nav_ready()` unless the recorded start was `slam: true`, and `nav_ready()`
checks nav2 lifecycle nodes that do not exist under slam_toolbox. **Do not
use `status` on a stock-launched park — use A3–A8's gates.**
---

# Part B — every other world (`sim.py`)

## B1 — Start

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

## B2 — Read the output

```
[0 clean      ] ok   killed 0, shm 0
[1 config     ] ok   default  (sensors: gps_0 imu_0 lidar2d_0 lidar3d_0)
[2 launch     ] ok   pid 41233, lake stepping after 6.8 s
[3 controllers] ok   clean            <- or: recovered  (...)
[4 robot      ] ok   pose -47.00 -14.98 3.89  4/4 topics receiving
[5 extras     ] skip default config has no compass/radio
[6 nav2       ] skip no config/nav2_lake.yaml
READY lake default
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

## B3 — Later checks

```bash
python3 scripts/sim.py status     # re-runs the gates read-only
```

Valid only for a sim this same `sim.py start` launched (see A6).

---

# Part C — Stopping (both paths)

```bash
cd ~/Documents/Husky_viz
source /opt/ros/jazzy/setup.bash
python3 scripts/sim.py stop       # must print CLEAN
```

It kills by pattern sweep and needs no state file, so it also tears down a
stock-launched park. If it prints `FAIL`, follow `CLEAN_SIM.md`.

`scripts/run_husky_sim.sh` remains as a manual entry point; do not run it
alongside `sim.py` or the Part A chain (two bridges on the same topics).
