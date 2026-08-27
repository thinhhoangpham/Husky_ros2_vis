# Running the simulation

Steps only. Follow them in order, top to bottom, every time.
Background, rationale and troubleshooting live in `CLAUDE.md`.

Worlds: `park`, `lake`, `warehouse_ext`, `warehouse_ramp`.

---

## Step 1 — Go to the project

```bash
cd ~/Documents/Husky_viz
```

## Step 2 — Robot config

**Use `default` unless the task explicitly says otherwise.**

```bash
# is default already live?
diff <(grep -v '^\s*#' ~/clearpath/robot.yaml) \
     <(grep -v '^\s*#' robot_configs/robot_default.yaml) >/dev/null && echo default
```

If that printed `default`, this step is done — continue to Step 3.
If it printed nothing, apply it:

```bash
./scripts/apply_config.sh default
```

Only when the task explicitly asks for the full sensor set — note this must be
carried into Step 3 as well, or the launcher will re-apply `default` over it:

```bash
./scripts/apply_config.sh full
```

Report which config is live either way.

`run_husky_sim.sh` re-applies the config at launch, so whatever it uses is what
actually runs. It defaults to `default`; Step 3 shows how to select `full`.

## Step 3 — Launch

```bash
~/run_husky_sim.sh <world>
```

Add `true` as a second argument for RViz: `~/run_husky_sim.sh park true`

Non-interactive / agent runs must detach, or an interrupt kills Gazebo mid-load:

```bash
setsid nohup ~/run_husky_sim.sh <world> > /tmp/sim.log 2>&1 < /dev/null &
disown
```

Optional spawn-pose override (any subset):

```bash
SPAWN_X=47 SPAWN_Y=1 SPAWN_Z=4.0 SPAWN_YAW=3.05 ~/run_husky_sim.sh park
```

The launcher re-applies the robot config before spawning, defaulting to
`default`. For the full sensor set, carry it here — applying it in Step 2 alone
is not enough:

```bash
SIM_CONFIG=full ~/run_husky_sim.sh <world>
```

The launch log prints which config was applied. Check it against what Step 2
reported.

## Step 4 — Verify it came up

One-shot commands only. No sleeps, no polling loops, no wait-for-ready wrapper.
If not ready, run the same command again.

```bash
source /opt/ros/jazzy/setup.bash

ros2 topic list | grep -E "platform/odom|imu_0/data$|lidar2d_0/scan|lidar3d_0/points|gps_0/fix"
ros2 topic info -v /a200_0000/sensors/imu_0/data | grep -A1 "Publisher count"
```

Required: the topics listed, and `Publisher count: 1`.

`imu_0/data` is published straight from the Gazebo sensor, so it is
structurally blind to a dead controller spawner (CLAUDE.md gotcha #27). Also
required, every time:

```bash
ros2 service call /a200_0000/controller_manager/list_controllers controller_manager_msgs/srv/ListControllers "{}"
ros2 topic info -v /a200_0000/platform/odom
```

Required: both `joint_state_broadcaster` and `platform_velocity_controller`
state `active`, and `Publisher count: 1` on `platform/odom`. (`ros2 control`
CLI is not installed on this machine — the service call above is the working
form.)

If that gate fails, it is the controller spawner race (CLAUDE.md gotcha #27).
On `park` it is **deterministic, not intermittent**: the world blocks physics
stepping for ~14 s while it loads meshes, and the spawner's 5.00 s switch
timeout is wall-clock, so activation cannot complete. It can lose at either
controller — `joint_state_broadcaster` or `platform_velocity_controller` — not
just the second. Recovery, exercised successfully three times:

```bash
ros2 run controller_manager spawner \
  joint_state_broadcaster platform_velocity_controller \
  --controller-manager /a200_0000/controller_manager \
  --switch-timeout 30
```

## Step 5 — Verify the robot landed

```bash
gz model -m a200_0000/robot -p
```

Required: base ~0.13 m above the ground surface, roll and pitch near zero.

| World | Ground z | Expected robot z |
|---|---|---|
| `park` | 2.99 | ~3.12 |
| `lake` | 3.5–5.9 (varies with terrain) | ~0.13 above local terrain |
| `warehouse_ext` / `warehouse_ramp` | 0 | ~0.13 |

A large negative z means it spawned under the terrain and is falling.

Steps 4 and 5 read the ROS graph and the physics server only. Neither looks at
what is on screen, and the GUI can miss a model that is spawned while it is
still loading its scene — every gate green, empty park in the window. Also
required:

```bash
gz service -s /world/park/scene/info --reqtype gz.msgs.Empty \
  --reptype gz.msgs.Scene --timeout 30000 --req '' | grep -c 'a200_0000/robot'
pgrep -af "gz sim" | grep -v "bash -c" | grep -v server
```

Required: count `1`, and a GUI process alive. Use the running world's name in
the service path (CLAUDE.md gotcha #26).

If the count is `0` while Step 5 returns a valid pose, the GUI missed the
model. Do **not** kill the GUI or relaunch it with `gz sim -g` — it is a child
of the `ros2 launch` tree and detaching it crashes the session. Run a full
`CLEAN_SIM.md` + Step 3 cycle instead.

---

## Optional — drive it

```bash
ros2 topic pub -t 20 /a200_0000/cmd_vel geometry_msgs/msg/TwistStamped \
  '{twist: {linear: {x: 0.5}}}'
```

The Gazebo GUI Teleop panel publishes to the same topic. Note this publishes
straight to `cmd_vel`, downstream of the whole nav2 chain — it proves the
wheels work, it does not prove nav2 works.

---

# Autonomous navigation (park)

Steps 6 onward. Do not start these until Steps 4 and 5 have passed.

## Step 6 — Launch the navigation stack

`nav_park.launch.py` brings up GPS localization and nav2 together.

```bash
cd ~/Documents/Husky_viz
source /opt/ros/jazzy/setup.bash
setsid nohup ros2 launch launch/nav_park.launch.py > /tmp/nav_park.log 2>&1 < /dev/null &
disown
```

The command path is stock Clearpath wiring: `controller_server` ->
`cmd_vel_nav` -> `velocity_smoother` -> `cmd_vel_smoothed` ->
`collision_monitor` -> `cmd_vel`. The monitor is **in** the path; nothing
bypasses it.

## Step 6b — RViz

Launch it with the stack:

```bash
setsid nohup ros2 launch launch/nav_park.launch.py rviz:=true > /tmp/nav_park.log 2>&1 < /dev/null &
disown
```

Or attach to a stack that is already running — this exact form is the one that
works; do **not** add `-r __ns:=/a200_0000`, the config already uses absolute
topics and the namespace push breaks the TF remaps:

```bash
cd ~/Documents/Husky_viz
source /opt/ros/jazzy/setup.bash
setsid nohup rviz2 -d config/nav_park.rviz \
  --ros-args -r /tf:=/a200_0000/tf -r /tf_static:=/a200_0000/tf_static \
  -p use_sim_time:=true > /tmp/rviz.log 2>&1 < /dev/null &
disown
```

Verify it bound to the live stack rather than assuming:

```bash
pgrep -f nav_park.rviz | head -1
grep -icE "not available|error" /tmp/rviz.log
```

Required: a pid, and `0`. A nonzero count means panels or displays failed to
connect.

Ten displays, all namespaced: global costmap, local costmap, global plan, MPPI
local plan (`optimal_trajectory`), 3D lidar cloud, 2D scan, robot model, TF
(`map`/`odom`/`base_link`), grid, waypoint markers.

Notes on what you will see:

- The **local costmap follows the robot** — it is a 5 x 5 m rolling window in
  the `odom` frame, so its cells appear ahead and vanish behind as it drives.
  The global costmap is fixed and never changes (static by design since
  2026-08-27; verified `delta +0` across a 30 m drive).
- The local grid is drawn **diagonally** across the global one: it is
  axis-aligned to `odom`, which sits ~139 deg from `map` because the local EKF
  fuses the spawn-relative stock IMU yaw.
- The waypoint display stays empty — nothing in this repo or in nav2 Jazzy
  publishes `/a200_0000/waypoints`.
- `/a200_0000/optimal_trajectory` is empty until a goal is executing.

RViz costs ~60% of a core and the planner misses deadlines under load, so close
it before the timed gates in Steps 8 and 11.

To kill it, never use `pkill -f` with a pattern that matches your own command
line (CLAUDE.md gotcha #9):

```bash
for p in $(pgrep -f nav_park.rviz); do kill $p 2>/dev/null; done
```

## Step 7 — Gate on readiness

```bash
python3 tools/check_nav2_ready.py
```

Required: `READY`.

If not ready, run it again. One-shot only — no sleeps, no polling loops.

**Do not skip this and do not substitute a lifecycle check.** Nodes reporting
`active` does not mean the costmaps hold data or that TF resolves. A goal sent
too early makes the planner read the robot's pose as `(0.00, 0.00)` and fail
with `Costmap timed out`; nav2 then falls through to recovery behaviours, and
repeated `BackUp` recoveries drive the robot **north off the terrain edge**,
where it falls forever (CLAUDE.md gotcha #25). Observed three times.

## Step 8 — Gate on localization

```bash
python3 tools/check_localization_drive.py --distance 10
```

Required: `PASS`, exit code 0. It drives a 10 m straight leg with raw
`cmd_vel`, then holds a rest window, comparing the EKF `map -> base_link` pose
against Gazebo truth.

Expected result, measured on park:

| Quantity | Value |
|---|---|
| EKF vs truth, driving | 0.049 m mean / 0.112 m max |
| EKF vs truth, at rest | 0.000 m |
| heading error | 0.00 deg |

Run this before sending any goal. A localization error that survives this gate
is the one thing every later gate silently inherits.

## Step 9 — Send a goal

```bash
python3 tools/nav_goal.py X Y [YAW_DEG]        # metric, map frame
python3 tools/nav_goal_ll.py LAT LON           # latitude / longitude
```

Required: exit code 0.

`YAW_DEG` is optional and does not constrain arrival. The controller's goal
checker is `nav2_controller::PositionGoalChecker`, which ignores orientation —
heading at the goal is whatever the approach produced, not what was asked for.

Directions in park, after `heading_deg 90`: **+x is North, +y is West.** The
spawn at `x=45.64` is 4.36 m inside the **north** edge, so a goal with a
smaller x drives **south**, down the length of the park.

Terrain is x `-50 .. 50`, y `-26.55 .. 23.45`. The goal scripts reject anything
outside it before sending, but recovery behaviours are not constrained by the
keepout mask — only the planner is.

A goal inside a mapped obstacle (e.g. the water tower) or off the terrain is
refused by the planner with `NO PATH`. That is the correct result, not a
failure of this step.

## Step 10 — Confirm arrival

```bash
gz model -m a200_0000/robot -p
```

Required: within 0.5 m of the goal, and z near 3.12.

Reached goals end **exactly** on the goal: measured gap `0.000 m` on all five
route waypoints and on a 59 m far-corner goal, path length 1.01x straight line,
zero lethal cells on the planned path. A gap in the tens of centimetres is
still a pass; a metre is not.

A large negative z means it left the terrain during recoveries. Restore it
with a fresh `CLEAN_SIM.md` + Step 3 cycle — `set_pose` does not clear the
fall velocity, so teleporting a falling robot does not hold.

## Step 11 — End-to-end acceptance: the full route

The single check that exercises planning, control, localization and the
collision monitor together. Run it whenever the nav stack or its config has
changed.

```bash
python3 tools/nav_route.py routes/park_route_1.yaml
```

Required: exit code 0, all five waypoints reported reached, and no recovery
behaviours and no aborts in the run.

Expected result, measured on park:

| Quantity | Value |
|---|---|
| distance driven | 77.2 m |
| duration | 170 s |
| waypoints reached | 5 of 5 |
| closest approach per waypoint | 0.02 / 0.05 / 0.19 / 0.04 / 0.15 m |
| final pose vs waypoint 5 | 0.15 m |
| recoveries / aborts | 0 / 0 |

Confirm the end pose with Step 10's command. A run that reaches all five
waypoints but logs recoveries is a fail — it means a gate above was skipped.
