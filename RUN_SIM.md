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

Do **not** launch RViz unless you need to look at something. It costs ~60% of
a core, and the planner misses its deadlines under load.

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

## Step 8 — Send a goal

```bash
python3 tools/nav_goal.py X Y [YAW_DEG]        # metric, map frame
python3 tools/nav_goal_ll.py LAT LON           # latitude / longitude
python3 tools/nav_route.py routes/park_route_1.yaml   # recorded route
```

Required: exit code 0.

Directions in park, after `heading_deg 90`: **+x is North, +y is West.** The
spawn at `x=45.64` is 4.36 m inside the **north** edge, so a goal with a
smaller x drives **south**, down the length of the park.

Terrain is x `-50 .. 50`, y `-26.55 .. 23.45`. The goal scripts reject anything
outside it before sending, but recovery behaviours are not constrained by the
keepout mask — only the planner is.

## Step 9 — Confirm arrival

```bash
gz model -m a200_0000/robot -p
```

Required: within 0.5 m of the goal, and z near 3.12.

A large negative z means it left the terrain during recoveries. Restore it
with a fresh `CLEAN_SIM.md` + Step 3 cycle — `set_pose` does not clear the
fall velocity, so teleporting a falling robot does not hold.
