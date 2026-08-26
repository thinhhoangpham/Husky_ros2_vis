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

The Gazebo GUI Teleop panel publishes to the same topic.
