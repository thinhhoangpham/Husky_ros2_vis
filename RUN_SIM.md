# Running the simulation

Steps only. Follow them in order, top to bottom, every time.
Background, rationale and troubleshooting live in `CLAUDE.md`.

Worlds: `park`, `lake`, `warehouse_ext`, `warehouse_ramp`, and Clearpath's
six stock worlds (`warehouse`, `construction`, `office`, `orchard`,
`pipeline`, `solar_farm`).

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
cd ~/Documents/Husky_viz
source /opt/ros/jazzy/setup.bash
setsid nohup ros2 launch launch/park_sim.launch.py world:=<world> \
  > /tmp/sim.log 2>&1 < /dev/null &
disown
```

Detaching is required, not optional — a plain `&` leaves the launch in the
caller's process group, so interrupting the invoking command sends SIGINT to
Gazebo and it exits mid-load, reading as a crash (CLAUDE.md gotcha #22).

`launch/park_sim.launch.py` is Clearpath's own `simulation.launch.py` with the
minimum deviation needed to load a custom world; `launch/gz_sim.launch.py` is
its `gz_sim.launch.py` with this repo's `worlds/` and `models/` on
`GZ_SIM_RESOURCE_PATH`. CLAUDE.md gotcha #6 lists every difference.

**Spawn pose is automatic.** park and lake carry authored poses
(`WORLD_SPAWN_POSES` in `launch/park_sim.launch.py`) and get them without any
argument — needed because Clearpath's default puts the robot at the origin at
`z=0.3`, below both terrains, where it falls out of the world (gotcha #23).
Override any element explicitly and yours wins:

```bash
ros2 launch launch/park_sim.launch.py world:=park x:=47 y:=1 z:=4.0 yaw:=3.05
```

This launcher does **not** re-apply the robot config — Step 2 is what decides
it, and it stays whatever `~/clearpath/robot.yaml` holds. That is a difference
from `scripts/run_husky_sim.sh`, which re-applies on every launch.

The alternative entry point, which also starts the compass/radio bridges the
generator does not create:

```bash
setsid nohup ~/run_husky_sim.sh <world> > /tmp/sim.log 2>&1 < /dev/null &
disown
```

It defaults to config `default`; select the full suite with
`SIM_CONFIG=full`, and override the pose with `SPAWN_X/Y/Z/YAW`. Note its GPS
bridge line is now redundant — Clearpath generates that bridge itself (gotcha
#7) — and running both would put two publishers on `sensors/gps_0/fix`.

