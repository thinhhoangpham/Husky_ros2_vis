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

