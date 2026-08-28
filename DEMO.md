# Demos

Steps only. Each demo is self-contained and runs verbatim, top to bottom.
Background and rationale live in `CLAUDE.md`.

Prerequisites for every demo: `CLEAN_SIM.md` has reported clean, and
`RUN_SIM.md` has completed through Step 5.

---

## Demo: uphill traction on lake

Proves the robot climbs a grade without sliding sideways off its heading.
Fails if `urdf/wheel_slip.urdf.xacro` is not reaching the robot.

**World:** `lake`
**Robot config:** `default`
**Spawn override:** none — the demo teleports to the test start instead

### Step 1 — Place the robot at the bottom of the longest climb

The dip at `(32, -16)` begins the longest sustained monotonic rise in the world:
20 m of travel in `-x` for 1.79 m of gain, a 5.1° mean grade. Yaw `π` faces it.

```bash
source /opt/ros/jazzy/setup.bash
gz service -s /world/lake/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 10000 \
  --req 'name: "a200_0000/robot" position { x: 32 y: -16 z: 4.36 } orientation { x: 0 y: 0 z: 1 w: 0 }'
gz model -m a200_0000/robot -p
```

Required: reported position within 0.1 m of `x=32, y=-16`, yaw ≈ `±3.14`.

`set_pose` returns `data: true` even for a nonexistent entity, so the
`gz model` read is the actual check — not the service response.

### Step 2 — Drive uphill and sample

```bash
ros2 topic pub -r 20 -t 1600 /a200_0000/cmd_vel \
  geometry_msgs/msg/TwistStamped '{twist: {linear: {x: 1.0}}}' >/dev/null 2>&1 &
PUB=$!

for i in 1 2 3 4; do
  gz model -m a200_0000/robot -p 2>/dev/null | grep -A1 "Pose \[ XYZ" | tail -1 | tr -s ' '
done

kill $PUB 2>/dev/null    # stop driving as soon as sampling ends
```

The publisher outlives the four samples by a wide margin, so it must be stopped
explicitly — otherwise the robot keeps climbing for another minute after the
measurement window and whatever runs next inherits an unpredictable pose.

Capture the PID rather than reaching for `pkill`: a `pkill -f` pattern matching
`cmd_vel` also matches the shell running it (CLAUDE.md gotcha #9).

Required, comparing the first and last samples:

| quantity | required | meaning |
|---|---|---|
| `x` decreases | by > 2 m | it is moving, and uphill |
| `z` increases | by > 0.1 m | it is climbing, not stalling |
| **`y` drift** | **< 0.05 m per metre of `x` travelled** | it is not sliding sideways |

The drift figure is the point of the demo. Reference measurements on this
grade: **0.0045 m/m** with the wheel-slip override, **0.198 m/m** on Clearpath's
stock settings — a 44× difference, so the 0.05 threshold separates them
unambiguously rather than finely.

### Cleanup

Step 2 stops the drive. The robot is left partway up the slope — fine to leave,
since the demo teleports to a known start rather than assuming a pose. Tear the
sim down with `CLEAN_SIM.md` when finished with it.

### If it fails

A drift near 0.2 m/m means `urdf/wheel_slip.urdf.xacro` is not reaching the
robot. Check that the live config's `platform.extras.urdf` points at it —
`robot_default.yaml` and `robot_full.yaml` reach it by different paths. See
CLAUDE.md gotcha #24.

---

## Demo: local costmap avoids an unmapped box (park)

Proves that nav2 avoids a solid obstacle the prior map knows nothing about —
one injected into the world at runtime, while the robot is already driving.
Both layers of avoidance are exercised: the global costmap's obstacle layer
marks the box from live lidar and the planner replans around it, while the
local costmap and MPPI handle the close-in reaction.

**World:** `park`
**Robot config:** `default`
**Spawn override:** none — the demo starts from `NAV_PARK.md`'s end state

**Prerequisite:** `NAV_PARK.md` complete and passing, with the robot parked at
route waypoint 5, `≈(-30.80, -3.45)`. That is the end state of `NAV_PARK.md`
Step 10, so the standing practice of a full `CLEAN_SIM.md` → `RUN_SIM.md` →
`NAV_PARK.md` cycle before a test still applies unchanged — this demo continues
that cycle rather than replacing it.

Directions in park: **+x is North, +y is West** (CLAUDE.md gotcha #32).

### Step 1 — Confirm the start pose

```bash
source /opt/ros/jazzy/setup.bash
gz model -m a200_0000/robot -p
```

Required: `x ≈ -30.80`, `y ≈ -3.45`, and `z` near `3.12`. A large negative `z`
means the robot left the terrain (CLAUDE.md gotcha #25) and the run is void —
restart from `CLEAN_SIM.md`.

### Step 2 — Send the return goal north

The box has to be spawned mid-drive, so the goal must not hold the shell.

```bash
cd ~/Documents/Husky_viz
python3 tools/nav_goal.py 27.12 1.10 > /tmp/nav_goal_box.log 2>&1 &
NAV=$!
```

Required: `/tmp/nav_goal_box.log` shows `goal accepted, navigating...` and
`gz model -m a200_0000/robot -p` reports `x` increasing.

### Step 3 — Spawn the box while the robot is driving

Watch `x` with one-shot reads — no sleeps, no polling loop (CLAUDE.md,
Workflow). When `x` reaches roughly `0`, about 14 m short of the box, spawn it:

```bash
gz model -m a200_0000/robot -p        # repeat as one-shot reads until x ≈ 0
python3 tools/spawn_obstacle.py 14 -0.66
```

`y = -0.66`, not `0`: the wp2↔wp3 leg runs `(27.12, 1.10) → (1.16, -2.39)` and
is not axis-aligned, so at `x = 14` the path is actually at `y = -0.66`. A box
centred on `y = 0` would clip the robot's swept width by only ~0.18 m instead
of blocking it head-on, and the robot would pass without ever having to detour.

Required: the tool prints `PASS  'test_obstacle' present in /world/park scene`.
The `/world/park/create` reply is not the check — it reports success for a no-op
(CLAUDE.md gotcha #4), which is why the tool re-reads `scene/info` itself.

Two ways this step fails on a first execution, both before the box exists:

- **Pre-spawn verification fails.** The tool first requires `test_obstacle` to
  be *absent*, and reads `/world/park/scene/info` to decide. It reports `FAIL`
  both when the scene service does not answer in time and when the name is
  already taken by a leftover box from an earlier run. Remove the leftover
  before re-running (Step 6); if the service simply did not answer, re-run the
  command — the drive is still in progress and `x` will have moved on, so
  re-check `x` before spawning.
- **The create call times out.** The whole box SDF is passed as one escaped
  protobuf text string through `bash -lc` against a 5 s service timeout, so a
  busy sim can leave the reply empty. The scene re-check then reports `FAIL`.
  Confirm the box really is absent, then re-run.

### Step 4 — Measure the clearance as it passes

```bash
gz model -m a200_0000/robot -p        # one-shot reads through the pass
```

Take the minimum centre-to-centre distance from the sampled poses to
`(14, -0.66)`.

Required: **minimum distance > 0.84 m** — the robot's 0.34 m half-width plus the
box's 0.5 m half-width. At or below 0.84 m the two are in contact.

### Step 5 — Confirm arrival

```bash
gz model -m a200_0000/robot -p
cat /tmp/nav_goal_box.log
```

Required: final pose within the goal tolerance of `(27.12, 1.10)`, `z` still
near `3.12`, and `z` near `3.12` at every sample taken along the way.

### Step 6 — Cleanup

```bash
python3 tools/spawn_obstacle.py --remove
wait $NAV 2>/dev/null
```

`--remove` takes no `X Y`; it deletes `--name` (default `test_obstacle`) from
`--world` (default `park`). Spell out both if either default was changed at
spawn time: `python3 tools/spawn_obstacle.py --remove --name test_obstacle
--world park`.

Required: `PASS  'test_obstacle' absent from /world/park scene`. A box left
behind blocks the next run's pre-spawn verification (Step 3).

### Results

| quantity | required | measured |
|---|---|---|
| box present in `/world/park/scene/info` after spawn | yes | not yet recorded |
| box marked in the global costmap once in lidar range | yes | not yet recorded |
| global plan bends around the box after it is marked | yes | not yet recorded |
| min robot-to-box centre distance | > 0.84 m | not yet recorded |
| goal `(27.12, 1.10)` reached | yes | not yet recorded |
| `z` throughout | ≈ 3.12 | not yet recorded |

This demo has not been executed yet, so the right-hand column is thresholds
awaiting data, not results.

### What the global plan should do

Since 2026-08-27 park's **global** costmap carries a live `obstacle_layer` on
`/a200_0000/sensors/lidar2d_0/scan`, matching stock nav2 and Clearpath
(`config/nav2_park.yaml`, CLAUDE.md gotcha #36) — this reversed a brief
same-day experiment that made the global costmap static. Both costmaps now see
the box.

So the expected picture in RViz is: the box appears as marked cells in the
global costmap within lidar range, and the global plan **bends around it**.
Watch for the plan to update on the next planner cycle after the box is first
marked; the plan drawn before that is simply stale and not a fault.

A global plan still running through the box *after* the box is visible in the
global costmap is now a symptom worth investigating — check that the obstacle
layer's topic is the absolute form (gotcha #36) and that the planner is
actually replanning. A robot that drives into the box remains the hard
failure.

### If it fails

Distinguish an avoidance failure from a stall before diagnosing either:

- **Avoidance failure** — the robot keeps moving, closes to ≤ 0.84 m, and
  contacts or climbs the box. Displacement per `cmd_vel` sample stays normal.
  The costmaps or their lidar source is the place to look (gotcha #36); check
  both the local `voxel_layer` and the global `obstacle_layer`.
- **Planner-only failure** — the box is marked in the local costmap and the
  robot dodges it, but the global plan never bends. Only the global
  `obstacle_layer` is at fault; the run is not void, but it is a regression.
- **Stall** — the robot stops making progress with `cmd_vel` still flowing.
  Check for many consecutive `cmd_vel` messages against under ~1 cm of
  displacement per pose sample; that signature is CLAUDE.md gotcha #24, rigid
  wheel-slip compliance making a tight avoidance turn physically unexecutable,
  and it is not an avoidance failure at all. The discriminator is displacement
  per commanded sample, not distance to the box.

If the goal is never accepted, nav2 came up without `map -> odom`
(gotcha #34) — re-run `tools/check_nav2_ready.py` and redo `NAV_PARK.md`.

---

## Adding a demo

Copy the block below and fill it in. Keep it executable: commands only, with
required results stated so a step can pass or fail rather than be judged.

```markdown
## Demo: <short name>

**World:** <park | lake | warehouse_ext | warehouse_ramp>
**Robot config:** <default | full>
**Spawn override:** <SPAWN_X=… SPAWN_Y=… SPAWN_Z=… SPAWN_YAW=… | none>

### Step 1 — <what it does>

​```bash
<command>
​```

Required: <the observable result that decides pass or fail>

### Step 2 — <…>

​```bash
<command>
​```

Required: <…>

### Cleanup

<anything to restore, or "none — CLEAN_SIM.md handles it">
```

Two things that make a demo reproducible rather than merely repeatable:

- **State the required result, not the intent.** "Robot reaches the far bench"
  cannot be checked; "`gz model -m a200_0000/robot -p` reports x < 20" can.
- **Name the world and config explicitly.** A demo that silently assumes the
  previous demo's state breaks the moment it is run first, and that failure is
  hard to attribute.

If a demo needs a spawn pose other than the world default, put the `SPAWN_*`
override in the demo's own launch step rather than editing `RUN_SIM.md` — the
runbook's defaults are the authored poses and should stay that way.
