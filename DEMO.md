# Demos

Steps only. Each demo is self-contained and runs verbatim, top to bottom.
Background and rationale live in `CLAUDE.md`.

Prerequisites for every demo: `python3 scripts/sim.py start <world>` has
printed `READY` (`CLEAN_SIM.md` and `RUN_SIM.md` are both that one command now).

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

## Demo: warehouse SLAM mapping run

Builds an occupancy map of the stock `warehouse` world by driving a manual
perimeter loop under `slam_toolbox`, then saves it to `maps/`. Executed
2026-08-31; every number below is measured from that run, not a target.

**World:** `warehouse`
**Robot config:** `default`
**Spawn override:** none — Clearpath's default, settled at `(1.965, -0.044)` yaw `0.019`

**Prerequisite, and a deviation to be aware of:** this run was executed against a
sim started by hand as `ros2 launch clearpath_gz simulation.launch.py
world:=warehouse` with `slam_toolbox` launched separately, **not** through
`python3 scripts/sim.py start warehouse`. It was already running and holding a
pose graph that a restart would have discarded. Re-running this demo from a
`sim.py start warehouse` READY verdict has not been tried.

### Step 1 — Put RViz on sim time

RViz launched without `use_sim_time` drops messages with `the timestamp on the
message is earlier than all the data in the transform cache`. Stop only the
RViz launch tree — never Gazebo, never `slam_toolbox`:

```bash
pgrep -af "clearpath_viz"          # find the launch pid (parent of rviz2)
kill -INT <launch pid>
setsid nohup bash -c 'source /opt/ros/jazzy/setup.bash && exec ros2 launch \
  clearpath_viz view_navigation.launch.py namespace:=a200_0000 use_sim_time:=true' \
  > /tmp/rviz_simtime.log 2>&1 < /dev/null & disown
ros2 param get /a200_0000/rviz2 use_sim_time
```

Required: `Boolean value is: True`, and `/tmp/rviz_simtime.log` stops accruing
`transform cache` warnings after start-up. Measured: exactly one such warning
during start-up (before the tf cache filled) and none afterwards; the Map
display loaded (`Trying to create a map of size 507 x 681 using 1 swatches`).

`rviz2` segfaults on `SIGINT` and Ubuntu's apport dumps core for ~10 s before
the pid disappears — cosmetic, same family as CLAUDE.md gotcha #8. Confirm the
pid is really gone before relaunching.

### Step 2 — Drive the perimeter loop

One segment per invocation of `tools/drive_segment.py`:

```bash
python3 -m tools.drive_segment forward 3.0
python3 -m tools.drive_segment turn_left 1.5708
python3 -m tools.drive_segment reverse 1.0
```

Each segment is one bounded `TwistStamped` publication on `/a200_0000/cmd_vel`
(CLAUDE.md gotcha #3), closed-loop on `platform/odom/filtered` distance or
accumulated yaw, with `/a200_0000/sensors/lidar2d_0/scan` checked every cycle.
Parameters for this run: linear `0.5 m/s`, angular `0.4 rad/s` — the tool's
defaults. No collision monitor runs in this configuration, so the clearance
abort is the only obstacle protection. Between segments the scan sector minima
decide the next heading — turn toward the open sector before the forward
clearance reaches the abort threshold.

The clearance rule is **direction-aware** (`tools/drive_geometry.py`), which is
a deliberate change from the flat 1.0 m whole-scan rule this run was driven
with — see the notes after the table:

| mode | region evaluated | threshold |
|---|---|---|
| `forward` / `reverse` | the swept corridor: lateral offset within the 0.34 m half-width + 0.10 m margin, on the side being driven toward; distance measured **along track** | 1.00 m |
| `turn_left` / `turn_right` | the whole scan — an in-place turn sweeps the circumscribed circle in every direction | 0.70 m (rotation radius `hypot(0.495, 0.34)` = 0.600 m + 0.10 m) |
| any | the whole scan, contact imminent | 0.35 m, vetoes all motion |

The route driven, as `(segment -> resulting odom pose)`:

| # | segment | end odom pose | min range seen |
|---|---|---|---|
| 1 | forward 3.0 m | `(5.09, 0.00)` yaw `0.004` | 1.92 |
| 2 | turn left 1.5708 rad | `(5.13, 0.00)` yaw `1.642` | 2.08 |
| 3 | forward 4.0 m | `(4.70, 4.07)` yaw `1.675` | 2.77 |
| 4 | forward 2.5 m | `(4.43, 6.68)` yaw `1.675` | 1.10 |
| 5 | turn left 1.5708 rad | `(4.43, 6.72)` yaw `-2.970` | 1.44 |
| 6 | forward 2.0 m | `(2.37, 6.34)` yaw `-2.957` | 1.59 |
| 7 | forward 3.0 m | `(-0.71, 5.76)` yaw `-2.957` | 1.71 |
| 8 | forward 4.0 m | `(-4.78, 5.00)` yaw `-2.957` | 2.07 |
| 9 | forward 3.0 m | `(-7.86, 4.42)` yaw `-2.957` | 1.42 |
| 10 | turn left 1.5708 rad | `(-7.91, 4.42)` yaw `-1.320` | 2.63 |
| 11 | forward 5.0 m | **ABORTED at 4.31 m**, `(-6.75, 0.17)` | **1.00** |
| 12 | reverse 1.0 m (recovery) | `(-7.02, 1.18)` | 1.80 |
| 13 | turn right 1.5708 rad | `(-7.03, 1.22)` yaw `-2.935` | 2.23 |
| 14 | turn left 2.079 rad | `(-7.03, 1.22)` yaw `-0.047` | 2.22 |
| 15 | forward 5.0 m | `(-1.97, 0.91)` yaw `-0.060` | 2.76 |
| 16 | forward 4.0 m | `(2.16, 0.65)` yaw `-0.059` | 1.10 |

Roughly 35 m driven. Required: no contact with any obstacle, and the run ends
near the start point. Measured: **no collision and no stall** — the abort at
segment 11 stopped the robot with an obstacle 0.85 m off the front-right,
correctly, before contact. Final Gazebo truth pose `(1.655, 0.003)` against the
start `(1.965, -0.044)` — **0.31 m from the start point**, so `slam_toolbox`
gets its loop closure. Odom read `(2.16, 0.65)` at the same instant, i.e. ~0.65 m
of accumulated wheel-odometry drift over the loop, which the map frame absorbs.

Three things this run exposed. The first two were bugs in the driver and are
fixed in `tools/drive_geometry.py`; the table above is the record of the run as
driven, so segments 12-14 are artefacts of the old behaviour and a repeat will
not need them.

- **The flat 1.0 m abort rule strands the robot — fixed.** Once the minimum
  range over the whole scan was below 1.0 m, every subsequent segment aborted
  instantly at its first scan check, *including the reverse or turn that would
  escape*. Segment 12 is a deliberate deviation: a hand-flown reverse at a
  lowered 0.5 m threshold, purely to get clear again. The rule is now
  direction-aware — only the region the robot is about to sweep is evaluated —
  so the 0.85 m front-right obstacle that fired segment 11's abort vetoes
  driving forward while still permitting reverse and either turn. The
  omnidirectional veto that remains is 0.35 m, at which being stranded is the
  correct outcome.
- **Turns of `pi` or more do not terminate — fixed.** Progress was measured as
  a shortest-angle difference, which wraps: a commanded 3.1416 rad turn read
  1.31 rad of progress, timed out, and left the robot at yaw 2.079 having
  actually rotated ~5.0 rad. Segments 13-14 are the correction that was needed
  at the time. Progress is now the **unwrapped** accumulated per-sample heading
  change, so a turn of exactly `pi`, more than `pi`, or several revolutions all
  terminate at the commanded angle.
- **The map does not repaint after a pure rotation** — segments 2, 5 and 10 left
  the known-cell count unchanged. Not a bug; expected of a 360 deg scanner.

### Step 3 — Map growth checkpoints

```bash
python3 -m tools.check_map_growth
```

It samples `/a200_0000/map` with **transient-local, reliable** QoS — the latched
map is not readable with sensor-data QoS and reads as a dead topic — and prints
grid size, resolution, origin, occupied/free/unknown counts and the known-cell
bounding box in metres. Known cells are those with value >= 0.

| checkpoint | known cells | occ / free / unk | bbox |
|---|---|---|---|
| pre-drive baseline (stationary) | 47 715 | 696 / 47 019 / 297 552 | 25.30 x 31.50 m |
| after segment 1 | 93 513 | 1 529 / 91 984 / 251 754 | 25.30 x 34.00 m |
| after segment 3 | 139 855 | 1 999 / 137 856 / 268 064 | 29.95 x 34.00 m |
| after segment 9 | 202 239 | 4 161 / 198 078 / 335 949 | 30.10 x 37.00 m |
| after segment 11 (abort) | 212 160 | 4 678 / 207 482 / 357 318 | 30.05 x 37.15 m |
| after segment 15 | 219 934 | 5 519 / 214 415 / 358 484 | 31.85 x 38.70 m |
| after segment 16 (final) | 221 736 | 5 341 / 216 395 / 356 434 | 31.80 x 37.20 m |

Stopping rule for this run: stop when the known-cell count grows by less than 5%
over two consecutive segments. Measured: +2.9% then +0.8% at segments 15 and 16,
so driving stopped there. Note the last row's occupied count *falls* (5 519 ->
5 341) and the bbox shrinks — that is the loop closure re-optimising the graph
and retracting spurious cells, which is the desired outcome, not a loss.

### Step 4 — Save the map

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /home/thinhpham/Documents/Husky_viz/maps/warehouse_slam_map \
  --ros-args -r map:=/a200_0000/map
```

Required: `Map saved successfully`, and both files present in `maps/` (project
convention — not a home directory). Measured:

```
image: warehouse_slam_map.pgm
mode: trinary
resolution: 0.050
origin: [-17.265, -19.794, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

646 x 895 px @ 0.05 m/pix. Saved-image histogram, which must match the live
sample above exactly: **occupied (0) 5 341, free (254) 216 395, unknown (205)
356 434**.

### Step 5 — Localize on the saved map and navigate to a goal

Brings up AMCL against the map saved in Step 4, then the nav2 stack, then sends
one goal. Both launches need `setup_path` — its default `/etc/clearpath/` does
not exist on this machine and the launch fails on it.

**Ordering is mandatory: stop `slam_toolbox` FIRST, and verify it stopped.**
`slam_toolbox` and AMCL both publish `map -> odom`. Two publishers on one
transform do not error, do not warn, and are not rejected — the transform simply
flickers between two answers at whatever rate each publishes, and every
consumer (costmaps, the controller, RViz) silently reads a pose that jumps.
This is the failure family of CLAUDE.md gotcha #7: a second publisher on a topic
the stack already owns, producing wrong behaviour with a clean log.

```bash
pgrep -af "slam_toolbox"                 # find the launch pid (parent of the node)
kill -INT <launch pid>
pgrep -af "slam_toolbox"                 # required: prints nothing
ros2 topic info -v /a200_0000/tf | grep -c "map"   # map -> odom must have stopped
```

Do not stop Gazebo and do not stop RViz.

```bash
setsid nohup bash -c 'source /opt/ros/jazzy/setup.bash && exec ros2 launch \
  clearpath_nav2_demos localization.launch.py use_sim_time:=true \
  setup_path:=/home/thinhpham/clearpath/ \
  map:=/home/thinhpham/Documents/Husky_viz/maps/warehouse_slam_map.yaml' \
  > /tmp/localization.log 2>&1 < /dev/null & disown

setsid nohup bash -c 'source /opt/ros/jazzy/setup.bash && exec ros2 launch \
  /home/thinhpham/Documents/Husky_viz/launch/nav2_warehouse.launch.py \
  use_sim_time:=true setup_path:=/home/thinhpham/clearpath/' \
  > /tmp/nav2.log 2>&1 < /dev/null & disown
```

**Use the repo's `launch/nav2_warehouse.launch.py`, not
`clearpath_nav2_demos nav2.launch.py`.** The stock a200 config
(`.../config/a200/nav2.yaml:181-183`) sets `rolling_window: true`, `width: 20`,
`height: 20` on the **global** costmap, so it is a 20 x 20 m window that follows
the robot rather than a costmap fixed in `map`. Measured live on
`/a200_0000/global_costmap/costmap`: `333 x 333 @ 0.06 m = 20.0 x 20.0 m`,
origin `(-6.66, -19.26)`, moving with the robot — while the saved map is
`646 x 895 @ 0.05 m = 32.3 x 44.75 m`. A goal outside that window is simply not
in the costmap, so the planner cannot route to it and navigation only reaches
goals within ~10 m. (`allow_unknown: true` is already set, so unmapped space is
not the blocker.) The stock launch file hardcodes its parameter file and offers
no override, hence the fork; `config/nav2_warehouse.yaml` documents its three
deviations in its header.

`setsid nohup ... & disown` is required, not decorative — a plain `nohup ... &`
stays in the caller's process group and an interrupt reaches Gazebo (gotcha #22).

**Seed AMCL's initial pose.** AMCL starts with no estimate and will not converge
from a driven map on its own; it comes up healthy and useless, the same shape as
gotcha #34. Read the robot's true pose from Gazebo and publish it once on
`/a200_0000/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`, frame
`map`; `-t 1` exits cleanly, unlike killing a `ros2` CLI under `timeout` —
gotcha #8):

```bash
gz model -m a200_0000/robot -p          # truth pose, for the values below
ros2 topic pub -t 1 /a200_0000/initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: "map"},
    pose: {pose: {position: {x: 1.655150, y: 0.002513, z: 0.0},
                  orientation: {z: -0.130368, w: 0.991467}},
           covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0,
                        0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}'
```

The orientation above is yaw `-0.261389` rad as a quaternion. Substitute the
truth pose your own run reports; these are the values from 2026-08-31.

**Gate 1 — AMCL is publishing `map -> odom`.**

```bash
ros2 topic info -v /a200_0000/amcl_pose | grep -A1 "Publisher count"
```

Required: `Publisher count: 1`. A topic listing alone proves nothing here
(gotcha #38).

**Gate 2 — AMCL agrees with truth at rest.**

```bash
python3 -m tools.check_amcl_error --world warehouse
```

Measured: **0.017 m** position error at rest. The map frame coincides with the
Gazebo world frame in this run because the map was built from this spawn and
AMCL was seeded from truth, which is what makes the direct comparison valid.

**Gate 3 — AMCL holds up while driving.**

```bash
python3 -m tools.drive_segment forward 3.0
python3 -m tools.drive_segment forward 3.0
python3 -m tools.check_amcl_error --world warehouse
```

Measured after a **6.73 m** run: position error **0.059 m**, yaw error grown to
**10.4 deg**. The position error staying small while yaw degrades is the
expected signature of a particle filter on a corridor-like map — it is well
constrained across the corridor and poorly constrained in heading.

**Gate 4 — nav2 drives to a goal.**

```bash
python3 -m tools.send_nav_goal -5.00 1.00
```

Required: `accepted: yes`, result `SUCCEEDED`, and a final position error inside
nav2's `xy_goal_tolerance`. Measured: **SUCCEEDED in 31.9 s**, final position
error **0.102 m**, and the minimum lidar range seen during the run **1.00 m** —
i.e. it kept a metre of clearance from the shelving it passed.

A rejected goal or an immediate `ABORTED` almost always means `map -> odom` is
absent or contested — go back and re-check that `slam_toolbox` is really gone.

### Cleanup

None during Steps 1-4 — the sim, `slam_toolbox` and RViz are left running
deliberately, since the pose graph is only in memory and a restart discards it.
Once Step 4 has saved the map that no longer applies, and Step 5 replaces
`slam_toolbox` with AMCL. Tear down with `CLEAN_SIM.md` when finished.

### Provenance of the numbers

Steps 2 and 3 were originally executed by two throwaway scratchpad scripts. They
now live in `tools/drive_segment.py` (with `tools/drive_geometry.py` holding its
ROS-free decision logic, unit-tested in `tests/test_drive_geometry.py`) and
`tools/check_map_growth.py`. Every number in the tables above is measured from
the original run under the *old* flat clearance rule and the wrapped turn
progress, so segments 11-14 in particular will not reproduce identically — the
totals (~35 m driven, 221 736 known cells, 0.31 m from the start) are what to
compare against.

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
