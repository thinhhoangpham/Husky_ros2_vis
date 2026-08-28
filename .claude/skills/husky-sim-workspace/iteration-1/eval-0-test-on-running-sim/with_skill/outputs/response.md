# Plan — unmapped-box avoidance in park (DRY RUN, nothing executed)

## Bottom line first

**I cannot run this test against the sim that is currently up, and I am stopping
to ask before anything is executed.**

The test you are asking for exists as a runbook block: `DEMO.md` → *"Demo: local
costmap avoids an unmapped box (park)"*. Read verbatim, it does **not** start
from "a park sim that happens to be running". It states:

> **Prerequisite:** `NAV_PARK.md` complete and passing, with the robot parked at
> route waypoint 5, `≈(-30.80, -3.45)`. That is the end state of `NAV_PARK.md`
> Step 10, so the standing practice of a full `CLEAN_SIM.md` → `RUN_SIM.md` →
> `NAV_PARK.md` cycle before a test still applies unchanged — this demo continues
> that cycle rather than replacing it.

And the skill's own rule: *"No test begins without proof the machine is clean"* —
a `CLEAN_SIM.md` Step 3 report from **this** attempt, not a belief that the
running sim is healthy. Plus: *"Between two tests, return to Phase 1. A second
test on a world that already has a spawned object, a robot 20 m off its spawn
pose, or an aborted goal in its history is not the same test."*

So the honest answer is: the running sim must be torn down and the full cycle
re-run. That is your call, not mine.

### Decision I need from you

1. **Tear down the running sim and run the full cycle** (CLEAN_SIM → RUN_SIM →
   NAV_PARK → DEMO box block), ~4–6 minutes of nav route driving before the box
   step even starts. This is the only path that produces a reportable closest
   approach. **← recommended**
2. **Do not touch the running sim.** Then no measurement is possible and I report
   nothing.

Two further things I will not decide for you if you pick (1):

- **`DEMO.md` Step 3's box position is `(14, -0.66)`, on the wp2↔wp3 leg** — it
  is not "in front of the robot wherever it currently is". If you want the box
  somewhere else, that is a different test and needs a new `DEMO.md` block
  written first; I will not improvise coordinates in chat.
- **`NAV_PARK.md` Step 9 (`tools/check_local_avoidance.py`) also reports a
  closest approach**, but against a *small tree the prior map omits*, not a box,
  with a `> 1.89 m` gate. If a closest-approach number is all you want, that is
  the cheaper check. Tell me which one you mean.

Also worth flagging before you choose: `DEMO.md`'s results table for the box demo
reads *"not yet recorded"* in every row — **this demo has never been executed**.
Its thresholds are authored, not measured. First run may surface a runbook bug,
in which case I stop and propose the edit rather than working around it.

---

## The plan, in full, if you approve option (1)

Runbooks needed: `CLEAN_SIM.md` (4 steps), `RUN_SIM.md` (11 steps), `NAV_PARK.md`
(10 steps), `DEMO.md` box block (6 steps). All four read in full already.

### Phase 1 — `CLEAN_SIM.md`

**Step 1 — Go to the project**
```bash
cd ~/Documents/Husky_viz
```

**Step 2 — Kill and clear** (all four blocks, in this order; shm cleared *after*
processes are dead)
```bash
./scripts/kill_sim.sh
```
```bash
for pid in $(pgrep -f "a200_0000|gz sim|gz_tools_vendor" 2>/dev/null); do
  grep -qa "bash -c" /proc/$pid/cmdline 2>/dev/null && continue
  kill -9 $pid 2>/dev/null
done
```
```bash
source /opt/ros/jazzy/setup.bash && ros2 daemon stop 2>/dev/null || true
```
```bash
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
```
I ignore whatever `kill_sim.sh` prints about being clean — it verifies with the
same list it kills with and has printed "clean" with 73 processes alive.

**Step 3 — Verify** *(GATE)*
```bash
ps -eo pid,cmd --no-headers | grep -viE "grep|bash -c" \
  | grep -iE "ros|gazebo|gz |a200|husky|clearpath|rviz"
echo "opt/ros : $(ps -eo cmd --no-headers | grep -c '^/opt/ros')"
echo "shm     : $(ls /dev/shm | grep -c fastrtps)"
```
Must show: **no process lines**, **`opt/ros : 0`**, **`shm : 0`**.
A nonzero `shm` on the first read → run the exact same block a second time (no
sleep). Drops to 0 = transient, continue. Stays nonzero = real leak → Step 4.
I will **not** `rm` a second time to force it down.

**Step 4 — only if survivors remain**
```bash
ps -eo pid,etime,cmd --no-headers | grep -viE "grep|bash -c" \
  | grep -iE "ros|gazebo|gz |a200|husky|clearpath|rviz"
```
A survivor is the finding, not a nuisance: I report which node type leaked, since
it means `CLEAN_SIM.md`'s sweep pattern needs a change — and that edit is yours
to approve.

### Phase 2 — `RUN_SIM.md` Steps 1–5

**Step 2 — Robot config.** Task does not ask for `full`, so `default`.
```bash
diff <(grep -v '^\s*#' ~/clearpath/robot.yaml) \
     <(grep -v '^\s*#' robot_configs/robot_default.yaml) >/dev/null && echo default
```
Prints `default` → done. Prints nothing → `./scripts/apply_config.sh default`.

**Step 3 — Launch.** Detached form (agent run), no RViz second argument — passing
`true` makes Gazebo fail to render the robot while every gate still passes.
```bash
setsid nohup ~/run_husky_sim.sh park > /tmp/sim.log 2>&1 < /dev/null &
disown
```

**Step 4 — Verify it came up** *(GATE)*. One-shot only; if not ready I re-run the
same command, never sleep or loop.
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep -E "platform/odom|imu_0/data$|lidar2d_0/scan|lidar3d_0/points|gps_0/fix"
ros2 topic info -v /a200_0000/sensors/imu_0/data | grep -A1 "Publisher count"
```
Must show: all five topics, `Publisher count: 1`.
```bash
ros2 service call /a200_0000/controller_manager/list_controllers controller_manager_msgs/srv/ListControllers "{}"
ros2 topic info -v /a200_0000/platform/odom
```
Must show: `joint_state_broadcaster` **active** and `platform_velocity_controller`
**active**, and `Publisher count: 1` on `platform/odom`. On park the spawner race
is *deterministic* (~14 s mesh load vs 5.00 s wall-clock switch timeout), so I
expect to need the runbook's recovery:
```bash
ros2 run controller_manager spawner \
  joint_state_broadcaster platform_velocity_controller \
  --controller-manager /a200_0000/controller_manager --switch-timeout 30
```

**Step 5 — Verify the robot landed** *(GATE)*
```bash
gz model -m a200_0000/robot -p
```
Must show: **z ≈ 3.12** (park ground 2.99 + 0.13), roll/pitch near zero. A large
negative z = spawned under terrain, falling.
```bash
gz service -s /world/park/scene/info --reqtype gz.msgs.Empty \
  --reptype gz.msgs.Scene --timeout 30000 --req '' | grep -c 'a200_0000/robot'
pgrep -af "gz sim" | grep -v "bash -c" | grep -v server
```
Must show: count **`1`**, and a GUI process alive. Count `0` with a valid pose =
GUI missed the model → full CLEAN_SIM + relaunch, never `gz sim -g`.

### Phase 3 — `NAV_PARK.md` Steps 3–10

**Step 3 — nav stack**
```bash
cd /home/thinhpham/Documents/Husky_viz
source /opt/ros/jazzy/setup.bash
setsid nohup ros2 launch launch/nav_park.launch.py > /tmp/nav_park.log 2>&1 &
disown
```
No RViz — `RUN_SIM.md` Step 6b notes it costs ~60% of a core and the planner
misses deadlines under load; the timed gates want it closed.

**Step 4 — readiness** *(GATE)*
```bash
python3 tools/check_nav2_ready.py
```
Must print **`READY`**. Not ready → run it again, one-shot. Never skipped and
never substituted with a lifecycle check: a goal sent early makes the planner
read the pose as `(0.00, 0.00)`, and the resulting repeated `BackUp` recoveries
drive the robot north off the terrain edge (observed three times).

**Step 5 — prior map** *(GATE)*
```bash
python3 tools/check_map_alignment.py
```
Must print **`PASS`**.

**Step 6 — localization** *(GATE)*
```bash
python3 tools/check_localization_drive.py --distance 10
```
Must print **`PASS`**, exit 0. Expected: 0.049 m mean / 0.112 m max driving,
0.000 m at rest, 0.00 deg heading error.

**Steps 7–8** — skipped, conditionally: Step 10's route subsumes a single goal and
the demo needs the route's end state, not an arbitrary goal.

**Step 10 — full route** *(GATE, and the thing that produces the demo's start
pose)*
```bash
python3 tools/nav_route.py routes/park_route_1.yaml
```
Must show: exit 0, **5 of 5 waypoints reached**, **0 recoveries, 0 aborts**.
Expected 77.2 m over 170 s; per-waypoint closest approach
0.02 / 0.05 / 0.19 / 0.04 / 0.15 m; final pose 0.15 m from wp5. Waypoints reached
*with* recoveries logged is a **fail** — it means a gate above was skipped.

### Phase 4 — `DEMO.md`, "local costmap avoids an unmapped box (park)"

**Step 1 — Confirm the start pose** *(GATE)*
```bash
source /opt/ros/jazzy/setup.bash
gz model -m a200_0000/robot -p
```
Must show: **x ≈ -30.80, y ≈ -3.45, z ≈ 3.12**. Large negative z → run void,
restart from `CLEAN_SIM.md`.

**Step 2 — Send the return goal north** (backgrounded; the box must be spawned
mid-drive, so the goal must not hold the shell)
```bash
cd ~/Documents/Husky_viz
python3 tools/nav_goal.py 27.12 1.10 > /tmp/nav_goal_box.log 2>&1 &
NAV=$!
```
Must show: `goal accepted, navigating...` in the log, and `x` increasing.

**Step 3 — Spawn the box while driving** — one-shot pose reads only, no loop, no
sleep; when `x` reaches roughly `0` (about 14 m short of the box):
```bash
gz model -m a200_0000/robot -p        # repeat as one-shot reads until x ≈ 0
python3 tools/spawn_obstacle.py 14 -0.66
```
Must print **`PASS  'test_obstacle' present in /world/park scene`**. The
`/world/park/create` reply is *not* the check — it reports success for a no-op
(gotcha #4), which is why the tool re-reads `scene/info`.
`y = -0.66`, not `0`: the wp2↔wp3 leg runs `(27.12, 1.10) → (1.16, -2.39)` and is
not axis-aligned, so a box on `y = 0` would clip only ~0.18 m of swept width and
the robot would pass without detouring.
Two documented first-run failure modes, both *before* the box exists: pre-spawn
verification fails (scene service slow, or a leftover box — remove via Step 6,
re-check `x`, re-run), or the create call times out on a busy sim (confirm the box
is genuinely absent, then re-run).

**Step 4 — Measure the clearance** *(this is your requested number)*
```bash
gz model -m a200_0000/robot -p        # one-shot reads through the pass
```
I take the **minimum centre-to-centre distance from the sampled poses to
`(14, -0.66)`**.
Must be **> 0.84 m** = robot half-width 0.34 m + box half-width 0.5 m. At or below
0.84 m the two are in contact.

**Step 5 — Confirm arrival** *(GATE)*
```bash
gz model -m a200_0000/robot -p
cat /tmp/nav_goal_box.log
```
Must show: final pose within goal tolerance of `(27.12, 1.10)`, and **z ≈ 3.12 at
every sample along the way**, not just at the end.

**Step 6 — Cleanup** *(GATE — a leftover box blocks the next run's Step 3)*
```bash
python3 tools/spawn_obstacle.py --remove
wait $NAV 2>/dev/null
```
Must print **`PASS  'test_obstacle' absent from /world/park scene`**.

---

## What I would report back

| quantity | required | measured |
|---|---|---|
| box present in `/world/park/scene/info` after spawn | yes | — |
| **min robot-to-box centre distance** | **> 0.84 m** | — (the answer to your question) |
| goal `(27.12, 1.10)` reached | yes | — |
| `z` throughout | ≈ 3.12 | — |

Plus a per-step `command / output / result` block for every step above, and a
closing count of steps in each file vs steps run vs steps conditionally skipped
(Phase 3 Steps 7–8, reason stated).

## Interpretation note I would apply, not a failure

The **global** plan will run straight through the box for the whole run. park's
global costmap is static by design since 2026-08-27 (prior map + keepout +
inflation, **no sensor layer**); only the local costmap has an observation source
(gotcha #36). Every metre of clearance comes from MPPI on the 5×5 m rolling local
costmap. A plan drawn through the box in RViz is not the failure mode — a robot
that drives through it is.

If it does fail, I discriminate before diagnosing: robot still moving and closes
to ≤ 0.84 m = genuine avoidance failure, look at the local costmap's lidar source
(gotcha #36). Robot stops with `cmd_vel` still flowing and < ~1 cm displacement
per sample = a **stall**, gotcha #24 (rigid wheel-slip compliance makes a tight
avoidance turn physically unexecutable) — not an avoidance failure at all. The
discriminator is displacement per commanded sample, not distance to the box.

## Points where I stop and ask

1. **Now** — before touching the running sim (the decision above).
2. Any `CLEAN_SIM.md` Step 3 survivor — I name the leaking node type and propose
   the sweep-pattern edit; I do not quietly kill it and move on.
3. Any gate that fails — I stop at that step, quote the runbook line, give the
   exact command and its real output, my reading of why, and a concrete proposed
   edit to the runbook. I do not continue to later steps and do not route around
   it. Diagnosis (reading logs, querying topics, reading a tool's source) I do
   freely; running a command the file does not specify to get past a failure I do
   not.
