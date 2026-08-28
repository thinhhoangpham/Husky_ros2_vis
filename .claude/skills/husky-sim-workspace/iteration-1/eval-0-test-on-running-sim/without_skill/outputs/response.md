# DRY RUN — plan only, nothing executed

Requested: spawn a box in front of the robot in the running park sim, see whether nav2 goes around it, report closest approach.

This exact test already exists as a written procedure: **`DEMO.md` → "Demo: local costmap avoids an unmapped box (park)"** (Steps 1–6), backed by `tools/spawn_obstacle.py` and `tools/nav_goal.py`. I would follow it verbatim rather than improvise a box spawn. Below is what I would run, in order, with the value each check must show.

---

## Stop-and-ask BEFORE anything runs

Two blockers, both of which I would raise before touching the sim:

1. **The demo is not defined against an arbitrary running sim.** Its prerequisite is: `CLEAN_SIM.md` → `RUN_SIM.md` → `NAV_PARK.md` complete and passing, with the robot parked at route waypoint 5, ≈`(-30.80, -3.45)`. Both the goal `(27.12, 1.10)` and the box location `(14, -0.66)` are derived from that start pose — the box sits on the wp2↔wp3 leg, and `y = -0.66` is the path's actual lateral offset at `x = 14`. From any other pose those numbers block nothing and the result is meaningless. My standing practice (and the project's) is a full clean cycle before any test; a partial reuse of a running sim is the documented way results get silently invalidated.
2. **I do not know how this running sim was started.** Specifically: was the config `default` (required — `full` drops RTF to ~0.1 and makes the pass unreadable), is `nav_park.launch.py` actually up and did `check_nav2_ready.py` pass, and is there a leftover `test_obstacle` from an earlier run (it would fail the pre-spawn check).

**So my first action is to ask, not to run:**

> The box demo's numbers assume the robot is parked at waypoint 5 `(-30.80, -3.45)` after a full CLEAN_SIM → RUN_SIM → NAV_PARK cycle. Do you want me to (a) run the read-only precondition checks below against the sim you have up and proceed only if it is at that pose, (b) do the full clean restart cycle first (the safe, documented path), or (c) place the box relative to wherever the robot actually is now — which means picking new goal/box coordinates, and I would want you to approve them?

I would not pick one of those myself. Everything below is option (a)/(b) once you choose.

---

## Step 0 — Read-only precondition checks (safe on a live sim, no state change)

```bash
source /opt/ros/jazzy/setup.bash
gz model -m a200_0000/robot -p | head -3
```
Must show: `x ≈ -30.80`, `y ≈ -3.45`, `z ≈ 3.12`. A large negative `z` means the robot left the terrain (gotcha #25) and the run is void.

```bash
grep -i "applied config\|robot_default\|robot_full" /tmp/*husky*.log 2>/dev/null | tail -5
```
Must show config `default` (gotcha #13 — the launch script re-applies, so its log is the authority, not whatever `apply_config.sh` was last run with).

```bash
python3 tools/check_nav2_ready.py
```
Must print `READY`. Nav2 comes up healthy but silently ignores goals if `map -> odom` is missing (gotcha #34).

```bash
ros2 service call /a200_0000/controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers "{}"
```
Must show `joint_state_broadcaster` and `platform_velocity_controller` both `active` (gotcha #27; the `ros2 control` CLI is not installed, gotcha #33).

```bash
gz service -s /world/park/scene/info --reqtype gz.msgs.Empty \
  --reptype gz.msgs.Scene --timeout 30000 --req '' | grep -c test_obstacle
```
Must show `0` — no leftover box. If nonzero I would stop and ask before removing it, since removing is a state change.

If any of these fails I stop and report, rather than working around it.

## Step 1 — Send the return goal north (non-blocking)

```bash
cd ~/Documents/Husky_viz
python3 tools/nav_goal.py 27.12 1.10 > /tmp/nav_goal_box.log 2>&1 &
NAV=$!
```
Must show in the log: `goal accepted, navigating...`, and `gz model -m a200_0000/robot -p` shows `x` increasing.

## Step 2 — Spawn the box mid-drive

Watch `x` with **one-shot reads only** — no `sleep`, no polling loop, no wrapper script (project Workflow rule; `wait_for_ready.py` is explicitly rejected):

```bash
gz model -m a200_0000/robot -p        # repeat as separate one-shot calls
```
When `x ≈ 0` (about 14 m short of the box):

```bash
python3 tools/spawn_obstacle.py 14 -0.66
```
Must print `PASS  'test_obstacle' present in /world/park scene`. The `/world/park/create` reply itself is **not** the check — it returns success for a no-op (gotcha #4), which is why the tool re-reads `scene/info`.

Known first-run failure modes, both harmless and pre-box: pre-spawn verification FAIL (leftover box, or the scene service didn't answer in time — re-check `x` before retrying), and the create call timing out on a busy sim (confirm the box is genuinely absent, then re-run).

## Step 3 — Measure closest approach

```bash
gz model -m a200_0000/robot -p        # one-shot reads through the pass
```
Take the minimum centre-to-centre distance from the sampled poses to `(14, -0.66)`.

**Pass threshold: minimum distance > 0.84 m** = robot half-width 0.34 m + box half-width 0.50 m. At or below 0.84 m the robot and box are in contact, i.e. it did not go around.

Note on sampling: this is hand-sampled `gz model` reads, so the reported minimum is an upper bound on the true closest approach — the robot can pass its nearest point between two samples. I would say so explicitly in the report rather than quote it as exact. (`tools/check_local_avoidance.py` does the continuous 5 Hz TF sampling properly, but it is hardcoded to the `arbol4` tree at `(5.67, 4.58)` and its own goal, so it is not the tool for a spawned box.)

## Step 4 — Confirm arrival

```bash
gz model -m a200_0000/robot -p
cat /tmp/nav_goal_box.log
```
Must show: final pose within goal tolerance of `(27.12, 1.10)`, `z ≈ 3.12` at the end and at every sample along the way.

## Step 5 — Cleanup (I would ask before running this)

```bash
python3 tools/spawn_obstacle.py --remove
wait $NAV 2>/dev/null
```
Must print `PASS  'test_obstacle' absent from /world/park scene`. A box left behind blocks the next run's pre-spawn check.

---

## What the report would contain

| quantity | required | measured |
|---|---|---|
| box present in `/world/park/scene/info` after spawn | yes | — dry run — |
| **min robot-to-box centre distance** | **> 0.84 m** | — dry run — |
| goal `(27.12, 1.10)` reached | yes | — dry run — |
| `z` throughout | ≈ 3.12 | — dry run — |

Plus: `DEMO.md`'s results table currently reads "not yet recorded" for all four rows — this demo has never been executed. After a real run I would propose updating that table with the measured values, and ask before editing.

## Interpreting the result

- The **global plan will run straight through the box** — the global costmap in `config/nav2_park.yaml` is static by design (prior map + keepout + inflation, no sensor layer since 2026-08-27, gotcha #36). A plan drawn through the obstacle in RViz is expected, not the failure. The failure is the *robot* going through it.
- All avoidance comes from the local costmap + MPPI, fed by `/a200_0000/sensors/lidar2d_0/scan` (absolute topic — relative names silently never bind, gotchas #31/#36).
- If the robot stalls rather than detours, check `cmd_vel` displacement per sample, not distance to the box — gotcha #24 (wheel slip compliance) produced exactly that stuck-in-a-turn signature on park before. `wheel_slip.urdf.xacro` is currently disabled, so this should not recur, but it is the first thing to check.

## Explicitly NOT run in this dry run

No `run_husky_sim.sh`, no `ros2 launch`, no `kill`/`pkill`, no `rm /dev/shm/*`, no state-changing `gz service`, no `spawn_obstacle.py`, no `nav_goal.py`. Only file reads were performed.
