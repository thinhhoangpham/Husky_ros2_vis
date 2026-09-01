# NAV_PARK.md - autonomous navigation in park

Steps only. If a step fails, fix the file rather than working around it.
Step 1 runs `RUN_SIM.md` Part A, which does **not** clean itself — its A1 is a
full `CLEAN_SIM.md` pass.

The stack: `launch/park_stock.launch.py world_and_robot:=false` brings up GPS
localization and nav2 under one lifecycle manager
(`lifecycle_manager_navigation`), which owns the map server and every nav2
node. `world_and_robot:=false` is mandatory — the default `true` starts the
world and robot from the same invocation and renders an empty park (gotcha
#39). Global position in `map` comes from GPS only, heading from the
ENU-referenced IMU, and wheel odometry is fused as velocity only, never
position (`config/gps_localization.yaml`). Goals are kept on the terrain by
`tools/nav_goal.py`'s `in_terrain()` alone — the keepout mask and both
costmaps' `keepout_filter` were removed on 2026-08-31 (gotcha #30).
`collision_monitor` is in the command path: `cmd_vel_smoothed` -> `cmd_vel`,
stock Clearpath wiring, nothing bypasses it.

## Step 1 - start park on the stock chain
Follow `RUN_SIM.md` Part A, stages A1-A6 then A7a (**not** A7b). Do not
continue past a stage until its gates pass (gotcha #39).
Gate: every row of the A8 checklist holds. In particular:
- A5 - `joint_state_broadcaster` and `platform_velocity_controller` both
  `active`. Nothing on this path recovers the spawner automatically and park
  loses that race in 42% of runs (gotcha #27); re-run A5's
  `--switch-timeout 30` spawner by hand until both are active.
- A6 - `/gui/camera/pose` moved to `a200_0000/robot` and did **not** move for
  a bogus name. If it did not move, the GUI missed the spawn: stop (Part C)
  and restart from A1.
- A7a - `python3 tools/check_nav2_ready.py` prints `READY`, 8 lifecycle nodes
  active, both action servers, both costmaps OK.
There is no `READY park default nav` verdict on this path, and
`scripts/sim.py status` does not apply to it (A9).

## Step 2 - verify the prior map
    python3 tools/check_map_alignment.py
Gate: prints `PASS`.

## Step 3 - gate on localization
    python3 tools/check_localization_drive.py --distance 10
Gate: prints `PASS`, exit code 0. Expected: EKF vs Gazebo truth
0.049 m mean / 0.112 m max while driving, 0.000 m at rest, heading
error 0.00 deg.

## Step 4 - send a goal
    python3 tools/nav_goal.py X Y [YAW_DEG]           # metric
    python3 tools/nav_goal_ll.py LAT LON              # lat/lon
Gate: exit code 0. `YAW_DEG` is optional and is not enforced - the goal
checker is `PositionGoalChecker`, position only. A goal off the terrain is
refused by `in_terrain()` before it is sent (`REFUSED`); a goal inside a
mapped obstacle is refused by the planner with `NO PATH`.

## Step 5 - confirm arrival
    gz model -m a200_0000/robot -p | head -3
Gate: position within 0.5 m of the goal; z near 3.1, not large negative.
Reached goals land on the goal exactly - measured gap 0.000 m, path
length 1.01x straight line, zero lethal cells on the path.

## Step 6 - demonstrate local obstacle avoidance
Run from the repo root (`cd ~/Documents/Husky_viz`); the module form is
required because this tool imports `tools.nav_goal`.
    python3 -m tools.check_local_avoidance
Gate: prints `PASS`. Drives at a small tree the prior map omits and
measures the closest approach; must stay above 1.89 m.

## Step 7 - end-to-end acceptance: the full route
    python3 tools/nav_route.py routes/park_route_1.yaml
Gate: exit code 0, all 5 waypoints reached, no recoveries, no aborts.
Expected: 77.2 m driven over 170 s, closest approach per waypoint
0.02 / 0.05 / 0.19 / 0.04 / 0.15 m, final pose 0.15 m from waypoint 5.
