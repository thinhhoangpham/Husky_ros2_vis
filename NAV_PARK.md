# NAV_PARK.md - autonomous navigation in park

Steps only. If a step fails, fix the file rather than working around it.
Step 1 cleans automatically — no separate `CLEAN_SIM.md` pass is needed.

The stack: `launch/park_stock.launch.py` brings up GPS localization and nav2
under one lifecycle manager (`lifecycle_manager_navigation`), which owns the
map server and every nav2 node. Global position in `map` comes from GPS only,
heading from the ENU-referenced IMU, and wheel odometry is fused as velocity
only, never position (`config/gps_localization.yaml`). Goals are kept on the
terrain by `tools/nav_goal.py`, not by a costmap filter. `collision_monitor`
is in the command path: `cmd_vel_smoothed` -> `cmd_vel`, stock Clearpath wiring, nothing
bypasses it.

## Step 1 - start park
    python3 scripts/sim.py start park
Gate: must end with `READY park default nav`.

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
checker is `PositionGoalChecker`, position only. A goal inside a mapped
obstacle or off the terrain is correctly refused with `NO PATH`.

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
