# NAV_PARK.md - autonomous navigation in park

Steps only. If a step fails, fix the file rather than working around it.
CLEAN_SIM.md must report `opt/ros : 0` / `shm : 0` before Step 2.

The stack: `launch/nav_park.launch.py` brings up GPS localization and nav2
under one lifecycle manager (`lifecycle_manager_navigation`), which owns the
map server, the keepout filter mask server, the costmap filter info server and
every nav2 node. Global position in `map` comes from GPS only, heading from
the ENU-referenced IMU, and wheel odometry is fused as velocity only, never
position (`config/gps_localization.yaml`). The keepout mask is what stops the
planner routing off the terrain edge. `collision_monitor` is in the command
path: `cmd_vel_smoothed` -> `cmd_vel`, stock Clearpath wiring, nothing
bypasses it.

## Step 1 - clean
Dispatch sim-operator: "Clean up and verify the machine is clean."
Gate: report shows `opt/ros : 0` and `shm : 0`.

## Step 2 - start park
Dispatch sim-operator: "Start the park world."
Gate: its report shows the robot spawned and sensors publishing.

## Step 3 - launch the navigation stack
    cd /home/thinhpham/Documents/Husky_viz
    source /opt/ros/jazzy/setup.bash
    setsid nohup ros2 launch launch/nav_park.launch.py > /tmp/nav_park.log 2>&1 &
    disown

## Step 4 - gate on readiness
    python3 tools/check_nav2_ready.py
Gate: prints `READY`. If not, report which gate failed and re-run.
Never sleep, never poll in a loop.

## Step 5 - verify the prior map
    python3 tools/check_map_alignment.py
Gate: prints `PASS`.

## Step 6 - gate on localization
    python3 tools/check_localization_drive.py --distance 10
Gate: prints `PASS`, exit code 0. Expected: EKF vs Gazebo truth
0.049 m mean / 0.112 m max while driving, 0.000 m at rest, heading
error 0.00 deg.

## Step 7 - send a goal
    python3 tools/nav_goal.py X Y [YAW_DEG]           # metric
    python3 tools/nav_goal_ll.py LAT LON              # lat/lon
Gate: exit code 0. `YAW_DEG` is optional and is not enforced - the goal
checker is `PositionGoalChecker`, position only. A goal inside a mapped
obstacle or off the terrain is correctly refused with `NO PATH`.

## Step 8 - confirm arrival
    gz model -m a200_0000/robot -p | head -3
Gate: position within 0.5 m of the goal; z near 3.1, not large negative.
Reached goals land on the goal exactly - measured gap 0.000 m, path
length 1.01x straight line, zero lethal cells on the path.

## Step 9 - demonstrate local obstacle avoidance
Run from the repo root (`cd ~/Documents/Husky_viz`); the module form is
required because this tool imports `tools.nav_goal`.
    python3 -m tools.check_local_avoidance
Gate: prints `PASS`. Drives at a small tree the prior map omits and
measures the closest approach; must stay above 1.89 m.

## Step 10 - end-to-end acceptance: the full route
    python3 tools/nav_route.py routes/park_route_1.yaml
Gate: exit code 0, all 5 waypoints reached, no recoveries, no aborts.
Expected: 77.2 m driven over 170 s, closest approach per waypoint
0.02 / 0.05 / 0.19 / 0.04 / 0.15 m, final pose 0.15 m from waypoint 5.
