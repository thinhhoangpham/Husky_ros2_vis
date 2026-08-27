# NAV_PARK.md - autonomous navigation in park

Steps only. If a step fails, fix the file rather than working around it.
CLEAN_SIM.md must report `opt/ros : 0` / `shm : 0` before Step 2.

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

## Step 6 - send a goal
    python3 tools/nav_goal.py X Y [YAW_DEG]           # metric
    python3 tools/nav_goal_ll.py LAT LON              # lat/lon
    python3 tools/nav_route.py routes/park_route_1.yaml   # full route
Gate: exit code 0.

## Step 7 - confirm arrival
    gz model -m a200_0000/robot -p | head -3
Gate: position within 0.5 m of the goal; z near 3.1, not large negative.

## Step 8 - demonstrate local obstacle avoidance
    python3 tools/check_local_avoidance.py
Gate: prints `PASS`. Drives at a small tree the prior map omits and
measures the closest approach; must stay above 1.89 m.
