#!/bin/bash
# Husky A200 simulation: camera, 2D+3D GPU lidar, IMU, compass, RF radio.
#
# Needs a custom world (Clearpath's stock worlds have no Magnetometer or
# RFComms system) and a custom launch file (clearpath_gz's
# simulation.launch.py restricts --world to six fixed names).
set -eo pipefail   # not -u: /opt/ros/jazzy/setup.bash references unset vars

# Any .sdf basename in worlds/: warehouse_ext (default), warehouse_ramp, park.
WORLD="${1:-warehouse_ext}"
RVIZ="${2:-false}"

# Spawn pose, per world. Clearpath's defaults (and sim_compass.launch.py's)
# put the robot at the origin, which only works for worlds whose ground plane
# is at z=0. worlds/park.sdf keeps the original ROS 1 world's authored model
# placements, so its terrain sits at z~=2.99 and a robot spawned at z=0.15
# materialises ~2.8 m BELOW the ground and falls through the world forever.
#
# Values are the authored spawn pose from the original
# natural_enviroment/launch/add_husky_park_1.launch, used verbatim.
# Worlds not listed here spawn at the launch file's defaults (origin), as
# before.
#
# Override any component from the command line without editing this file,
# e.g. the add_husky_park_2.launch alternative:
#   SPAWN_X=47 SPAWN_Y=1 SPAWN_Z=4.0 SPAWN_YAW=3.05 ~/run_husky_sim.sh park
case "$WORLD" in
  park)
    SPAWN_X="${SPAWN_X:-45.64}"
    SPAWN_Y="${SPAWN_Y:-0.02}"
    SPAWN_Z="${SPAWN_Z:-3.3}"
    SPAWN_YAW="${SPAWN_YAW:-2.6132}"
    ;;
  lake)
    SPAWN_X="${SPAWN_X:--47}"
    SPAWN_Y="${SPAWN_Y:--15}"
    SPAWN_Z="${SPAWN_Z:-4.0}"
    SPAWN_YAW="${SPAWN_YAW:-0}"
    ;;
esac

# Only forward the components that are actually set, so unlisted worlds keep
# whatever sim_compass.launch.py declares as its default.
# ("[ -n ... ] && ..." would be an exit under set -e when the test fails.)
POSE_ARGS=()
if [ -n "${SPAWN_X:-}" ];   then POSE_ARGS+=("x:=$SPAWN_X"); fi
if [ -n "${SPAWN_Y:-}" ];   then POSE_ARGS+=("y:=$SPAWN_Y"); fi
if [ -n "${SPAWN_Z:-}" ];   then POSE_ARGS+=("z:=$SPAWN_Z"); fi
if [ -n "${SPAWN_YAW:-}" ]; then POSE_ARGS+=("yaw:=$SPAWN_YAW"); fi
if [ ${#POSE_ARGS[@]} -gt 0 ]; then
  echo "==> spawn pose overrides: ${POSE_ARGS[*]}"
fi

# ros2 launch detaches its child nodes (static_transform_publisher,
# robot_state_publisher, ...): killing the launch process does not kill them.
# A previous run's leftovers cause intermittent, sensor-specific failures on
# the next launch (DDS discovery contention) even though the URDF is correct.
# See CLAUDE.md gotcha #11.
STALE=$(pgrep -f "static_transform_publisher|gz sim|clearpath_gz|sim_compass|parameter_bridge|rviz2|robot_state_publisher|controller_manager|ekf_node|twist_mux|imu_filter" | grep -v "^$$\$" || true)
if [ -n "$STALE" ]; then
  echo "==> stale sim processes found from a previous run, killing them first:"
  echo "$STALE" | xargs -r ps -o pid,cmd -p | sed 's/^/    /'
  for p in $STALE; do kill -9 "$p" 2>/dev/null || true; done
  sleep 3
fi

# kill -9 above (and any prior interrupted run) leaves FastDDS shared-memory
# segments locked in /dev/shm - the next process to bind that port then fails
# non-deterministically, causing specific topics/sensors to silently produce
# no data while others work fine. Safe to clear: these are transient IPC
# segments recreated on next use, not persistent state. See CLAUDE.md #12.
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true

source /opt/ros/jazzy/setup.bash

# Custom Gazebo models used by worlds/park.sdf (model://arbol4/... etc.) live
# in this repo, outside any ROS package. Harmonic reads GZ_SIM_RESOURCE_PATH
# (not Classic's GAZEBO_MODEL_PATH).
#
# This export only covers running `gz sim worlds/park.sdf` by hand: on the
# `ros2 launch` path below, clearpath_gz's gz_sim.launch.py OVERWRITES
# GZ_SIM_RESOURCE_PATH, so the models dir is injected in our own
# launch/gz_sim.launch.py instead. Keep the two in sync.
export GZ_SIM_RESOURCE_PATH="/home/thinhpham/Documents/Husky_viz/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

# (Re)apply the robot config before launching so the sim never silently runs
# whatever apply_config.sh last deployed: clearpath_gz regenerates the URDF
# from ~/clearpath/robot.yaml at startup (generate:=true default), so a config
# left active by an unrelated apply_config.sh run elsewhere would get spawned
# as-is, wrong sensors and all, without any error - lidar/IMU look fine
# because they exist in both configs, so nothing looks broken.
#
# Defaults to robot_default.yaml. Select the full sensor suite (camera,
# compass, radio) with:  SIM_CONFIG=full ~/run_husky_sim.sh
SIM_CONFIG="${SIM_CONFIG:-default}"
if [ ! -f "/home/thinhpham/Documents/Husky_viz/robot_configs/robot_$SIM_CONFIG.yaml" ]; then
  echo "ERROR: unknown SIM_CONFIG '$SIM_CONFIG' - no such file:" >&2
  echo "       /home/thinhpham/Documents/Husky_viz/robot_configs/robot_$SIM_CONFIG.yaml" >&2
  exit 1
fi
echo "==> applying robot config: $SIM_CONFIG"
/home/thinhpham/Documents/Husky_viz/scripts/apply_config.sh "$SIM_CONFIG" > /dev/null

ros2 launch /home/thinhpham/Documents/Husky_viz/launch/sim_compass.launch.py \
  world:="/home/thinhpham/Documents/Husky_viz/worlds/$WORLD" rviz:="$RVIZ" \
  "${POSE_ARGS[@]}" &
LAUNCH_PID=$!

# Clearpath's generator only bridges sensors it knows about, so the
# magnetometer and the comms topics need their own bridge.
#   [ = gz->ROS,  ] = ROS->gz
sleep 12
ros2 run ros_gz_bridge parameter_bridge \
  "/a200_0000/sensors/compass_0/mag@sensor_msgs/msg/MagneticField[gz.msgs.Magnetometer" \
  "/broker/msgs@ros_gz_interfaces/msg/Dataframe]gz.msgs.Dataframe" \
  "/husky/rx@ros_gz_interfaces/msg/Dataframe[gz.msgs.Dataframe" \
  "/base_station/rx@ros_gz_interfaces/msg/Dataframe[gz.msgs.Dataframe" \
  --ros-args -r __node:=extras_gz_bridge &
BRIDGE_PID=$!

trap 'kill $BRIDGE_PID $LAUNCH_PID 2>/dev/null' INT TERM
wait $LAUNCH_PID
