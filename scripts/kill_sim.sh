#!/bin/bash
# Kill every Husky sim process, cleanly, every time - not PID-dependent.
#
# `ros2 launch` detaches its child nodes (static_transform_publisher,
# robot_state_publisher, controller_manager, ...), so killing the launch
# process alone leaves them running. This kills by process-name pattern
# instead, so it works regardless of which launch spawned them or how many
# generations have accumulated. See CLAUDE.md #11/#12.
set -e  # not -u: setup.bash below references unset vars if sourced elsewhere

PATTERNS=(
  "gz sim"
  "gz_tools_vendor/bin/gz"
  "ros2 launch"
  "ros_gz_bridge/parameter_bridge"
  "static_transform_publisher"
  "robot_state_publisher"
  "controller_manager"
  "ekf_node"
  "twist_mux"
  "imu_filter"
  # Clearpath's teleop stack - survives ros2 launch teardown
  "marker_server"
  "joy_linux"
  "teleop_twist_joy"
  "urg_node"
  "velodyne"
  "rviz2"
  # RSSI localization stage - tools/rssi_viz.py and
  # tools/rssi_localization_node.py. Both are started by absolute script
  # path because tools/ is not an ament package, so the path is what shows
  # up on the command line and is what has to be matched. rssi_viz carries
  # no launch_ros `namespace`, so its command line contains no "a200_0000"
  # and sim.py's EXTRA_SWEEP never reached it - it survived a "CLEAN" stop.
  # rssi_localization_node only died because its `namespace` puts
  # "-r __ns:=/a200_0000" on its command line, which is incidental, so it is
  # listed explicitly too. See CLAUDE.md #21.
  "tools/rssi_viz.py"
  "tools/rssi_localization_node.py"
  # Ground segmentation node (ros2_ws package, run by executable path). It
  # only died because "-r pointcloud_topic:=/a200_0000/..." happens to contain
  # "a200_0000", which sim.py's EXTRA_SWEEP matches - incidental, same as the
  # rssi node above, so match its install path explicitly. See CLAUDE.md #21.
  "patchworkpp/lib/patchworkpp/patchworkpp_node"
  # nav2 + GPS localization stack (launch/park_stock.launch.py) - see CLAUDE.md #21
  "navsat_transform_node"
  "ekf_node_map"
  "controller_server"
  "planner_server"
  "smoother_server"
  "route_server"
  "behavior_server"
  "bt_navigator"
  "waypoint_follower"
  "velocity_smoother"
  "collision_monitor"
  "docking_server"
  "map_server"
  "filter_mask_server"
  "costmap_filter_info_server"
  "lifecycle_manager"
)

SELF=$$
KILLED=0
for pat in "${PATTERNS[@]}"; do
  for pid in $(pgrep -f "$pat" 2>/dev/null || true); do
    # skip ourselves and this script's own bash -c wrapper
    [ "$pid" = "$SELF" ] && continue
    grep -qa "$0" "/proc/$pid/cmdline" 2>/dev/null && continue
    kill -9 "$pid" 2>/dev/null && KILLED=$((KILLED+1)) || true
  done
done
echo "==> killed $KILLED process(es)"

sleep 2

# a killed DDS participant cannot release its shared-memory port cleanly;
# the next launch that tries to bind the same port fails non-deterministically
# (confirmed cause of silent per-topic data loss - CLAUDE.md #12)
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
echo "==> cleared stale FastDDS shared-memory segments"

if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1 || true
  echo "==> stopped ros2 daemon"
fi

echo "==> verifying..."
REMAIN=""
for pat in "${PATTERNS[@]}"; do
  for pid in $(pgrep -f "$pat" 2>/dev/null || true); do
    [ "$pid" = "$SELF" ] && continue
    grep -qa "$0" "/proc/$pid/cmdline" 2>/dev/null && continue
    REMAIN="$REMAIN $pid"
  done
done

if [ -n "$REMAIN" ]; then
  echo "==> WARNING: still running:$REMAIN"
  ps -o pid,cmd -p $REMAIN 2>/dev/null
  exit 1
else
  echo "==> clean: no sim processes remain"
fi
