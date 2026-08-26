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
