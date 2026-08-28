#!/bin/bash
# start_sim.sh - one command from nothing to a fully-verified-ready sim.
#
# Replaces the manual CLEAN_SIM.md -> RUN_SIM.md -> NAV_PARK.md sequence:
#
#   ./scripts/start_sim.sh <world> [--rviz] [--no-nav]
#
# Phases, in order, each gate named in the output and timed:
#   1  clean      graceful shutdown (SIGINT group -> SIGTERM -> SIGKILL),
#                 ros2 daemon stop, clear /dev/shm, verify
#   2  launch     detached run_husky_sim.sh
#   3  gates      3a landed  3b sensors  3c controllers  3d description+TF
#                 3e renderer
#   4  nav2       unless --no-nav
#   5  rviz       only with --rviz, only after everything above passed
#
# Measured reference on this machine (park, config `default`, no RViz):
#   robot landed 24 s | controllers active 32 s | robot_description 35 s |
#   nav2 READY 73 s.  These are the baseline to compare against, NOT the
#   timeouts - the timeouts below are deliberately much looser.
#
# On a failed gate this prints the observed vs expected value, exits nonzero,
# and LEAVES EVERYTHING RUNNING for inspection. No retries, no self-healing,
# with one documented exception: the controller-spawner recovery in gate 3c.
set -eo pipefail   # not -u: /opt/ros/jazzy/setup.bash references unset vars
                   # (CLAUDE.md gotcha #10)

REPO="/home/thinhpham/Documents/Husky_viz"
cd "$REPO"

# ---------------------------------------------------------------- arguments
WORLD=""
WANT_RVIZ=false
WANT_NAV=true
for arg in "$@"; do
  case "$arg" in
    --rviz)   WANT_RVIZ=true ;;
    --no-nav) WANT_NAV=false ;;
    -*)       echo "ERROR: unknown option '$arg'" >&2; exit 2 ;;
    *)        if [ -n "$WORLD" ]; then
                echo "ERROR: more than one world given: '$WORLD' and '$arg'" >&2; exit 2
              fi
              WORLD="$arg" ;;
  esac
done
if [ -z "$WORLD" ]; then
  echo "usage: $0 <park|lake|warehouse_ext|warehouse_ramp> [--rviz] [--no-nav]" >&2
  exit 2
fi

# Per-world expected settled robot z for gate 3a (RUN_SIM.md Step 5 table).
# The robot's base sits ~0.13 m above the surface it rests on. lake's terrain
# spans 3.5-5.9 m so its window is a range, not a point.
case "$WORLD" in
  park)                        Z_MIN=2.9  Z_MAX=3.4 ;;
  lake)                        Z_MIN=3.4  Z_MAX=6.2 ;;
  warehouse_ext|warehouse_ramp) Z_MIN=-0.1 Z_MAX=0.4 ;;
  *) echo "ERROR: unknown world '$WORLD' - supported: park, lake, warehouse_ext, warehouse_ramp" >&2
     exit 2 ;;
esac
if [ ! -f "$REPO/worlds/$WORLD.sdf" ]; then
  echo "ERROR: no such world file: $REPO/worlds/$WORLD.sdf" >&2
  exit 2
fi
# nav2 wiring in this repo is park-specific (config/nav2_park.yaml, the park
# prior map and keepout mask, park's datum). Refuse rather than bring up a
# stack pointed at the wrong world.
if [ "$WANT_NAV" = true ] && [ "$WORLD" != "park" ]; then
  echo "ERROR: the nav2 stack (launch/nav_park.launch.py) is park-only." >&2
  echo "       Re-run with --no-nav for world '$WORLD'." >&2
  exit 2
fi

source /opt/ros/jazzy/setup.bash

T_START=$(date +%s)
PHASE_T=$T_START
gate_start() { PHASE_T=$(date +%s); printf '\n==> GATE %s\n' "$*"; }
gate_ok()    { echo "    OK  ($(( $(date +%s) - PHASE_T ))s)"; }
fail() {
  echo ""
  echo "!!! FAILED GATE: $1"
  echo "    expected: $2"
  echo "    observed: $3"
  echo "!!! everything left running for inspection; total $(( $(date +%s) - T_START ))s"
  exit 1
}

echo "=== start_sim: world=$WORLD  rviz=$WANT_RVIZ  nav2=$WANT_NAV"

# ======================================================= PHASE 1 - clean
# CLEAN_SIM.md, but graceful first. kill -9 is what strands FastDDS
# shared-memory segments (gotcha #12); a process that exits on SIGINT/SIGTERM
# releases them itself. SIGINT goes to the process GROUP so `ros2 launch`
# unwinds its children instead of orphaning them (gotcha #11).
gate_start "1 - clean"

# --- process collection ----------------------------------------------------
# The sweep is NAMESPACE- and ANCESTRY-based first, name-based only as a net.
#
# Gotcha #21: a sweep verified with the same pattern list it kills with cannot
# see what it does not kill, and that list has now been wrong three times -
# first Clearpath's teleop stack (marker_server/joy_linux/teleop_twist_joy),
# then ros_gz_image/image_bridge, and on 2026-08-27 `joy_linux_node` survived
# this script's own first live run and tripped Gate 1. So the primary
# mechanism is no longer a name list:
#
#   PRIMARY  1. any process whose /proc/<pid>/cmdline contains `a200_0000`
#               (every Clearpath node is pushed into that namespace)
#            2. every member of those processes' process groups, and the
#               full descendant closure of them - i.e. whatever a launch tree
#               being shut down owns, whatever it happens to be called
#   SECONDARY 3. a name list, for the things that live OUTSIDE the namespace:
#               gz sim, gz_tools_vendor, rviz2, the bridges, `ros2 launch`
#
# Gate 1's verification below stays deliberately independent of all three
# (it greps ps for ros/gazebo/a200/husky/clearpath/rviz), so it can still
# report a survivor the sweep never targeted - exactly as it just did.
#
# Never pgrep a pattern that appears in this script's own command line
# (gotcha #9) - hence the explicit self/parent/`bash -c` exclusions.
NAME_PATTERNS='gz sim|gz_tools_vendor/bin/gz|ros2 launch|ros_gz_bridge|parameter_bridge|clock_bridge|ros_gz_image|image_bridge|rviz2|static_transform_publisher|robot_state_publisher|controller_manager|ekf_node|twist_mux|imu_filter|marker_server|interactive_marker|joy_linux|joy_node|teleop_twist_joy|urg_node|velodyne|navsat_transform_node|controller_server|planner_server|smoother_server|route_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|collision_monitor|docking_server|map_server|filter_mask_server|costmap_filter_info_server|lifecycle_manager|sim_compass|clearpath_gz'

SELF_PGID=$(ps -o pgid= -p $$ | tr -d ' ')
# ...and the group of whatever invoked us, so a caller shell that happens to
# share a group with a matched process is never swept up (gotcha #9).
PARENT_PGID=$(ps -o pgid= -p "$PPID" 2>/dev/null | tr -d ' ')

proc_cmdline() { tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null; }

# gotcha #9: never kill this script, its parent, or the shell wrapping it.
excluded() {
  local pid="$1" cl
  [ "$pid" = "$$" ] && return 0
  [ "$pid" = "$PPID" ] && return 0
  cl=$(proc_cmdline "$pid")
  case "$cl" in
    *"bash -c"*|*start_sim.sh*) return 0 ;;
  esac
  return 1
}

sim_pids() {
  local d pid cl seeds="" all="" snap kids g pgids pg
  # 1 + 3: seed from the namespace (primary) and the name net (secondary).
  for d in /proc/[0-9]*; do
    pid=${d#/proc/}
    cl=$(proc_cmdline "$pid")
    [ -z "$cl" ] && continue
    if printf '%s' "$cl" | grep -q 'a200_0000'; then
      seeds="$seeds $pid"
    elif printf '%s' "$cl" | grep -qE "$NAME_PATTERNS"; then
      seeds="$seeds $pid"
    fi
  done
  [ -z "${seeds// /}" ] && return 0

  snap=$(ps -eo pid=,ppid=,pgid= 2>/dev/null)
  all=$(printf '%s\n' $seeds | sort -un | tr '\n' ' ')

  # 2a: whole process group of every seed (a launch tree shares one pgid),
  #     never this script's own group.
  pgids=$(for p in $all; do echo "$snap" | awk -v p="$p" '$1==p{print $3}'; done | sort -un)
  for g in $pgids; do
    [ "$g" = "$SELF_PGID" ] && continue
    [ -n "$PARENT_PGID" ] && [ "$g" = "$PARENT_PGID" ] && continue
    all="$all $(echo "$snap" | awk -v g="$g" '$3==g{print $1}' | tr '\n' ' ')"
  done
  all=$(printf '%s\n' $all | sort -un | tr '\n' ' ')

  # 2b: descendant closure - children spawned by anything already in the set.
  while :; do
    kids=$(echo "$snap" | awk -v L="$all" '
      BEGIN { n=split(L,a," "); for(i=1;i<=n;i++) m[a[i]]=1 }
      m[$2] && !m[$1] { print $1 }' | sort -un | tr '\n' ' ')
    [ -z "${kids// /}" ] && break
    all=$(printf '%s\n' $all $kids | sort -un | tr '\n' ' ')
  done

  for pid in $all; do
    [ -d "/proc/$pid" ] || continue
    excluded "$pid" && continue
    # never target this script's own process group, nor the caller's
    pg=$(echo "$snap" | awk -v p="$pid" '$1==p{print $3}')
    [ "$pg" = "$SELF_PGID" ] && continue
    [ -n "$PARENT_PGID" ] && [ "$pg" = "$PARENT_PGID" ] && continue
    echo "$pid"
  done
}

describe() { ps -o pid=,cmd= -p "$@" 2>/dev/null | cut -c1-110 | sed 's/^/      /'; }

# Wait up to $2 seconds for the pids in $1 to disappear. Not a fixed sleep -
# it re-checks and returns as soon as they are gone.
wait_gone() {
  local pids="$1" limit="$2" waited=0 alive
  while [ "$waited" -lt "$limit" ]; do
    alive=""
    for p in $pids; do [ -d "/proc/$p" ] && alive="$alive $p"; done
    [ -z "$alive" ] && return 0
    sleep 1; waited=$((waited+1))
  done
  echo "$alive"
  return 1
}

# One full SIGINT-group -> SIGTERM -> SIGKILL escalation over a given pid set.
# Appends anything that needed SIGKILL to $FORCED.
FORCED=""
sweep_once() {
  local victims="$1" left pgid p
  # Escalation step 1: SIGINT to each victim's process GROUP.
  for p in $victims; do
    pgid=$(ps -o pgid= -p "$p" 2>/dev/null | tr -d ' ' || true)
    [ -n "$pgid" ] && [ "$pgid" != "$SELF_PGID" ] \
      && kill -INT "-$pgid" 2>/dev/null || true
  done
  left=$(wait_gone "$victims" 15) || true
  # Escalation step 2: SIGTERM the individuals that ignored SIGINT.
  if [ -n "${left// /}" ]; then
    echo "    SIGINT did not clear these; sending SIGTERM:"
    describe $left
    for p in $left; do kill -TERM "$p" 2>/dev/null || true; done
    left=$(wait_gone "$left" 10) || true
  fi
  # Escalation step 3: SIGKILL, genuine stragglers only. Each one named -
  # a process that CONSISTENTLY needs force is a bug to surface, because it
  # is the one leaking /dev/shm segments into the next run (gotcha #12).
  if [ -n "${left// /}" ]; then
    echo "    SIGTERM did not clear these either; forcing SIGKILL:"
    describe $left
    FORCED="$FORCED $left"
    for p in $left; do kill -KILL "$p" 2>/dev/null || true; done
    wait_gone "$left" 10 >/dev/null || true
  fi
}

# Re-collect after each escalation instead of trusting the pid set captured
# at the start. On 2026-08-27 the surviving joy_linux_node had a pid HIGHER
# than this script's own, i.e. it did not exist when the first set was taken:
# a launch tree that is mid-shutdown can still spawn (respawn-on-exit nodes,
# a launch action that had not started yet), and that child is then orphaned
# when its parent finally exits. Bounded at 3 rounds - if new sim processes
# keep appearing after that, something is actively restarting them and Gate 1
# below must be the thing that says so, not a loop that hides it.
ROUND=0
while [ "$ROUND" -lt 3 ]; do
  VICTIMS=$(sim_pids | tr '\n' ' ')
  [ -z "${VICTIMS// /}" ] && break
  ROUND=$((ROUND+1))
  if [ "$ROUND" -eq 1 ]; then
    echo "    found $(echo $VICTIMS | wc -w) sim process(es); shutting down gracefully"
  else
    echo "    round $ROUND: $(echo $VICTIMS | wc -w) more appeared during shutdown:"
    describe $VICTIMS
  fi
  sweep_once "$VICTIMS"
done
# `[ ] && echo` would be an AND-list whose failure exits under set -e; use if.
if [ "$ROUND" -eq 0 ]; then echo "    nothing was running"; fi

if [ -n "${FORCED// /}" ]; then
  echo "    NOTE: needed SIGKILL:$FORCED  <- report this; repeat offenders are a bug"
else
  echo "    all processes exited gracefully (no SIGKILL used)"
fi

# The daemon owns one live FastDDS participant, so it keeps two fastrtps_*
# segments alive and recreates them on the next `ros2` call. Stop it BEFORE
# the rm or the shm gate can never read 0. It restarts itself when needed.
ros2 daemon stop >/dev/null 2>&1 || true
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true

# CLEAN_SIM.md Step 3. kill_sim.sh's own "clean" message is never trusted -
# it verifies with the same list it kills with (gotcha #21).
clean_report() {
  ps -eo pid,cmd --no-headers | grep -viE "grep|bash -c|start_sim.sh" \
    | grep -iE "ros|gazebo|gz |a200|husky|clearpath|rviz" \
    | grep -v "ros2-daemon" || true          # a lone ros2-daemon is acceptable
}
count_optros() { ps -eo cmd --no-headers | grep -c '^/opt/ros' || true; }
count_shm()    { ls /dev/shm 2>/dev/null | grep -c fastrtps || true; }

REMAIN=$(clean_report)
OPTROS=$(count_optros)
SHM=$(count_shm)
# A nonzero shm on the FIRST read is usually transient - participants release
# their segments as they tear down. Re-read ONCE. Never rm a second time:
# that hides a live process instead of finding it (CLEAN_SIM.md Step 3).
if [ "$SHM" -ne 0 ]; then
  echo "    shm read $SHM on first look; re-reading once (transient teardown?)"
  SHM=$(count_shm)
fi
# `[ test ] && fail` would be an AND-list whose failure exits under set -e; use if.
if [ -n "$REMAIN" ]; then
  fail "1 - clean (survivors)" "no sim process lines" "$(echo "$REMAIN" | cut -c1-110)"
fi
if [ "$OPTROS" -ne 0 ]; then
  fail "1 - clean (opt/ros)" "opt/ros : 0" "opt/ros : $OPTROS"
fi
if [ "$SHM" -ne 0 ]; then
  fail "1 - clean (shm)" "shm : 0 on the second read" \
       "shm : $SHM (segments are being recreated - something is still alive)"
fi
echo "    opt/ros : 0    shm : 0"
gate_ok

# ================================================ PHASE 2 - launch Gazebo
# setsid + disown: a plain `nohup ... &` stays in the caller's process group,
# so an interrupt reaches Gazebo mid-load and reads as a crash (gotcha #22).
#
# run_husky_sim.sh takes the world only - there is no RViz argument. RViz is
# started separately in phase 5 (RUN_SIM.md Step 6).
gate_start "2 - launch gazebo"
SIM_LOG=/tmp/sim.log
: > "$SIM_LOG"
setsid nohup "$HOME/run_husky_sim.sh" "$WORLD" > "$SIM_LOG" 2>&1 < /dev/null &
disown
echo "    launched, log: $SIM_LOG"
gate_ok

# ================================================== PHASE 3 - gates on sim
# Generic bounded waiter: runs a shell snippet until it succeeds or the
# timeout expires. Bounded, named, and it re-checks rather than sleeping
# blind - a fixed sleep is never a substitute for a check.
await() {   # await <seconds> <shell-test>
  local limit="$1" test="$2" waited=0
  while [ "$waited" -lt "$limit" ]; do
    if eval "$test" >/dev/null 2>&1; then return 0; fi
    sleep 2; waited=$((waited+2))
  done
  return 1
}

# --- 3a: robot landed ------------------------------------------------------
# A large negative z means it spawned under the terrain and is falling
# (gotcha #23). Baseline: park lands at ~24 s.
gate_start "3a - robot landed (expect z in $Z_MIN..$Z_MAX for $WORLD)"
# `gz model -p` prints the pose as a bracketed XYZ triple under a
# "Pose [ XYZ (m) ]" header; some builds print `position:` / `z:` fields
# instead. Accept either, print nothing if neither is present.
robot_z() {
  gz model -m a200_0000/robot -p 2>/dev/null | awk '
    /Pose|position/ { seen=1 }
    seen && match($0, /\[[ ]*[-0-9.eE+]+[ ]+[-0-9.eE+]+[ ]+[-0-9.eE+]+[ ]*\]/) {
      s = substr($0, RSTART+1, RLENGTH-2); split(s, a, /[ \t]+/);
      n = 0; for (i in a) if (a[i] != "") n++;
      print a[3]; exit
    }
    seen && /^[ \t]*z:/ { gsub(/[^-0-9.eE+]/, "", $2); print $2; exit }
  '
}
z_in_window() {
  local z; z=$(robot_z)
  [ -n "$z" ] || return 1
  awk -v z="$z" -v a="$Z_MIN" -v b="$Z_MAX" 'BEGIN{exit !(z>=a && z<=b)}'
}
await 180 '[ -n "$(robot_z)" ]' \
  || fail "3a - robot landed" "a pose from gz model -m a200_0000/robot -p within 180s" \
          "no pose - the robot never spawned (see $SIM_LOG)"
await 120 z_in_window \
  || fail "3a - robot landed" "z between $Z_MIN and $Z_MAX" \
          "z = $(robot_z) (large negative = spawned under the terrain, gotcha #23)"
echo "    z = $(robot_z)"
gate_ok

# --- 3b: sensor topics -----------------------------------------------------
# A topic in `ros2 topic list` only means discovery found it; require an
# actual publisher on the two that matter.
gate_start "3b - sensor topics publishing"
pubcount() { ros2 topic info -v "$1" 2>/dev/null | grep -A1 "Publisher count" | head -1 | grep -oE '[0-9]+' | head -1; }
for t in /a200_0000/sensors/imu_0/data /a200_0000/platform/odom; do
  await 180 "ros2 topic info -v $t 2>/dev/null | grep -q 'Publisher count: 1'" \
    || fail "3b - sensor topics" "Publisher count: 1 on $t" "Publisher count: $(pubcount $t) on $t"
  echo "    $t : Publisher count 1"
done
gate_ok

# --- 3c: controllers -------------------------------------------------------
# `ros2 control` CLI is not installed on this machine - the service call is
# the working form (gotcha #33). imu_0/data is published straight from the
# Gazebo sensor, so 3b is structurally blind to a dead spawner (gotcha #27).
#
# The spawner race is NOT reliably deterministic - measured 2026-08-27,
# Clearpath's own spawner won. So CHECK FIRST and only recover if needed:
# running the recovery spawner unconditionally on a healthy launch prints
# `Failed to configure controller` and exits 1, which reads as a failure and
# is pure noise. Baseline: active at ~32 s.
gate_start "3c - controllers active"
ctrl_state() {
  ros2 service call /a200_0000/controller_manager/list_controllers \
    controller_manager_msgs/srv/ListControllers "{}" 2>/dev/null
}
# The reply prints one ControllerState(...) per controller, name and state on
# the same line, so match them together rather than counting 'active' globally
# (which a third controller could satisfy while one of ours is inactive).
ctrl_ok() {
  local out; out=$(ctrl_state)
  echo "$out" | grep -q "name='joint_state_broadcaster'.*state='active'" || return 1
  echo "$out" | grep -q "name='platform_velocity_controller'.*state='active'" || return 1
}
if ! await 120 ctrl_ok; then
  echo "    not active yet - running the recovery spawner once (--switch-timeout 30, gotcha #27)"
  ros2 run controller_manager spawner \
    joint_state_broadcaster platform_velocity_controller \
    --controller-manager /a200_0000/controller_manager \
    --switch-timeout 30 || true
  await 60 ctrl_ok \
    || fail "3c - controllers active" "joint_state_broadcaster and platform_velocity_controller both state='active'" \
            "$(ctrl_state | grep -E "name=|state=" | tr '\n' ' ' | cut -c1-300)"
  echo "    recovered by the manual spawner"
fi
echo "    joint_state_broadcaster + platform_velocity_controller : active"
gate_ok

# --- 3d: robot_description + kinematic TF ----------------------------------
# NEW GATE. robot_state_publisher has been observed alive and logging "Robot
# initialized" while being absent from the DDS graph and publishing nothing:
# no robot model in RViz, no robot kinematic TF, and every existing RUN_SIM.md
# gate still green. Root cause not yet established - this gate makes the
# failure visible instead of silent.
#
# TF must be looked up with /tf and /tf_static remapped into the namespace
# (Ruling D3): the whole Clearpath stack publishes on /a200_0000/tf, and a
# default tf2 listener (or `tf2_echo`) subscribes to the global /tf and sees
# nothing at all. Baseline: publishing at ~35 s.
gate_start "3d - robot_description + kinematic TF"
await 120 "ros2 topic info -v /a200_0000/robot_description 2>/dev/null | grep -q 'Publisher count: 1'" \
  || fail "3d - robot_description" "Publisher count: 1 on /a200_0000/robot_description" \
          "Publisher count: $(pubcount /a200_0000/robot_description) - robot_state_publisher may be alive but absent from the DDS graph"
echo "    /a200_0000/robot_description : Publisher count 1"

TF_CHECK=$(mktemp /tmp/start_sim_tfcheck.XXXXXX.py)
cat > "$TF_CHECK" <<'PY'
"""One-shot base_link -> front_left_wheel_link lookup, /tf remapped (Ruling D3)."""
import sys
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
rclpy.init(args=["--ros-args",
                 "-r", "/tf:=/a200_0000/tf",
                 "-r", "/tf_static:=/a200_0000/tf_static"])
node = Node("start_sim_tf_check", parameter_overrides=[])
buf = Buffer()
TransformListener(buf, node)
ok = False
for _ in range(120):                      # bounded: ~12 s of spinning, no sleep loop
    rclpy.spin_once(node, timeout_sec=0.1)
    if buf.can_transform("base_link", "front_left_wheel_link", rclpy.time.Time()):
        ok = True
        break
node.destroy_node()
rclpy.shutdown()
print("TF_OK" if ok else "TF_MISSING")
sys.exit(0 if ok else 1)
PY
if ! await 90 "python3 $TF_CHECK | grep -q TF_OK"; then
  rm -f "$TF_CHECK"
  fail "3d - kinematic TF" "base_link -> front_left_wheel_link resolves on /a200_0000/tf(_static)" \
       "transform never became available - robot_description published but the kinematic chain is absent from TF"
fi
rm -f "$TF_CHECK"
echo "    base_link -> front_left_wheel_link : resolves"
gate_ok

# --- 3e: renderer ----------------------------------------------------------
# Services are scoped by world name (gotcha #26). NOTE: this queries the
# SERVER's scene, so it proves the model reached the scene graph - it cannot
# prove the GUI actually rendered it. The GUI-process check is the closest
# available proxy; an empty-looking window with both green needs a full
# CLEAN_SIM + relaunch cycle (RUN_SIM.md Step 5).
gate_start "3e - renderer"
scene_count() {
  gz service -s "/world/$WORLD/scene/info" --reqtype gz.msgs.Empty \
    --reptype gz.msgs.Scene --timeout 30000 --req '' 2>/dev/null | grep -c 'a200_0000/robot' || true
}
await 90 '[ "$(gz service -s "/world/'"$WORLD"'/scene/info" --reqtype gz.msgs.Empty --reptype gz.msgs.Scene --timeout 30000 --req "" 2>/dev/null | grep -c "a200_0000/robot")" -ge 1 ]' \
  || fail "3e - renderer scene" "a200_0000/robot present in /world/$WORLD/scene/info" "count $(scene_count)"
GUI_PID=$(pgrep -af "gz sim" | grep -v "bash -c" | grep -v server | head -1 | awk '{print $1}')
if [ -z "$GUI_PID" ]; then
  fail "3e - renderer gui" "a gz sim GUI process alive" "no GUI process found"
fi
echo "    model in scene, GUI pid $GUI_PID"
gate_ok

# ==================================================== PHASE 4 - nav2
# nav_park.launch.py brings up GPS localization and nav2 under one lifecycle
# manager, now ten nodes (route_server, docking_server and smoother_server
# were removed 2026-08-27). check_nav2_ready.py is the only accepted gate: a
# lifecycle 'active' does not mean costmaps hold data or that TF resolves, and
# a goal sent too early drives the robot off the terrain edge (gotcha #25).
# Baseline: READY at ~73 s.
if [ "$WANT_NAV" = true ]; then
  gate_start "4 - nav2 READY"
  NAV_LOG=/tmp/nav_park.log
  : > "$NAV_LOG"
  setsid nohup ros2 launch "$REPO/launch/nav_park.launch.py" > "$NAV_LOG" 2>&1 < /dev/null &
  disown
  echo "    launched, log: $NAV_LOG"
  await 240 "python3 $REPO/tools/check_nav2_ready.py 2>/dev/null | grep -q READY" \
    || fail "4 - nav2 READY" "check_nav2_ready.py prints READY within 240s" \
            "$(python3 "$REPO/tools/check_nav2_ready.py" 2>&1 | tail -20)"
  gate_ok
else
  echo ""
  echo "==> PHASE 4 - nav2 skipped (--no-nav)"
fi

# ==================================================== PHASE 5 - RViz
# Attach form from RUN_SIM.md Step 6, verbatim. Do NOT add -r __ns:=/a200_0000
# - the config already uses absolute topics and the namespace push breaks the
# TF remaps. Started last, only after every gate above passed, because RViz
# costs ~60% of a core.
if [ "$WANT_RVIZ" = true ]; then
  gate_start "5 - rviz"
  RVIZ_LOG=/tmp/rviz.log
  : > "$RVIZ_LOG"
  setsid nohup rviz2 -d "$REPO/config/nav_park.rviz" \
    --ros-args -r /tf:=/a200_0000/tf -r /tf_static:=/a200_0000/tf_static \
    -p use_sim_time:=true > "$RVIZ_LOG" 2>&1 < /dev/null &
  disown
  await 60 'pgrep -f nav_park.rviz >/dev/null' \
    || fail "5 - rviz" "an rviz2 process bound to config/nav_park.rviz" "no pid (see $RVIZ_LOG)"
  echo "    rviz pid $(pgrep -f nav_park.rviz | head -1), log: $RVIZ_LOG"
  gate_ok
fi

echo ""
echo "=== READY: world=$WORLD  nav2=$WANT_NAV  rviz=$WANT_RVIZ  total $(( $(date +%s) - T_START ))s"
echo "    baseline for comparison (park, default, no rviz):"
echo "      landed 24s | controllers 32s | robot_description 35s | nav2 READY 73s"
