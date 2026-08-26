#!/bin/bash
# Deploy a config from robot_configs/ into ~/clearpath and regenerate.
#
# Husky_viz/ is the source of truth (configs, worlds, urdf, launch).
# ~/clearpath/ is the runtime target: Clearpath's generators only read
# ~/clearpath/robot.yaml, and write their generated output alongside it.
#
#   usage: apply_config.sh [default|full]
set -eo pipefail   # not -u: ROS setup.bash references unset vars
HV="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="${1:-default}"
SRC="$HV/robot_configs/robot_${NAME}.yaml"

[ -f "$SRC" ] || { echo "no such config: $SRC"; echo "available:"; ls "$HV/robot_configs/"; exit 1; }

source /opt/ros/jazzy/setup.bash
mkdir -p ~/clearpath
cp "$SRC" ~/clearpath/robot.yaml
echo "==> applied robot_${NAME}.yaml"

ros2 run clearpath_generator_common generate_description -s ~/clearpath
ros2 run clearpath_generator_gz     generate_param       -s ~/clearpath >/dev/null
ros2 run clearpath_generator_gz     generate_launch      -s ~/clearpath >/dev/null
echo "==> regenerated URDF, params and launch files"

# report which Gazebo sensors actually survive into SDF - the only reliable
# check that a declared sensor will really publish
xacro ~/clearpath/robot.urdf.xacro namespace:=a200_0000 > /tmp/_hv_check.urdf 2>/dev/null
echo "==> sensors in SDF:"
gz sdf -p /tmp/_hv_check.urdf 2>/dev/null \
  | grep -oE "<sensor name='[a-z0-9_]+' type='[a-z_]+'" \
  | sed "s/<sensor name='/    /; s/' type='/  ->  /; s/'$//"
