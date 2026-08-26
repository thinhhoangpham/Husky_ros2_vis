#!/bin/bash
# ROS 2 Jazzy + Gazebo Harmonic + Clearpath simulator deps  (Ubuntu 24.04 Noble)
set -euo pipefail
log(){ echo -e "\n\033[1;36m==> $*\033[0m"; }

log "Base tools"
sudo apt-get update
sudo apt-get install -y software-properties-common curl gnupg lsb-release git wget
sudo add-apt-repository -y universe

log "ROS 2 apt source"
CODENAME=$(. /etc/os-release && echo "$VERSION_CODENAME")
RAS_VER=$(curl -sSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
          | grep -F '"tag_name"' | awk -F'"' '{print $4}')
echo "    ros-apt-source ${RAS_VER} / ${CODENAME}"
curl -sSL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${RAS_VER}/ros2-apt-source_${RAS_VER}.${CODENAME}_all.deb"
sudo apt-get install -y /tmp/ros2-apt-source.deb

log "Clearpath apt source"
curl -sSL https://packages.clearpathrobotics.com/public.key \
  | sudo gpg --dearmor -o /usr/share/keyrings/clearpath-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/clearpath-archive-keyring.gpg] https://packages.clearpathrobotics.com/stable/ubuntu ${CODENAME} main" \
  | sudo tee /etc/apt/sources.list.d/clearpath-latest.list >/dev/null

sudo apt-get update

log "ROS 2 Jazzy desktop + dev tools (large download, be patient)"
sudo apt-get install -y ros-jazzy-desktop ros-dev-tools python3-colcon-common-extensions

log "Gazebo Harmonic bridge"
sudo apt-get install -y ros-jazzy-ros-gz

log "Clearpath ROS 2 packages"
sudo apt-get install -y ros-jazzy-clearpath-common ros-jazzy-clearpath-config \
                        ros-jazzy-clearpath-desktop || true
# Try the prebuilt simulator; harmless if absent (we build from source next)
sudo apt-get install -y ros-jazzy-clearpath-simulator || echo "  (no binary simulator pkg - will build from source)"

log "rosdep init"
sudo rosdep init 2>/dev/null || true
rosdep update

log "DONE - system install complete"
