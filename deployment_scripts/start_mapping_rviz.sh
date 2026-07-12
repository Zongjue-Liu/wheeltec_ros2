#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/wheeltec/wheeltec_ros2/install/setup.bash 2>/dev/null || true

export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/home/wheeltec/.Xauthority}"

exec rviz2 -d \
  /home/wheeltec/wheeltec_ros2/install/wheeltec_clean_nav/share/wheeltec_clean_nav/rviz/clean_mapping.rviz
