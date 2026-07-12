#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="/home/wheeltec/wheeltec_ros2"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash" 2>/dev/null || true

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# The perception service owns the RGB and depth camera devices.
exec ros2 launch turn_on_wheeltec_robot wheeltec_frontend.launch.py \
  enable_camera:=false
