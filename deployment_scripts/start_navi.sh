#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="/home/wheeltec/wheeltec_ros2"
LOG_DIR="/home/wheeltec/autostart_logs"
LOG_FILE="$LOG_DIR/clean_navigation.log"
mkdir -p "$LOG_DIR"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash" 2>/dev/null || true
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
export ROS2CLI_DISABLE_DAEMON=1

topic_message_is_ready() {
  timeout 10 python3 - "$1" "$2" <<'PY'
import importlib
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data

module_name, class_name = sys.argv[2].split("/")
message_type = getattr(importlib.import_module(module_name + ".msg"), class_name)
rclpy.init()
node = rclpy.create_node("clean_nav_topic_probe")
received = [False]
node.create_subscription(
    message_type,
    sys.argv[1],
    lambda _msg: received.__setitem__(0, True),
    qos_profile_sensor_data,
)
deadline = time.monotonic() + 8.0
while rclpy.ok() and not received[0] and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if received[0] else 1)
PY
}

topic_has_node() {
  timeout 8 python3 - "$1" "$2" "$3" <<'PY'
import sys
import time

import rclpy

topic, endpoint, expected_node = sys.argv[1:4]
rclpy.init()
node = rclpy.create_node("clean_nav_graph_probe")
ready = False
deadline = time.monotonic() + 6.0
while rclpy.ok() and not ready and time.monotonic() < deadline:
    if endpoint == "subscriber":
        infos = node.get_subscriptions_info_by_topic(topic)
    else:
        infos = node.get_publishers_info_by_topic(topic)
    ready = any(info.node_name == expected_node for info in infos)
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ready else 1)
PY
}

cmd_vel_output_is_safe() {
  timeout 35 python3 - <<'PY'
import sys
import time

import rclpy

rclpy.init()
node = rclpy.create_node("clean_nav_cmd_vel_probe")
names = []
stable_samples = 0
deadline = time.monotonic() + 30.0
while rclpy.ok() and time.monotonic() < deadline:
    names = [info.node_name for info in node.get_publishers_info_by_topic("/cmd_vel")]
    if names.count("traffic_light_nav_filter") == 1 and len(names) == 1:
        stable_samples += 1
    else:
        stable_samples = 0
    if stable_samples >= 3:
        break
    rclpy.spin_once(node, timeout_sec=0.5)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if stable_samples >= 3 else 1)
PY
}

lifecycle_is_active() {
  timeout 8 python3 - "$1" <<'PY'
import sys

import rclpy
from lifecycle_msgs.srv import GetState

rclpy.init()
node = rclpy.create_node("clean_nav_lifecycle_probe")
client = node.create_client(GetState, f"/{sys.argv[1]}/get_state")
active = False
if client.wait_for_service(timeout_sec=5.0):
    future = client.call_async(GetState.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    active = future.done() and future.result().current_state.label == "active"
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if active else 1)
PY
}

echo "[stage 3/3] checking perception and chassis"
topic_message_is_ready /traffic_light/state std_msgs/String || {
  echo "[stage 3/3] ERROR: perception state is not ready"; exit 1; }
topic_has_node /cmd_vel subscriber wheeltec_robot || {
  echo "[stage 3/3] ERROR: chassis is not subscribed to /cmd_vel"; exit 1; }

echo "[stage 3/3] stopping old navigation only"
pkill -f 'traffic_light_navigation.launch.py' 2>/dev/null || true
pkill -f 'clean_navigation.launch.py' 2>/dev/null || true
pkill -f '[l]slidar_driver_node' 2>/dev/null || true
pkill -f 'traffic_light_nav_filter_node.py' 2>/dev/null || true
# A terminated ros2 launch parent can leave non-composed Nav2 children orphaned.
# Remove only navigation/localization executables; chassis and perception stay up.
for process in \
  amcl map_server controller_server smoother_server planner_server \
  behavior_server bt_navigator waypoint_follower lifecycle_manager; do
  pkill -x "$process" 2>/dev/null || true
done
sleep 3

echo "[stage 3/3] starting clean AMCL + Nav2 + supervised lidar + velocity filter"
setsid bash -lc "
  source /opt/ros/humble/setup.bash
  source $WORKSPACE/install/setup.bash 2>/dev/null || true
  exec ros2 launch wheeltec_clean_nav clean_navigation.launch.py \
    start_lidar:=true start_route:=false
" >"$LOG_FILE" 2>&1 </dev/null &

READY=0
for _ in $(seq 1 30); do
  # Localization can become active before navigation. With manual AMCL
  # initialization, planner activation intentionally waits for map -> base.
  if [ "$(grep -c 'Managed nodes are active' "$LOG_FILE" 2>/dev/null)" -ge 1 ] && \
      pgrep -f '[l]slidar_driver_node' >/dev/null && \
      pgrep -x amcl >/dev/null && \
      pgrep -x map_server >/dev/null && \
      pgrep -x planner_server >/dev/null && \
      pgrep -f '[t]raffic_light_nav_filter_node.py' >/dev/null; then
    READY=1
    break
  fi
  sleep 2
done

if [ "$READY" != 1 ]; then
  echo "[stage 3/3] ERROR: clean navigation did not become ready"
  tail -n 100 "$LOG_FILE" || true
  exit 1
fi

echo "[stage 3/3] validating scan, lifecycle, and velocity endpoints"
topic_message_is_ready /scan sensor_msgs/LaserScan || {
  echo "[stage 3/3] ERROR: lidar has no /scan data"; exit 1; }
lifecycle_is_active amcl || {
  echo "[stage 3/3] ERROR: AMCL localization is not active"; exit 1; }

if ! cmd_vel_output_is_safe; then
  echo "[stage 3/3] ERROR: /cmd_vel publisher safety check failed"
  exit 1
fi

cat <<EOF

[stage 3/3] Clean navigation nodes are ready; localization is waiting.

AMCL localization: active, waiting for 2D Pose Estimate
Map and lidar:      active
Nav2:               waiting for map-to-base TF, then activates automatically
Safety output:      /nav2/cmd_vel -> filter -> /cmd_vel
Automatic route:   disabled

Before setting any goal in RViz:
  1. Click "2D Pose Estimate" at the robot's real map position.
  2. Drag the arrow in the robot's real forward direction.
  3. Confirm the red LaserScan walls overlap the black map walls.
  4. Only then click "2D Goal Pose".

Until the initial pose is set, do not send a navigation goal.
Stop everything: /home/wheeltec/stop_traffic_light.sh
Log: $LOG_FILE
EOF
