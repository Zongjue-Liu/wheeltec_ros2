#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="/home/wheeltec/wheeltec_ros2"
LOG_DIR="/home/wheeltec/autostart_logs"
mkdir -p "$LOG_DIR"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash" 2>/dev/null || true

perception_is_ready() {
  timeout 10 python3 - <<'PY'
import sys
import time

import rclpy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node("chassis_perception_probe")
received = [False]
node.create_subscription(
    String, "/traffic_light/state", lambda _msg: received.__setitem__(0, True), 10
)
deadline = time.monotonic() + 8.0
while rclpy.ok() and not received[0] and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if received[0] else 1)
PY
}

chassis_subscriber_is_ready() {
  timeout 7 python3 - <<'PY'
import sys
import time

import rclpy

rclpy.init()
node = rclpy.create_node("chassis_graph_probe")
ready = False
deadline = time.monotonic() + 5.0
while rclpy.ok() and not ready and time.monotonic() < deadline:
    ready = any(
        info.node_name == "wheeltec_robot"
        for info in node.get_subscriptions_info_by_topic("/cmd_vel")
    )
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ready else 1)
PY
}

publish_zero_velocity() {
  timeout 5 python3 - <<'PY'
import time

import rclpy
from geometry_msgs.msg import Twist

rclpy.init()
node = rclpy.create_node("chassis_zero_publisher")
publisher = node.create_publisher(Twist, "/cmd_vel", 10)
end = time.monotonic() + 1.0
while rclpy.ok() and time.monotonic() < end:
    publisher.publish(Twist())
    rclpy.spin_once(node, timeout_sec=0.05)
node.destroy_node()
rclpy.shutdown()
PY
}

echo "[stage 2/3] checking perception"
if ! perception_is_ready; then
  echo "[stage 2/3] ERROR: perception is not ready; run start_perception_navigation.sh first"
  exit 1
fi

echo "[stage 2/3] releasing the serial zero-hold process"
pkill -f "/tmp/wheeltec_serial_zero_hold.py" 2>/dev/null || true
sleep 1

if ! systemctl is-active --quiet wheeltec-frontend.service 2>/dev/null; then
  echo "[stage 2/3] ERROR: wheeltec-frontend.service is not active"
  echo "[stage 2/3] run start_perception_navigation.sh again before this stage"
  exit 1
fi

echo "[stage 2/3] using the chassis already owned by wheeltec-frontend.service"

if ! chassis_subscriber_is_ready; then
  echo "[stage 2/3] ERROR: wheeltec_robot did not subscribe to /cmd_vel"
  "$HOME/stop_traffic_light.sh"
  exit 1
fi

publish_zero_velocity || true

cat <<'EOF'

[stage 2/3] Chassis is ready and has received a zero-velocity command.

Run:
  /home/wheeltec/start_navi.sh
EOF
