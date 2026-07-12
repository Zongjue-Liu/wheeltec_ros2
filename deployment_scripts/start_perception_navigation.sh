#!/usr/bin/env bash
set -eo pipefail

STOP_SCRIPT="/home/wheeltec/stop_traffic_light.sh"
WORKSPACE="/home/wheeltec/wheeltec_ros2"

systemctl_robot() {
  sudo systemctl "$@"
}

echo "[stage 1/3] stopping old navigation and perception without duplicating systemd services"
systemctl_robot stop wheeltec-perception.service || true
"$STOP_SCRIPT" --keep-frontend --no-serial-hold

echo "[stage 1/3] ensuring the frontend service owns chassis, rosbridge, and web video"
systemctl_robot start wheeltec-frontend.service

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash" 2>/dev/null || true

wait_for_perception_state() {
  timeout 90 python3 - <<'PY'
import sys
import time

import rclpy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node("perception_state_probe")
state = [None]
node.create_subscription(
    String, "/traffic_light/state", lambda msg: state.__setitem__(0, msg.data), 10
)
deadline = time.monotonic() + 85.0
while rclpy.ok() and state[0] is None and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
if state[0] is not None:
    print(state[0])
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if state[0] is not None else 1)
PY
}

# systemd owns the camera and perception processes to prevent duplicate nodes.
echo "[stage 1/3] starting the systemd-managed Astra RGB/depth and perception stack"
systemctl_robot restart wheeltec-perception.service

echo "[stage 1/3] waiting for perception state"
if ! wait_for_perception_state; then
  echo "[stage 1/3] ERROR: /traffic_light/state did not become ready"
  systemctl status wheeltec-perception.service --no-pager || true
  exit 1
fi

cat <<'EOF'

[stage 1/3] Perception is ready. The systemd-managed chassis is running but has received zero velocity; Nav2 is stopped.

Check the STOP target in the debug image, then run:
  /home/wheeltec/start_chassis_only.sh
EOF
