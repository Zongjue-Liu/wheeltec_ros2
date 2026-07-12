#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="/home/wheeltec/wheeltec_ros2"
CAMERA_START_TIMEOUT_SEC=35

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash" 2>/dev/null || true

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

CAMERA_PID=""
PERCEPTION_PID=""

cleanup() {
  [ -z "$PERCEPTION_PID" ] || kill -TERM "$PERCEPTION_PID" 2>/dev/null || true
  [ -z "$CAMERA_PID" ] || kill -TERM "$CAMERA_PID" 2>/dev/null || true
  [ -z "$PERCEPTION_PID" ] || wait "$PERCEPTION_PID" 2>/dev/null || true
  [ -z "$CAMERA_PID" ] || wait "$CAMERA_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

topic_has_frame() {
  timeout 5 python3 - "$1" <<'PY' >/dev/null 2>&1
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

rclpy.init()
node = rclpy.create_node("camera_frame_probe")
received = [False]
node.create_subscription(
    Image,
    sys.argv[1],
    lambda _msg: received.__setitem__(0, True),
    qos_profile_sensor_data,
)
deadline = time.monotonic() + 4.0
while rclpy.ok() and not received[0] and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if received[0] else 1)
PY
}

echo "[perception-service] starting Astra RGB and depth camera"
ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py &
CAMERA_PID=$!

CAMERA_READY=0
for _ in $(seq 1 "$CAMERA_START_TIMEOUT_SEC"); do
  if ! kill -0 "$CAMERA_PID" 2>/dev/null; then
    set +e
    wait "$CAMERA_PID"
    CAMERA_STATUS=$?
    set -e
    echo "[perception-service] ERROR: Astra launch exited before images were ready (status=$CAMERA_STATUS)"
    exit 1
  fi

  if topic_has_frame /camera/color/image_raw && \
      topic_has_frame /camera/depth/image_raw; then
    CAMERA_READY=1
    break
  fi
  sleep 1
done

if [ "$CAMERA_READY" != "1" ]; then
  echo "[perception-service] ERROR: Astra RGB/depth frames were not ready within ${CAMERA_START_TIMEOUT_SEC}s"
  exit 1
fi

echo "[perception-service] starting traffic-light perception in safe test mode"
ros2 launch turn_on_wheeltec_robot traffic_light_nodes.launch.py \
  image_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  output_topic:=/traffic_light/cmd_vel_test &
PERCEPTION_PID=$!

# The service is healthy only while both launch processes remain alive. If
# either side exits, fail the unit so systemd restarts the complete stack.
set +e
wait -n "$CAMERA_PID" "$PERCEPTION_PID"
EXITED_STATUS=$?
set -e
echo "[perception-service] ERROR: camera or perception process exited (status=$EXITED_STATUS)"
exit 1
