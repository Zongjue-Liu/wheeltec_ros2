#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="/home/wheeltec/wheeltec_ros2"
LOG_DIR="/home/wheeltec/autostart_logs"
LOG_FILE="$LOG_DIR/clean_mapping.log"
STOP_SCRIPT="/home/wheeltec/stop_traffic_light.sh"
mkdir -p "$LOG_DIR"
SESSION_STAMP="$(date +%Y%m%d_%H%M%S)"
SESSION_DIR="/home/wheeltec/mapping_sessions/$SESSION_STAMP"
TRAJECTORY_FILE="$SESSION_DIR/odom_path.csv"
mkdir -p "$SESSION_DIR"
printf '%s\n' "$SESSION_DIR" > /home/wheeltec/current_mapping_session.txt

systemctl_robot() {
  sudo systemctl "$@"
}

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash" 2>/dev/null || true
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0

echo "[mapping] stopping navigation and perception; keeping frontend control"
systemctl_robot stop wheeltec-perception.service || true
"$STOP_SCRIPT" --keep-frontend --no-serial-hold
# A previous emergency stop may have left a raw serial zero writer alive.
# Mapping hands velocity control back to the mobile rosbridge client.
pkill -f '/tmp/wheeltec_serial_zero_hold.py' \
  2>/dev/null || true
sleep 1
echo "[mapping] restarting frontend to reset chassis odometry and EKF origin"
systemctl_robot restart wheeltec-frontend.service
sleep 2

if ! timeout 12 python3 - <<'PY'
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry

rclpy.init()
node = rclpy.create_node("mapping_odom_origin_probe")
pose = [None]

def on_odom(msg):
    q = msg.pose.pose.orientation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    pose[0] = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

node.create_subscription(Odometry, "/odom_combined", on_odom, 10)
deadline = time.monotonic() + 10.0
while rclpy.ok() and pose[0] is None and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
valid = pose[0] is not None
if valid:
    x, y, yaw = pose[0]
    valid = abs(x) < 0.25 and abs(y) < 0.25 and abs(yaw) < 0.35
    print(f"[mapping] odom origin: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.2f} deg")
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if valid else 1)
PY
then
  echo "[mapping] ERROR: odom_combined did not reset close to (0,0,0)"
  echo "[mapping] refusing to create another offset map"
  exit 1
fi

pkill -f 'clean_mapping.launch.py' 2>/dev/null || true
pkill -f '[a]sync_slam_toolbox_node' 2>/dev/null || true
pkill -f '[s]ync_slam_toolbox_node' 2>/dev/null || true
pkill -f '[l]slidar_driver_node' 2>/dev/null || true
sleep 3

echo "[mapping] starting supervised LiDAR and slam_toolbox"
setsid bash -lc "
  source /opt/ros/humble/setup.bash
  source $WORKSPACE/install/setup.bash 2>/dev/null || true
  exec ros2 launch wheeltec_clean_nav clean_mapping.launch.py \
    start_lidar:=true trajectory_file:=$TRAJECTORY_FILE
" >"$LOG_FILE" 2>&1 </dev/null &

if ! timeout 45 python3 - <<'PY'
import time
import sys

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

rclpy.init()
node = rclpy.create_node("clean_mapping_readiness_probe")
ready = {"scan": False, "map": False}
node.create_subscription(
    LaserScan, "/scan", lambda _msg: ready.__setitem__("scan", True),
    qos_profile_sensor_data)
node.create_subscription(
    OccupancyGrid, "/map", lambda _msg: ready.__setitem__("map", True), 10)
deadline = time.monotonic() + 40.0
while rclpy.ok() and not all(ready.values()) and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if all(ready.values()) else 1)
PY
then
  echo "[mapping] ERROR: /scan or /map did not become ready"
  tail -n 120 "$LOG_FILE" || true
  exit 1
fi

if ! pgrep -f '[t]rajectory_recorder' >/dev/null; then
  echo "[mapping] ERROR: trajectory recorder is not running"
  tail -n 80 "$LOG_FILE" || true
  exit 1
fi

cat <<EOF

[mapping] Clean 2D mapping is ready.

Mobile control: wheeltec-frontend.service is active
Maximum recommended speed: 0.05-0.08 m/s
Map origin: verified current robot position (0,0), current heading is +X
Log: $LOG_FILE
Trajectory: $TRAJECTORY_FILE

Optional RViz:
  /home/wheeltec/start_mapping_rviz.sh

After completing the route:
  /home/wheeltec/save_clean_map.sh

Stop everything:
  /home/wheeltec/stop_traffic_light.sh
EOF
