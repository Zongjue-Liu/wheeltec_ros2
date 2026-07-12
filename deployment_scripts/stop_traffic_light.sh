#!/usr/bin/env bash
set +e

WORKSPACE="/home/wheeltec/wheeltec_ros2"
STOP_FRONTEND=1
SERIAL_HOLD=1

for arg in "$@"; do
  case "$arg" in
    --with-frontend|--full)
      STOP_FRONTEND=1
      ;;
    --keep-frontend)
      STOP_FRONTEND=0
      ;;
    --no-serial-hold)
      SERIAL_HOLD=0
      ;;
    -h|--help)
      echo "Usage: /home/wheeltec/stop_traffic_light.sh [--keep-frontend] [--no-serial-hold]"
      echo
      echo "Default: emergency-safe stop. Publish zero velocity, stop traffic-light,"
      echo "write raw serial zero frames, start a serial-zero hold process, and stop"
      echo "navigation, frontend, and chassis ROS driver processes."
      echo "--keep-frontend: keep wheeltec-frontend.service and chassis driver running."
      echo "--no-serial-hold: do not keep the raw serial zero hold process running."
      exit 0
      ;;
  esac
done

source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$WORKSPACE/install/setup.bash" 2>/dev/null || true

# Stop the final navigation gate first so it cannot overwrite the zero Twist
# commands published below while the rest of Nav2 is being terminated.
pkill -f "traffic_light_nav_filter_node.py" 2>/dev/null || true
pkill -f "traffic_light_navigation.launch.py" 2>/dev/null || true
pkill -f "clean_navigation.launch.py" 2>/dev/null || true
pkill -f "clean_mapping.launch.py" 2>/dev/null || true
pkill -x async_slam_toolbox_node 2>/dev/null || true
pkill -f "[m]apping_trajectory_recorder" 2>/dev/null || true
sleep 0.3

stop_serial_zero_holders() {
  pkill -f "/tmp/wheeltec_serial_zero_hold.py" 2>/dev/null || true
}

raw_serial_zero_burst() {
  local seconds="${1:-3.0}"
  ZERO_SECONDS="$seconds" python3 - <<'PY'
import os
import time

port = "/dev/wheeltec_controller"
seconds = float(os.environ.get("ZERO_SECONDS", "3.0"))

if not os.path.exists(port):
    print(f"[stop] raw serial zero skipped: {port} missing")
    raise SystemExit(0)

normal = bytearray([0x7B, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x7D])
bcc = 0
for value in normal[:9]:
    bcc ^= value
normal[9] = bcc

red = bytearray(normal)
red[1] = 0x03
bcc = 0
for value in red[:9]:
    bcc ^= value
red[9] = bcc

try:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
except OSError as exc:
    print(f"[stop] raw serial zero open failed: {exc}")
    raise SystemExit(0)

try:
    count = max(1, int(seconds / 0.02))
    for _ in range(count):
        os.write(fd, normal)
        os.write(fd, red)
        time.sleep(0.02)
    print(f"[stop] raw serial zero sent for {seconds:.1f}s")
finally:
    os.close(fd)
PY
}

start_serial_zero_hold() {
  sudo rm -f /tmp/wheeltec_serial_zero_hold.py \
    /tmp/wheeltec_serial_zero_hold.log 2>/dev/null || true
  cat > /tmp/wheeltec_serial_zero_hold.py <<'PY'
#!/usr/bin/env python3
import os
import signal
import time

running = True

def stop(*_args):
    global running
    running = False

signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)

port = "/dev/wheeltec_controller"
normal = bytearray([0x7B, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x7D])
bcc = 0
for value in normal[:9]:
    bcc ^= value
normal[9] = bcc

red = bytearray(normal)
red[1] = 0x03
bcc = 0
for value in red[:9]:
    bcc ^= value
red[9] = bcc

fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
try:
    while running:
        os.write(fd, normal)
        os.write(fd, red)
        time.sleep(0.02)
finally:
    os.close(fd)
PY
  chmod +x /tmp/wheeltec_serial_zero_hold.py
  nohup python3 /tmp/wheeltec_serial_zero_hold.py > /tmp/wheeltec_serial_zero_hold.log 2>&1 &
  sleep 0.5
  pgrep -af wheeltec_serial_zero_hold.py >/dev/null 2>&1 && \
    echo "[stop] raw serial zero hold is running"
}

echo "[stop] publishing zero velocity"
ZERO_TWIST="{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
for topic in /cmd_vel /red_vel /nav2/cmd_vel /traffic_light/cmd_vel /traffic_light/cmd_vel_test; do
  timeout 2 ros2 topic pub --once "$topic" geometry_msgs/msg/Twist "$ZERO_TWIST" >/dev/null 2>&1 || true
done

for topic in /cmd_vel /red_vel /nav2/cmd_vel /traffic_light/cmd_vel /traffic_light/cmd_vel_test; do
  timeout 4 ros2 topic pub -r 30 "$topic" geometry_msgs/msg/Twist "$ZERO_TWIST" >/dev/null 2>&1 &
done
wait

echo "[stop] writing raw serial zero before killing chassis driver"
stop_serial_zero_holders
raw_serial_zero_burst 4.0

echo "[stop] stopping wheeltec perception systemd service"
sudo systemctl stop wheeltec-perception.service 2>/dev/null || true

if [ "$STOP_FRONTEND" = "1" ]; then
  echo "[stop] stopping wheeltec frontend systemd service"
  sudo systemctl stop wheeltec-frontend.service 2>/dev/null || true
else
  echo "[stop] keeping wheeltec frontend service running"
fi

echo "[stop] stopping traffic-light, camera/depth, and view nodes"
patterns=(
  "clean_navigation.launch.py"
  "clean_mapping.launch.py"
  "async_slam_toolbox_node"
  "mapping_trajectory_recorder"
  "clean_route_runner"
  "traffic_light_navigation.launch.py"
  "traffic_light_nodes.launch.py"
  "traffic_light_color_debug.launch.py"
  "traffic_light_debug_node.py"
  "traffic_light_nav_filter_node.py"
  "traffic_light_velocity_node.py"
  "wheeltec_nav2.launch.py"
  "nav2_waypoint_cycle"
  "component_container_isolated"
  "controller_server"
  "behavior_server"
  "bt_navigator"
  "map_server"
  "amcl"
  "lifecycle_manager"
  "planner_server"
  "smoother_server"
  "velocity_smoother"
  "lslidar_driver_node"
  "rqt_image_view"
  "usb_cam_node_exe"
  "ros2 run usb_cam"
  "wheeltec_camera.launch.py"
  "astra_camera_node"
)

if [ "$STOP_FRONTEND" = "1" ]; then
  patterns+=(
  "wheeltec_frontend.launch.py.*enable_camera:=true"
  "wheeltec_frontend_service.sh"
  "wheeltec_robot_node"
  "robot_state_publisher"
  "joint_state_publisher"
  "imu_filter_madgwick_node"
  "ekf_node"
  "static_transform_publisher"
  "gemini_camera_controller"
  "web_video_server"
  )
fi

kill_pattern() {
  local pat="$1"
  local sig="$2"
  local pid
  pgrep -f "$pat" 2>/dev/null | while read -r pid; do
    [ -n "$pid" ] || continue
    [ "$pid" = "$$" ] && continue
    [ "$pid" = "$PPID" ] && continue
    kill "$sig" "$pid" 2>/dev/null || true
  done
}

for pat in "${patterns[@]}"; do
  kill_pattern "$pat" -TERM
done

sleep 1

for pat in "${patterns[@]}"; do
  kill_pattern "$pat" -KILL
done

echo "[stop] publishing final zero velocity"
for topic in /cmd_vel /red_vel /nav2/cmd_vel /traffic_light/cmd_vel /traffic_light/cmd_vel_test; do
  timeout 2 ros2 topic pub --once "$topic" geometry_msgs/msg/Twist "$ZERO_TWIST" >/dev/null 2>&1 || true
done
raw_serial_zero_burst 2.0

if [ "$SERIAL_HOLD" = "1" ]; then
  echo "[stop] starting raw serial zero hold"
  start_serial_zero_hold
else
  echo "[stop] raw serial zero hold disabled"
fi

echo "[stop] remaining related processes:"
remaining_pattern="traffic_light|nav2|controller_server|behavior_server|bt_navigator|planner_server|velocity_smoother|astra_camera|usb_cam|rqt_image_view|wheeltec_camera"
if [ "$STOP_FRONTEND" = "1" ]; then
  remaining_pattern="${remaining_pattern}|wheeltec_frontend|wheeltec_robot_node|gemini|web_video_server"
fi
pgrep -af "$remaining_pattern" \
  | grep -v "stop_traffic_light.sh" || true
echo "[stop] done"
