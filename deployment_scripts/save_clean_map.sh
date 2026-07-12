#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="/home/wheeltec/wheeltec_ros2"
STAMP="$(date +%Y%m%d_%H%M%S)"
SESSION_DIR="$(cat /home/wheeltec/current_mapping_session.txt 2>/dev/null || true)"
TRAJECTORY_FILE="$SESSION_DIR/odom_path.csv"
DEFAULT_FULL_BASE="$WORKSPACE/src/wheeltec_clean_nav/maps/WHEELTEC_FULL_$STAMP"
FULL_BASE="${1:-$DEFAULT_FULL_BASE}"
DRIVEN_BASE="${2:-$WORKSPACE/src/wheeltec_clean_nav/maps/WHEELTEC_DRIVEN_$STAMP}"
CORRIDOR_RADIUS="${CORRIDOR_RADIUS:-0.24}"

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash" 2>/dev/null || true
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0

if ! pgrep -f '[a]sync_slam_toolbox_node' >/dev/null; then
  echo "[save-map] ERROR: slam_toolbox is not running"
  exit 1
fi

if [ ! -s "$TRAJECTORY_FILE" ]; then
  echo "[save-map] ERROR: trajectory file is missing: $TRAJECTORY_FILE"
  exit 1
fi

mkdir -p "$(dirname "$FULL_BASE")" "$(dirname "$DRIVEN_BASE")"
echo "[save-map] saving full SLAM map"
ros2 run nav2_map_server map_saver_cli -f "$FULL_BASE" \
  --ros-args \
  -p save_map_timeout:=20.0 \
  -p free_thresh_default:=0.25 \
  -p occupied_thresh_default:=0.65

test -s "$FULL_BASE.yaml"
test -s "$FULL_BASE.pgm"

echo "[save-map] generating driven-only navigation map"
ros2 run wheeltec_clean_nav generate_driven_map \
  --map-yaml "$FULL_BASE.yaml" \
  --trajectory "$TRAJECTORY_FILE" \
  --output-base "$DRIVEN_BASE" \
  --corridor-radius "$CORRIDOR_RADIUS"

test -s "$DRIVEN_BASE.yaml"
test -s "$DRIVEN_BASE.pgm"
printf '%s\n' "$FULL_BASE" > /home/wheeltec/last_full_map.txt
printf '%s\n' "$DRIVEN_BASE" > /home/wheeltec/last_driven_map.txt

cat <<EOF

[save-map] Map saved successfully.

Full localization map:
  $FULL_BASE.yaml
  $FULL_BASE.pgm

Driven-only navigation map:
  $DRIVEN_BASE.yaml
  $DRIVEN_BASE.pgm

Trajectory:
  $TRAJECTORY_FILE

The current navigation map was not overwritten. Unvisited free cells in the
DRIVEN map are unknown and are rejected by allow_unknown=false.
Stop mapping with:
  /home/wheeltec/stop_traffic_light.sh
EOF
