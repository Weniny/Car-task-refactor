#!/usr/bin/env bash
# Record the mapping data path without commanding vehicle motion.
set -euo pipefail

DURATION_SEC="${1:-45}"
if ! [[ "$DURATION_SEC" =~ ^[1-9][0-9]*$ ]]; then
  echo "Usage: $0 [positive-duration-seconds]" >&2
  exit 2
fi

REBUILD_WS="${REBUILD_WS:-/home/agilex/competition_rebuild_ws}"
source /opt/ros/humble/setup.bash
source /home/agilex/agilex_ws/install/setup.bash
source "$REBUILD_WS/install/setup.bash"

STAMP="$(date '+%Y%m%d_%H%M%S')"
OUTPUT_DIR="$REBUILD_WS/recordings/mapping_diagnostics/${STAMP}_raw"
mkdir -p "$(dirname "$OUTPUT_DIR")"

echo "Recording ${DURATION_SEC}s of mapping diagnostics to: $OUTPUT_DIR"
echo "This command only records ROS topics; it does not send motion commands."

set +e
timeout --signal=INT "$DURATION_SEC" ros2 bag record -o "$OUTPUT_DIR" \
  /livox/lidar \
  /livox/imu \
  /Odometry \
  /cloud_registered_body \
  /scan \
  /map \
  /path \
  /tf \
  /tf_static
STATUS=$?
set -e

if [[ "$STATUS" -ne 0 && "$STATUS" -ne 124 ]]; then
  exit "$STATUS"
fi

ros2 bag info "$OUTPUT_DIR"
echo "Diagnostic recording completed: $OUTPUT_DIR"
