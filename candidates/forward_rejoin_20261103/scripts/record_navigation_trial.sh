#!/usr/bin/env bash
# Foreground rosbag recorder. It never enables or disables the route.
set -Eeuo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source "$WS/scripts/source_candidate_runtime.sh"
set -u

LABEL=${1:-full_loop_050straight_030turn}
[[ "$LABEL" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "Invalid recording label" >&2; exit 1; }
EXTRA_TOPICS=()
if [[ "$LABEL" == obstacle_* ]]; then
  EXTRA_TOPICS=(/scan /avoidance/local_costmap)
fi
RUN_DIR="$WS/log/retest_${LABEL}_$(date +%Y%m%d_%H%M%S_%N)"
mkdir -p "$RUN_DIR"
ros2 param dump /rebuild_replan_mppi > "$RUN_DIR/mppi.yaml"
ros2 param dump /rebuild_replan_safety > "$RUN_DIR/safety.yaml"
ros2 param dump /rebuild_local_replanner > "$RUN_DIR/replanner.yaml"
ros2 param dump /rebuild_replan_proximity_stop > "$RUN_DIR/proximity.yaml"
cp "$WS/config/recording/navigation_commands_qos.yaml" "$RUN_DIR/recording_qos.yaml"
cp "$WS/CANDIDATE_MANIFEST.json" "$RUN_DIR/candidate_manifest.json"
date -Is > "$RUN_DIR/start_time.txt"
ip -statistics -details link show can2 > "$RUN_DIR/can_before.txt"
printf '%s\n' "$RUN_DIR" > "$WS/log/retest_current_run.txt"
printf 'Recording: %s\nCtrl+C saves this recording; route state is unchanged.\n' "$RUN_DIR"
exec ros2 bag record -s sqlite3 \
  --qos-profile-overrides-path "$RUN_DIR/recording_qos.yaml" \
  -o "$RUN_DIR/rosbag" \
  /mission/route_enable \
  /control/status /control/tracking_error /control/body_cmd /control/state_valid \
  /planning/local_replan_status /planning/local_stop_request \
  /planning/replan_reference_path /planning/local_trajectory /control/replan_executed_path \
  /avoidance/stop_request /avoidance/proximity_status /avoidance/speed_limit \
  /safety/event /cmd_vel_safe /cmd_vel \
  /odom /system_state /motion_state /amcl_pose /initialpose /tf /tf_static /rosout \
  "${EXTRA_TOPICS[@]}"
