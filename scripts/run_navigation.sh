#!/usr/bin/env bash
# One-command supervised startup for the validated R=1.2 m navigation baseline.
set -Eeuo pipefail

WS=/home/agilex/competition_rebuild_ws
RUNTIME="$WS/scripts/source_rebuild_runtime.sh"
MISSION=full_loop
RECORD=true
RVIZ=true
LAUNCH_PID=
BAG_PID=
ROUTE_ARMED=false
RUN_DIR=

lifecycle_state() {
  timeout 5s ros2 lifecycle get "$1" 2>/dev/null || true
}

activate_lifecycle_node() {
  local node_name=$1
  local state=

  for _ in $(seq 1 30); do
    state=$(lifecycle_state "$node_name")
    [[ -n "$state" ]] && break
    sleep 1
  done
  [[ -n "$state" ]] || fail "$node_name lifecycle service did not become ready"

  printf 'Preparing lifecycle node: %s (%s)\n' "$node_name" "$state"
  if grep -q '^active ' <<<"$state"; then
    return
  fi
  if grep -q '^unconfigured ' <<<"$state"; then
    for _ in $(seq 1 3); do
      timeout 10s ros2 lifecycle set "$node_name" configure >/dev/null 2>&1 || true
      state=$(lifecycle_state "$node_name")
      grep -q '^inactive ' <<<"$state" && break
      sleep 1
    done
    grep -q '^inactive ' <<<"$state" || fail "could not configure $node_name"
  fi
  if grep -q '^inactive ' <<<"$state"; then
    for _ in $(seq 1 3); do
      timeout 10s ros2 lifecycle set "$node_name" activate >/dev/null 2>&1 || true
      state=$(lifecycle_state "$node_name")
      grep -q '^active ' <<<"$state" && break
      sleep 1
    done
  fi
  grep -q '^active ' <<<"$state" ||
    fail "$node_name did not become active"
}

usage() {
  cat <<'EOF'
Usage: run_navigation.sh [--mission full_loop|30m] [--no-record] [--no-rviz]

The script starts the complete stack, initializes AMCL, runs preflight checks,
and waits for an explicit START confirmation before enabling the route.
EOF
}

fail() {
  printf 'STARTUP FAILED: %s\n' "$*" >&2
  exit 1
}

while (($#)); do
  case "$1" in
    --mission)
      (($# >= 2)) || fail "--mission requires a value"
      MISSION=$2
      shift 2
      ;;
    --no-record)
      RECORD=false
      shift
      ;;
    --no-rviz)
      RVIZ=false
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      fail "unknown argument: $1"
      ;;
  esac
done

case "$MISSION" in
  full_loop)
    TRAJECTORY="$WS/artifacts/indoor_manual_six_anchor_v3_radius120_turnrate250_020mps_trajectory.yaml"
    ;;
  30m)
    TRAJECTORY="$WS/artifacts/indoor_manual_first_segment_30m_radius120_turnrate250_020mps_trajectory.yaml"
    ;;
  *)
    fail "unknown mission '$MISSION'; use full_loop or 30m"
    ;;
esac

cleanup() {
  status=$?
  trap - EXIT INT TERM HUP
  set +e
  if [[ -n "$RUN_DIR" ]]; then
    printf '\nStopping navigation safely...\n'
  fi
  if command -v ros2 >/dev/null 2>&1; then
    timeout 4s ros2 topic pub --once /mission/route_enable \
      std_msgs/msg/Bool "{data: false}" >/dev/null 2>&1 || true
  fi
  ROUTE_ARMED=false
  if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
  fi
  if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    # The launch stack runs in its own session, so this reaches every node
    # without signalling the operator's terminal or leaving child processes.
    kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
    for _ in $(seq 1 10); do
      kill -0 "$LAUNCH_PID" 2>/dev/null || break
      sleep 1
    done
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
      printf 'Navigation launch did not exit after TERM; forcing shutdown.\n' >&2
      kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
    fi
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  if [[ -n "$RUN_DIR" ]]; then
    printf 'Stopped. Logs: %s\n' "$RUN_DIR"
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM HUP

[[ -f "$RUNTIME" ]] || fail "runtime setup is missing: $RUNTIME"
[[ -s "$TRAJECTORY" ]] || fail "trajectory is missing: $TRAJECTORY"

# ROS setup scripts legitimately reference optional unset variables.
set +u
source "$RUNTIME"
set -u

if pgrep -af \
  'rebuild_full_navigation.launch.py|rebuild_navigation_replan_|rebuild_mapping.launch.py|rebuild_final_map.launch.py|nav2_amcl|fastlio_mapping|ranger_base_node|pointcloud_to_laserscan_node|local_replanner_node|mppi_control_node|safety_node|ranger_twist_adapter_node|proximity_stop_node' \
  >/dev/null; then
  fail "navigation-related processes already exist; stop the previous run first"
fi

ip link show can2 2>/dev/null | grep -qE '<[^>]*UP[^>]*>' ||
  fail "can2 is not UP"

mkdir -p "$WS/log"
RUN_DIR="$WS/log/unified_${MISSION}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"

printf 'Starting unified navigation for mission: %s\n' "$MISSION"
# Keep high-rate sensor logs in a file so operator confirmations stay visible.
setsid ros2 launch rebuild_bringup rebuild_full_navigation.launch.py \
  trajectory_file:="$TRAJECTORY" \
  rviz:="$RVIZ" \
  start_chassis_adapter:=true \
  >"$RUN_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 45); do
  kill -0 "$LAUNCH_PID" 2>/dev/null ||
    fail "unified launch exited; inspect $RUN_DIR/launch.log"
  if [[ -n "$(lifecycle_state /map_server)" ]]; then
    break
  fi
  sleep 1
done
activate_lifecycle_node /map_server

for _ in $(seq 1 45); do
  kill -0 "$LAUNCH_PID" 2>/dev/null ||
    fail "unified launch exited; inspect $RUN_DIR/launch.log"
  if lifecycle_state /amcl | grep -q '^active '; then
    break
  fi
  sleep 1
done
lifecycle_state /amcl | grep -q '^active ' ||
  fail "AMCL did not become active within 45 seconds"

# Let AMCL receive the newly activated transient-local map before it accepts
# the fixed-start pose. A pose sent during that transition is discarded.
sleep 2

amcl_count=$(ros2 node list 2>/dev/null | grep -cx '/amcl' || true)
scan_count=$(ros2 node list 2>/dev/null | grep -cx '/rebuild_localization_scan' || true)
[[ "$amcl_count" == 1 ]] || fail "expected one /amcl node, found $amcl_count"
[[ "$scan_count" == 1 ]] ||
  fail "expected one /rebuild_localization_scan node, found $scan_count"

read -r -p \
  "Confirm the vehicle is physically at the marked start; type AT_START: " \
  start_confirmation
[[ "$start_confirmation" == AT_START ]] || fail "fixed-start initialization cancelled"

printf 'Publishing the validated fixed-start pose...\n'
ros2 topic pub --times 8 -r 2 /initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: -16.083, y: 2.309, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.998045, w: 0.062506}}, covariance: [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.03]}}" \
  >/dev/null

sleep 4

require_param() {
  node=$1
  parameter=$2
  expected=$3
  actual=$(ros2 param get "$node" "$parameter" 2>/dev/null | awk '{print $NF}')
  [[ -n "$actual" ]] || fail "cannot read $node.$parameter"
  awk -v actual="$actual" -v expected="$expected" \
    'BEGIN {delta = actual - expected; exit !((-0.000001 < delta) && (delta < 0.000001))}' ||
    fail "$node.$parameter is $actual, expected $expected"
}

require_param /rebuild_replan_mppi max_speed_mps 0.20
require_param /rebuild_replan_mppi max_curvature_rate_1pmps 2.50
require_param /rebuild_replan_safety recovery_speed_mps 0.15
require_param /rebuild_replan_safety max_lateral_error_m 0.40
require_param /rebuild_replan_safety max_heading_error_deg 45.0

python3 "$WS/scripts/navigation_preflight.py" --timeout 20

if [[ "$RECORD" == true ]]; then
  ros2 bag record \
    -o "$RUN_DIR/rosbag" \
    /control/status /control/tracking_error /control/body_cmd \
    /planning/local_replan_status /planning/local_stop_request \
    /avoidance/stop_request /safety/event \
    /cmd_vel_safe /cmd_vel /odom /motion_state \
    >"$RUN_DIR/rosbag.log" 2>&1 &
  BAG_PID=$!
  sleep 2
  kill -0 "$BAG_PID" 2>/dev/null || fail "rosbag failed to start"
fi

printf '\nREADY: mission=%s trajectory=%s\n' "$MISSION" "$TRAJECTORY"
printf 'Confirm RViz alignment, a clear area, and a remote stop operator.\n'
read -r -p "Type START and press Enter to enable the route: " confirmation
[[ "$confirmation" == START ]] || fail "operator cancelled startup"

ros2 topic pub --once /mission/route_enable \
  std_msgs/msg/Bool "{data: true}" >/dev/null
ROUTE_ARMED=true

printf 'ROUTE ENABLED. Press Ctrl+C to stop and save logs.\n'
wait "$LAUNCH_PID"
