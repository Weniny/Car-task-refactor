#!/usr/bin/env bash
# Starts a dry-only test. It never starts ranger_twist_adapter.
set -Eeuo pipefail

WS=/home/agilex/competition_rebuild_ws
CAR_WS=/home/agilex/competition_ws
TEST_TRAJECTORY=$WS/artifacts/indoor_manual_first_motion_0p8m_trajectory.yaml
STATE_DIR=$WS/log/dry_safety_test
PID_FILE=$STATE_DIR/pids

fail() {
  printf 'PRECHECK FAILED: %s\n' "$*" >&2
  exit 1
}

mkdir -p "$STATE_DIR"
[[ -s "$TEST_TRAJECTORY" ]] || fail "short test trajectory is missing: $TEST_TRAJECTORY"
[[ ! -s "$PID_FILE" ]] || fail "previous PID file exists; run stop_dry_safety_test.sh first"

# ROS setup scripts reference optional variables that may be unset.
set +u
source "$CAR_WS/scripts/car_source_env.sh"
set +u
source "$WS/install/setup.bash"
set -u

if pgrep -af 'rebuild_navigation_dry_run.launch.py|rebuild_proximity_dry_test.launch.py|rebuild_navigation_guarded.launch.py' >/dev/null; then
  fail "a dry-test launch is already running"
fi

publishers=$(ros2 topic info /cmd_vel 2>/dev/null | awk '/Publisher count:/ {print $3}')
if [[ -n "$publishers" && "$publishers" != 0 ]]; then
  fail "/cmd_vel has $publishers publisher(s); refusing dry test"
fi

for topic in /cloud_registered_body /odom /amcl_pose; do
  timeout 5s ros2 topic echo "$topic" --once >/dev/null 2>&1 ||
    fail "no message from $topic; start localization and set initial pose first"
done

rm -f "$PID_FILE"
start_launch() {
  name=$1
  shift
  nohup bash -lc "source '$CAR_WS/scripts/car_source_env.sh'; source '$WS/install/setup.bash'; exec ros2 launch $*" >"$STATE_DIR/$name.log" 2>&1 &
  printf '%s\n' "$!" >>"$PID_FILE"
}

start_launch guarded rebuild_bringup rebuild_navigation_guarded.launch.py \
  trajectory_file:="$TEST_TRAJECTORY" \
  start_chassis_adapter:=false
sleep 4

timeout 5s ros2 topic echo /avoidance/stop_request --once >/dev/null ||
  fail "proximity topic unavailable; inspect $STATE_DIR/guarded.log"
timeout 5s ros2 topic echo /control/status --once >/dev/null ||
  fail "MPPI status unavailable; inspect $STATE_DIR/guarded.log"

printf 'GUARDED 0.8 m DRY TEST READY\n'
printf 'Safety output is /cmd_vel_safe, but the chassis adapter is OFF\n'
printf 'Logs: %s\n' "$STATE_DIR"

