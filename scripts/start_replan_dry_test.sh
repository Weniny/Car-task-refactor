#!/usr/bin/env bash
# Isolated local-replanning dry test. It never starts ranger_twist_adapter.
set -Eeuo pipefail

WS=/home/agilex/competition_rebuild_ws
CAR_WS=/home/agilex/competition_ws

fail() { printf 'PRECHECK FAILED: %s\n' "$*" >&2; exit 1; }

set +u
source "$CAR_WS/scripts/car_source_env.sh"
source "$WS/install/setup.bash"
set -u

if pgrep -af 'rebuild_navigation_replan_dry_run.launch.py|rebuild_replan_' >/dev/null; then
  fail "local-replan dry test is already running"
fi

publishers=$(ros2 topic info /cmd_vel 2>/dev/null | awk '/Publisher count:/ {print $3; exit}')
[[ -z "${publishers:-}" || "$publishers" == "0" ]] || fail "/cmd_vel has $publishers publisher(s)"

for topic in /odom /amcl_pose; do
  timeout 5s ros2 topic echo "$topic" --once >/dev/null 2>&1 || fail "no $topic; finish localization first"
done

printf 'REPLAN DRY TEST: chassis adapter remains OFF; route starts disabled.\n'
exec ros2 launch rebuild_bringup rebuild_navigation_replan_dry_run.launch.py