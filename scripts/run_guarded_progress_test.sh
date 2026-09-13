#!/usr/bin/env bash
# Supervised route test: explicit human confirmation, progress completion, hard timeout.
set -Eeuo pipefail

WS=/home/agilex/competition_rebuild_ws
CAR_WS=/home/agilex/competition_ws
TRAJECTORY=${1:?usage: run_guarded_progress_test.sh TRAJECTORY [MAX_SECONDS]}
MAX_SECONDS=${2:-300}

set +u
source "$CAR_WS/scripts/car_source_env.sh"
source "$WS/install/setup.bash"
set -u

disable_route() {
  ros2 topic pub --once /mission/route_enable std_msgs/msg/Bool \
    "{data: false}" >/dev/null 2>&1 || true
}
trap 'disable_route; exit 130' INT TERM
trap disable_route EXIT

[[ -f "$TRAJECTORY" ]] || { echo "Missing trajectory: $TRAJECTORY"; exit 1; }

last_index=$(
  python3 - "$TRAJECTORY" <<'PY2'
import sys, yaml
data = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
print(len(data["points"]) - 1)
PY2
)

publishers=$(ros2 topic info /cmd_vel 2>/dev/null | awk '/Publisher count:/ {print $3; exit}')
[[ "$publishers" == "1" ]] || {
  echo "REFUSE: /cmd_vel must have exactly one guarded adapter publisher, got ${publishers:-none}"
  exit 1
}

system=$(timeout 5s ros2 topic echo /system_state --once 2>/dev/null || true)
grep -q 'control_mode: 1' <<<"$system" || { echo "REFUSE: chassis is not in CAN command mode"; exit 1; }
grep -q 'error_code: 0' <<<"$system" || { echo "REFUSE: chassis reports an error"; exit 1; }

wait_for_clear_replanner() {
  local clear_count=0
  local deadline=$((SECONDS + 20))
  local stop

  while (( SECONDS < deadline )); do
    stop=$(timeout 4s ros2 topic echo /planning/local_stop_request --once --field data 2>/dev/null || true)
    if [[ "$stop" == "false" ]]; then
      clear_count=$((clear_count + 1))
      if (( clear_count >= 3 )); then
        return 0
      fi
    else
      clear_count=0
    fi
    sleep 1
  done
  return 1
}

wait_for_clear_replanner || {
  echo "REFUSE: local replanner did not remain clear for three samples"
  exit 1
}

read_status() {
  timeout 4s ros2 topic echo /control/status --once --field data 2>/dev/null |
    python3 -c '
import json, sys
d = json.load(sys.stdin)
print(int(d.get("route_progress_index", -1)), d.get("status", ""), str(d.get("route_enabled", False)).lower())
'
}

read -r progress state enabled < <(read_status)
[[ "$state" == "ROUTE_DISABLED" && "$enabled" == "false" ]] || {
  echo "REFUSE: route is not disarmed: $state enabled=$enabled"
  exit 1
}

echo "READY: final_index=$last_index, hard_timeout=${MAX_SECONDS}s"
read -r -p "Clear corridor and observers ready. Type START to enable route: " confirm
[[ "$confirm" == "START" ]] || { echo "Cancelled."; exit 0; }

ros2 topic pub --once /mission/route_enable std_msgs/msg/Bool "{data: true}"
started=$SECONDS

while (( SECONDS - started < MAX_SECONDS )); do
  if read -r progress state enabled < <(read_status); then
    printf '\rprogress=%s/%s state=%s   ' "$progress" "$last_index" "$state"
    if (( progress >= last_index )); then
      echo
      echo "Final trajectory point reached; disabling route."
      disable_route
      exit 0
    fi
  fi
  sleep 1
done

echo
echo "Hard timeout reached; disabling route."
disable_route
exit 1
