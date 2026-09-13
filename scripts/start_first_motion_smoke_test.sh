#!/usr/bin/env bash
# Supervised 0.8 m physical smoke test.
# This starts the chassis adapter, but MPPI remains disarmed until a separate
# manual /mission/route_enable=true confirmation is received.

set -Eeuo pipefail

WS=/home/agilex/competition_rebuild_ws
CAR_WS=/home/agilex/competition_ws
TRAJECTORY=$WS/artifacts/indoor_manual_first_motion_0p8m_trajectory.yaml

fail() {
    printf 'PRECHECK FAILED: %s\n' "$*" >&2
    exit 1
}

set +u
source "$CAR_WS/scripts/car_source_env.sh"
set +u
source "$WS/install/setup.bash"
set -u

[[ -s "$TRAJECTORY" ]] || fail "missing short trajectory: $TRAJECTORY"

if pgrep -af \
'rebuild_navigation_dry_run.launch.py|rebuild_navigation_guarded.launch.py|ranger_twist_adapter_node' \
>/dev/null; then
    fail "a navigation or chassis-adapter launch is already running"
fi

publishers=$(ros2 topic info /cmd_vel 2>/dev/null | awk '/Publisher count:/ {print $3}')
[[ "$publishers" == "0" ]] || fail "/cmd_vel has ${publishers:-unknown} publisher(s)"

for topic in /cloud_registered_body /odom /amcl_pose /system_state /motion_state /avoidance/stop_request; do
    timeout 5s ros2 topic echo "$topic" --once >/dev/null 2>&1 ||
        fail "no message from $topic"
done

python3 - <<'PYTHON'
from pathlib import Path
import yaml
from competition_planning.artifact_provenance import (
    resolve_trajectory_source_paths,
    validate_source_manifest,
)

ws = Path("/home/agilex/competition_rebuild_ws")
artifact = yaml.safe_load(
    (ws / "artifacts/indoor_manual_first_motion_0p8m_trajectory.yaml")
    .read_text(encoding="utf-8")
)
validate_source_manifest(
    artifact,
    resolve_trajectory_source_paths(
        route_file=ws / "routes/indoor_manual_six_anchor_v3_loop.yaml",
        semantic_map_file=ws / "maps/indoor_manual_final/semantic_map_six_anchor_v3.yaml",
        planning_params_file=ws / "config/planning/semantic_lidar_guarded_six_anchor.yaml",
        optimizer_params_file=ws / "config/planning/continuous_trajectory_low_speed.yaml",
    ),
)
assert artifact.get("ok") is True
assert float(artifact["path_length_m"]) == 0.8
print("TRAJECTORY PROVENANCE OK")
PYTHON

system_state=$(timeout 5s ros2 topic echo /system_state --once) ||
    fail "cannot read /system_state"
grep -qE '^vehicle_state:[[:space:]]*0$' <<<"$system_state" ||
    fail "vehicle is not in normal state; check E-stop"
grep -qE '^control_mode:[[:space:]]*1$' <<<"$system_state" ||
    fail "remote is not in CAN control mode"
grep -qE '^error_code:[[:space:]]*0$' <<<"$system_state" ||
    fail "chassis reports a fault"
grep -qE '^motion_mode:[[:space:]]*0$' <<<"$system_state" ||
    fail "system motion mode is not dual Ackermann"

motion_state=$(timeout 5s ros2 topic echo /motion_state --once) ||
    fail "cannot read /motion_state"
grep -qE '^motion_mode:[[:space:]]*0$' <<<"$motion_state" ||
    fail "reported motion mode is not dual Ackermann"

avoidance=$(timeout 5s ros2 topic echo /avoidance/stop_request --once) ||
    fail "cannot read /avoidance/stop_request"
grep -qE '^data:[[:space:]]*false$' <<<"$avoidance" ||
    fail "proximity stop is active; clear the first 1 m of route"

printf '\nPRECHECKS PASSED\n'
printf 'Trajectory: %s\n' "$TRAJECTORY"
printf 'Adapter will connect /cmd_vel_safe -> /cmd_vel.\n'
printf 'Vehicle remains stationary until manual route_enable=true.\n\n'

exec ros2 launch rebuild_bringup rebuild_navigation_guarded.launch.py \
    trajectory_file:="$TRAJECTORY" \
    start_chassis_adapter:=true
