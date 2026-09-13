#!/usr/bin/env bash
set -Eeuo pipefail

set +u
source /home/agilex/competition_ws/scripts/car_source_env.sh
source /home/agilex/competition_rebuild_ws/install/setup.bash
set -u

timeout 3s ros2 topic pub --once /mission/route_enable std_msgs/msg/Bool \
  "{data: false}" >/dev/null 2>&1 || true

printf 'Route disable requested. Confirm zero output, then Ctrl+C the dry-test launch terminal.\n'
