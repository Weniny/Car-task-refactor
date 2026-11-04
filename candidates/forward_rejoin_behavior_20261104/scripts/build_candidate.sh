#!/usr/bin/env bash
set -Eeuo pipefail
set +u
source "$(dirname "${BASH_SOURCE[0]}")/source_candidate_base.sh"
set -u
cd "$COMPETITION_FORWARD_WS"
python3 scripts/verify_candidate.py
colcon build --symlink-install --packages-select competition_control competition_planning competition_safety competition_localization forward_rejoin_bringup --event-handlers console_direct+
set +u
source scripts/source_candidate_runtime.sh
set -u
python3 scripts/check_import_origin.py
python3 -m unittest discover -s src/competition_planning/test -p test_forward_rejoin_policy.py -v
python3 scripts/test_forward_rejoin_integration.py --runtime "$COMPETITION_FORWARD_WS"
python3 -m unittest discover -s src/competition_planning/test -p test_behavior_guard.py -v
python3 scripts/test_behavior_callbacks.py --runtime "$COMPETITION_FORWARD_WS"
ros2 launch forward_rejoin_bringup forward_rejoin_behavior_050_030_inflation025.launch.py --show-args
echo "Build and interface loading checks passed. No navigation or chassis was started."
