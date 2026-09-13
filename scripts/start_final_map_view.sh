#!/usr/bin/env bash
# Start the finalized map server and its RViz viewer from an IPC GUI terminal.
# ROS setup scripts legitimately reference optional unset environment variables.
set -e

source /opt/ros/humble/setup.bash
source /home/agilex/agilex_ws/install/setup.bash
source /home/agilex/competition_rebuild_ws/install/setup.bash

exec ros2 launch rebuild_bringup rebuild_final_map.launch.py "$@"
