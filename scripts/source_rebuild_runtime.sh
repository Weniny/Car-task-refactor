#!/usr/bin/env bash
# Source this file in every ROS terminal used for the rebuilt navigation stack.

source /opt/ros/humble/setup.bash
source /home/agilex/agilex_ws/install/setup.bash

for var in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH PATH; do
  value="${!var-}"
  filtered="$(tr ':' '\n' <<< "$value" | grep -v '^/home/agilex/competition_ws/install/' | paste -sd: -)"
  printf -v "$var" '%s' "$filtered"
  export "$var"
done

source /home/agilex/competition_rebuild_ws/install/local_setup.bash
