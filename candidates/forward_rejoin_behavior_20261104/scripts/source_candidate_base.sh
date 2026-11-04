#!/usr/bin/env bash
# Source only. Sensor/localization terminals may retain the rebuilt environment.
_forward_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source /home/agilex/agilex_ws/install/setup.bash
for _forward_var in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH PATH; do
  _forward_out=""
  IFS=: read -ra _forward_entries <<< "${!_forward_var-}"
  for _forward_entry in "${_forward_entries[@]}"; do
    case "$_forward_entry" in
      /home/agilex/competition_forward_rejoin_20261103_ws/install|/home/agilex/competition_forward_rejoin_20261103_ws/install/*|/home/agilex/competition_ws/install|/home/agilex/competition_ws/install/*|/home/agilex/competition_rebuild_ws/install|/home/agilex/competition_rebuild_ws/install/*|"$_forward_ws"/install|"$_forward_ws"/install/*) continue ;;
    esac
    [[ -n "$_forward_entry" ]] && _forward_out="${_forward_out:+$_forward_out:}$_forward_entry"
  done
  printf -v "$_forward_var" '%s' "$_forward_out"
  export "$_forward_var"
done
export COMPETITION_FORWARD_WS="$_forward_ws"
unset _forward_var _forward_out _forward_entry _forward_entries _forward_ws
