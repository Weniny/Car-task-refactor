#!/usr/bin/env bash
# Stops only the processes started by start_dry_safety_test.sh.
set -Eeuo pipefail

PID_FILE=/home/agilex/competition_rebuild_ws/log/dry_safety_test/pids

if [[ ! -f "$PID_FILE" ]]; then
  printf 'No dry-test PID file found. Nothing to stop.\n'
  exit 0
fi

while IFS= read -r pid; do
  [[ -n "$pid" ]] || continue
  if kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid"
    printf 'Sent SIGINT to PID %s\n' "$pid"
  fi
done <"$PID_FILE"

rm -f "$PID_FILE"
printf 'Dry-test stop requested.\n'

