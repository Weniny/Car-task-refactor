#!/usr/bin/env bash
source "$(dirname "${BASH_SOURCE[0]}")/source_candidate_base.sh"
if [[ ! -r "$COMPETITION_FORWARD_WS/install/local_setup.bash" ]]; then
  echo "Candidate is not built: bash $COMPETITION_FORWARD_WS/scripts/build_candidate.sh" >&2
  return 1
fi
source "$COMPETITION_FORWARD_WS/install/local_setup.bash"
