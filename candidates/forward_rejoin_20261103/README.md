# Forward rejoin candidate 20261103

This is an isolated reconstructed 0.50/0.30 workspace, not a field-accepted release.
See RUNBOOK.md for installation, checks, operator-controlled startup and rollback.

- Never source it together with another competition navigation overlay.
- Run bash scripts/build_candidate.sh first; no ROS processes are started by that script.
- The default launch disconnects the chassis adapter and requires explicit route enable.
- Original control core and Safety are preserved; planning tracking reserve is 0.10 m.
- Default planning radius remains 0.60 m; the 0.81 m experiment was not accepted.
- BASELINE_ORIGIN_MANIFEST.json describes the unmodified starting archive.
- CANDIDATE_MANIFEST.json describes this modified candidate.
