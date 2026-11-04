# Behavior repair candidate 20261104

Parent: the 2026-11-04 single-loop field pass with 0.25 m grid inflation.
This isolated workspace has not completed a field motion test.

Changes: discard unnecessary pre-departure detours after fresh clear observations;
allow a rechecked stopping prefix after a far-path search failure; require an
expiring path-specific speed guard in the controller. A held car requires a full
plan before restarting. Collision inflation, speed limits, MPPI core, Safety
supervisor and localization estimator retain the passed parent's settings/code.

Use only forward_rejoin_behavior_050_030_inflation025.launch.py for this repair.
See RUNBOOK.md. Build and import checks do not start hardware or enable a route.
The passed workspace /home/agilex/competition_forward_rejoin_20261103_ws is retained.
