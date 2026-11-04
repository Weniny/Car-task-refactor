#!/usr/bin/env python3
"""Reject mixed old/candidate Python packages. Read-only, no ROS nodes."""
import importlib
import json
from pathlib import Path

root=Path(__file__).resolve().parents[1]
found={}
for name in ['competition_planning.path_guard','competition_planning.local_trajectory_planner','competition_control.mppi_controller','competition_safety.supervisor','competition_localization.state_estimator']:
    module=importlib.import_module(name)
    path=Path(module.__file__).resolve()
    if not path.is_relative_to(root):
        raise SystemExit('Mixed runtime rejected: '+name+' -> '+str(path))
    found[name]=str(path)
print(json.dumps({'status':'PASS','imports':found},indent=2))
