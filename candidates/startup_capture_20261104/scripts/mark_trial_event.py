#!/usr/bin/env python3
"""Timestamp an operator obstacle action in the current recording; no ROS."""
import argparse
from datetime import datetime
import json
from pathlib import Path

ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('event',choices=['obstacle_placed','obstacle_moved_after_stop','obstacle_removed'])
args=ap.parse_args()
root=Path(__file__).resolve().parents[1]
log=(root/'log').resolve()
run=Path((log/'retest_current_run.txt').read_text().strip()).resolve()
if not run.is_relative_to(log) or not run.is_dir():raise SystemExit('Invalid recording directory')
row={'time':datetime.now().astimezone().isoformat(),'event':args.event}
with (run/'operator_events.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(row)+'\n')
print(json.dumps(row))
