#!/usr/bin/env python3
"""Read-only integrity and import-origin checks; does not start any ROS nodes."""
import hashlib
import json
from pathlib import Path

root=Path(__file__).resolve().parents[1]
manifest=json.loads((root/'CANDIDATE_MANIFEST.json').read_text(encoding='utf-8'))
for name,expected in manifest['files'].items():
    path=(root/name).resolve()
    if not path.is_relative_to(root) or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
        raise SystemExit('Candidate hash mismatch: '+name)
print('Candidate integrity PASS:',manifest['id'],len(manifest['files']),'files')
