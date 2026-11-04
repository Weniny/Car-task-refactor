#!/usr/bin/env python3
"""Extract the candidate into a separate workspace; never overwrite or start ROS."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('--archive',type=Path,default=Path(__file__).with_name('forward_rejoin_20261103.zip'))
ap.add_argument('--target',type=Path,default=Path.home()/'competition_forward_rejoin_20261103_ws')
args=ap.parse_args();target=args.target.resolve()
with zipfile.ZipFile(args.archive) as z:
    manifest=json.loads(z.read('CANDIDATE_MANIFEST.json'))
    for name,expected in manifest['files'].items():
        if not (target/name).resolve().is_relative_to(target):raise SystemExit('Unsafe archive member: '+name)
        if hashlib.sha256(z.read(name)).hexdigest()!=expected:raise SystemExit('Archive hash mismatch: '+name)
    if set(z.namelist())!=set(manifest['files'])|{'CANDIDATE_MANIFEST.json'}:
        raise SystemExit('Archive contains unexpected members')
    if target.exists() and any(target.iterdir()):
        for name,expected in manifest['files'].items():
            p=target/name
            if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=expected:
                raise SystemExit('Existing workspace differs; no files overwritten: '+str(target))
        print('Identical candidate already present:',target)
    else:
        target.mkdir(parents=True,exist_ok=True)
        z.extractall(target)
        print('Candidate extracted:',target)
print('No ROS processes started. Next: cd '+str(target)+' && bash scripts/build_candidate.sh')
