#!/usr/bin/env python3
"""Verify and package the independent candidate; no network or robot control."""
import ast
import difflib
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

root=Path(__file__).resolve().parents[1]
candidate=root/'candidates/forward_rejoin_20261103'
baseline=root/'.codex-build/obstacle-origin-20261103/frozen_baseline'
results=root/'.codex-build/forward-rejoin-20261103/results'
evidence=root/'docs/evidence'
log=[]
for command in [
    [sys.executable,'-m','unittest','discover','-s',str(candidate/'src/competition_planning/test'),'-p','test_forward_rejoin_policy.py','-v'],
    [sys.executable,str(root/'scripts/test_forward_rejoin_integration.py'),'--runtime',str(candidate)],
]:
    run=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    log.append(run.stdout)
    if run.returncode:raise SystemExit(run.stdout)
(evidence/'2026-11-03_forward_rejoin_unit_checks.txt').write_text('\n'.join(log),encoding='utf-8')
for p in candidate.rglob('*.py'):ast.parse(p.read_text(encoding='utf-8'))
for name in ['src/competition_control/competition_control/mppi_controller.py','src/competition_safety/competition_safety/supervisor.py']:
    assert (candidate/name).read_bytes()==(baseline/name).read_bytes(),name
checks={}
for name in ['rolling_34','rolling_66','rolling_171','rolling_172','rolling_173','lag_bias_171','clear_after_stop_171','moving_obstacle_invalidation_171','radius081_171','empty_loop_static_check']:
    data=json.loads((results/(name+'.json')).read_text())
    checks[name]={k:v for k,v in data.items() if k not in ['plans','samples','mode']}
    if name not in ['radius081_171','empty_loop_static_check']:
        assert data['outcome']=='PASSED_SECTION',name
    if name.startswith('rolling_') or name=='lag_bias_171':
        assert data['planner_stop_s']==data['stationary_s']==data['proximity_stop_s']==0,name
assert checks['empty_loop_static_check']['static_reference_dense_clear']
checks.update(status='OFFLINE_PASS_ROS_BUILD_PENDING_FIELD_UNVERIFIED',unit_tests={'policy':10,'integration_logic':6})
(evidence/'2026-11-03_forward_rejoin_checks.json').write_text(json.dumps(checks,ensure_ascii=False,indent=2),encoding='utf-8')
(candidate/'OFFLINE_CHECKS.json').write_text(json.dumps(checks,ensure_ascii=False,indent=2),encoding='utf-8')
changes=[];diff=[]
for p in sorted((candidate/'src').rglob('*.py')):
    rel=p.relative_to(candidate);old=baseline/rel
    if not old.exists() or old.read_bytes()!=p.read_bytes():
        changes.append({'path':rel.as_posix(),'baseline_sha256':hashlib.sha256(old.read_bytes()).hexdigest() if old.exists() else None,'candidate_sha256':hashlib.sha256(p.read_bytes()).hexdigest()})
        diff.extend(difflib.unified_diff(old.read_text(encoding='utf-8').splitlines() if old.exists() else [],p.read_text(encoding='utf-8').splitlines(),fromfile='baseline/'+rel.as_posix(),tofile='candidate/'+rel.as_posix(),lineterm=''))
(evidence/'2026-11-03_forward_rejoin_source_diff.txt').write_text('\n'.join(diff)+'\n',encoding='utf-8')
manifest={'id':'forward_rejoin_20261103','status':checks['status'],'baseline':'verified_050_030_20261102_reconstructed','changes':changes,'files':{}}
for p in sorted(candidate.rglob('*')):
    if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc' and p.name!='CANDIDATE_MANIFEST.json':
        manifest['files'][p.relative_to(candidate).as_posix()]=hashlib.sha256(p.read_bytes()).hexdigest()
(candidate/'CANDIDATE_MANIFEST.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
out=root/'output/candidates';out.mkdir(parents=True,exist_ok=True)
archive=out/'forward_rejoin_20261103.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
    for name in manifest['files']:z.write(candidate/name,name)
    z.write(candidate/'CANDIDATE_MANIFEST.json','CANDIDATE_MANIFEST.json')
shutil.copyfile(root/'scripts/install_forward_rejoin_20261103.py',out/'install_forward_rejoin_20261103.py')
with zipfile.ZipFile(root/'output/diagnostics/forward_rejoin_20261103_results.zip','w',zipfile.ZIP_DEFLATED) as z:
    for p in results.glob('*.json'):z.write(p,p.name)
fingerprint={'archive_sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'bytes':archive.stat().st_size,'files':len(manifest['files']),'status':checks['status']}
(evidence/'2026-11-03_forward_rejoin_archive.json').write_text(json.dumps(fingerprint,indent=2),encoding='utf-8')
print(json.dumps(fingerprint))
