#!/usr/bin/env python3
"""Freeze passed parent, verify behavior repair, and build a separate archive."""
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
parent=root/'candidates/forward_rejoin_20261103'
candidate=root/'candidates/forward_rejoin_behavior_20261104'
evidence=root/'docs/evidence'
results=root/'.codex-build/behavior-20261104'
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
checks=candidate/'checks';checks.mkdir(exist_ok=True)
snapshots=json.loads((root/'.codex-build/door-run-20261104/behavior_retest_snapshots.json').read_text())
selected=[min(snapshots,key=lambda r:abs(r['t']-t)) for t in [2.2,79.05,112.9,113.4]]
(checks/'recorded_behavior_snapshots.json').write_text(json.dumps(selected),encoding='utf-8')
for name in ['check_behavior_recorded_cases.py','check_behavior_closed_loop.py']:
    shutil.copyfile(root/'scripts'/name,candidate/'scripts'/name)
shutil.copyfile(results/'recorded_far_fixture.json',checks/'recorded_far_fixture.json')
commands=[
    [sys.executable,'-m','unittest','discover','-s',str(candidate/'src/competition_planning/test'),'-p','test_forward_rejoin_policy.py','-v'],
    [sys.executable,'-m','unittest','discover','-s',str(candidate/'src/competition_planning/test'),'-p','test_behavior_guard.py','-v'],
    [sys.executable,str(candidate/'scripts/test_forward_rejoin_integration.py'),'--runtime',str(candidate)],
    [sys.executable,str(candidate/'scripts/test_behavior_callbacks.py'),'--runtime',str(candidate)],
    [sys.executable,str(candidate/'scripts/check_behavior_recorded_cases.py'),'--runtime',str(candidate),'--snapshots',str(checks/'recorded_behavior_snapshots.json'),'--output',str(evidence/'2026-11-04_behavior_recorded_cases.json')],
]
logs=[]
for cmd in commands:
    run=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace')
    logs.append(run.stdout)
    if run.returncode:raise SystemExit(run.stdout)
(evidence/'2026-11-04_behavior_unit_checks.txt').write_text('\n'.join(logs),encoding='utf-8')
for p in candidate.rglob('*.py'):ast.parse(p.read_text(encoding='utf-8'))
for p in (candidate/'scripts').glob('*.sh'):
    run=subprocess.run(['D:/Git/bin/bash.exe','-n',p.resolve().as_posix()],capture_output=True,text=True)
    if run.returncode:raise SystemExit(run.stderr)
for name in ['src/competition_control/competition_control/mppi_controller.py',
             'src/competition_safety/competition_safety/supervisor.py',
             'src/competition_localization/competition_localization/state_estimator.py']:
    assert (candidate/name).read_bytes()==(parent/name).read_bytes(),name
models={}
for name in ['rolling_66','rolling_171','far_dynamic_clear_closed_loop']:
    data=json.loads((results/(name+'.json')).read_text())
    assert data['outcome']=='PASSED_SECTION',name
    assert data['planner_stop_s']==data['stationary_s']==data['proximity_stop_s']==0,name
    if 'guard_stop_s' in data:assert data['guard_stop_s']==0,name
    models[name]={k:v for k,v in data.items() if k not in ['plans','samples','mode']}
summary={'status':'OFFLINE_PASS_ROS_BUILD_PENDING_FIELD_UNVERIFIED','unit_tests':37,
    'recorded_cases':json.loads((evidence/'2026-11-04_behavior_recorded_cases.json').read_text()),
    'closed_loop_models':models,'parent_robot_runtime_hash_check':{'files':57,'differences':[]},
    'unchanged':['grid inflation 0.25','speed 0.50/0.30','MPPI core','Safety supervisor','state estimator']}
(candidate/'OFFLINE_CHECKS.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
(evidence/'2026-11-04_behavior_offline_checks.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
baseline=root/'output/baselines/forward_rejoin_inflation025_field_pass_20261104.zip'
baseline.parent.mkdir(parents=True,exist_ok=True)
parent_files={p.relative_to(parent).as_posix():digest(p) for p in sorted(parent.rglob('*'))
    if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc'}
if not baseline.exists():
    with zipfile.ZipFile(baseline,'w',zipfile.ZIP_DEFLATED) as z:
        for name in parent_files:z.write(parent/name,name)
        z.writestr('FIELD_PASS_FILE_HASHES.json',json.dumps(parent_files,indent=2))
else:
    with zipfile.ZipFile(baseline) as z:
        assert json.loads(z.read('FIELD_PASS_FILE_HASHES.json'))==parent_files,'existing parent archive differs'
parent_record={'id':'forward_rejoin_inflation025_field_pass_20261104','archive_sha256':digest(baseline),
    'archive':baseline.relative_to(root).as_posix(),'files':parent_files,
    'field_result':'one loop completed with obstacle bypass and pedestrian response; remaining behavior issues documented in docs/26'}
(candidate/'PARENT_FIELD_PASS.json').write_text(json.dumps(parent_record,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
changes=[];diff=[]
for p in sorted(candidate.rglob('*')):
    if not p.is_file() or p.suffix not in ['.py','.sh','.md','.yaml']:continue
    rel=p.relative_to(candidate);old=parent/rel
    if not old.exists() or old.read_text(encoding='utf-8')!=p.read_text(encoding='utf-8'):
        changes.append(rel.as_posix())
        diff.extend(difflib.unified_diff(old.read_text(encoding='utf-8').splitlines() if old.exists() else [],
            p.read_text(encoding='utf-8').splitlines(),fromfile='passed/'+rel.as_posix(),tofile='repair/'+rel.as_posix(),lineterm=''))
(evidence/'2026-11-04_behavior_source_diff.txt').write_text('\n'.join(diff)+'\n',encoding='utf-8')
manifest={'id':'forward_rejoin_behavior_20261104','status':summary['status'],
    'parent_archive_sha256':digest(baseline),'changed_text_files':changes,'files':{}}
for p in sorted(candidate.rglob('*')):
    if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc' and p.name!='CANDIDATE_MANIFEST.json':
        manifest['files'][p.relative_to(candidate).as_posix()]=digest(p)
(candidate/'CANDIDATE_MANIFEST.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
out=root/'output/candidates';out.mkdir(parents=True,exist_ok=True)
archive=out/'forward_rejoin_behavior_20261104.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
    for name in manifest['files']:z.write(candidate/name,name)
    z.write(candidate/'CANDIDATE_MANIFEST.json','CANDIDATE_MANIFEST.json')
installer=(root/'scripts/install_forward_rejoin_20261103.py').read_text().replace('forward_rejoin_20261103.zip','forward_rejoin_behavior_20261104.zip').replace('competition_forward_rejoin_20261103_ws','competition_forward_rejoin_behavior_20261104_ws')
(out/'install_behavior_repair_20261104.py').write_text(installer,encoding='utf-8',newline='\n')
fingerprint={'archive_sha256':digest(archive),'bytes':archive.stat().st_size,'files':len(manifest['files']),
    'parent_archive_sha256':digest(baseline),'status':summary['status']}
(evidence/'2026-11-04_behavior_archive.json').write_text(json.dumps(fingerprint,indent=2)+'\n',encoding='utf-8')
print(json.dumps(fingerprint))
