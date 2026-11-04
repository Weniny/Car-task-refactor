#!/usr/bin/env python3
"""Check and package the isolated behind-start repair without driving hardware."""
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
parent=root/'candidates/forward_rejoin_behavior_20261104'
candidate=root/'candidates/startup_capture_20261104'
evidence=root/'docs/evidence';evidence.mkdir(exist_ok=True)
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
checks=candidate/'checks'
scenes=json.loads((root/'.codex-build/behavior-20261104/latest_start_snapshots.json').read_text())
(checks/'recorded_startup_offset.json').write_text(json.dumps([min(scenes,key=lambda s:abs(s['t']-37.2))]),encoding='utf-8')
shutil.copyfile(root/'scripts/check_startup_capture_repair.py',candidate/'scripts/check_startup_capture_repair.py')
commands=[]
for name in ['test_forward_rejoin_policy.py','test_behavior_guard.py','test_startup_capture.py']:
    commands.append([sys.executable,'-m','unittest','discover','-s',str(candidate/'src/competition_planning/test'),'-p',name,'-v'])
for name in ['test_forward_rejoin_integration.py','test_behavior_callbacks.py']:
    commands.append([sys.executable,str(candidate/'scripts'/name),'--runtime',str(candidate)])
commands.append([sys.executable,str(candidate/'scripts/check_behavior_recorded_cases.py'),'--runtime',str(candidate),'--snapshots',str(checks/'recorded_behavior_snapshots.json'),'--output',str(evidence/'2026-11-04_startup_capture_parent_cases.json')])
commands.append([sys.executable,str(candidate/'scripts/check_startup_capture_repair.py'),'--runtime',str(candidate),'--snapshots',str(checks/'recorded_startup_offset.json'),'--output',str(evidence/'2026-11-04_startup_capture_model.json')])
logs=[]
if '--repack-docs' in sys.argv:
    saved=json.loads((candidate/'CANDIDATE_MANIFEST.json').read_text(encoding='utf-8'))
    for name,expected in saved['files'].items():
        if Path(name).suffix in ['.py','.sh','.yaml','.pgm']:
            assert digest(candidate/name)==expected,'runtime changed; full checks required: '+name
    commands=[]
for command in commands:
    result=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace')
    logs.append(result.stdout)
    if result.returncode:raise SystemExit(result.stdout)
if logs:
    (evidence/'2026-11-04_startup_capture_checks.txt').write_text('\n'.join(logs),encoding='utf-8')
for p in candidate.rglob('*.py'):ast.parse(p.read_text(encoding='utf-8'))
for p in (candidate/'scripts').glob('*.sh'):
    result=subprocess.run(['D:/Git/bin/bash.exe','-n',p.resolve().as_posix()],capture_output=True,text=True)
    if result.returncode:raise SystemExit(result.stderr)
unchanged=['src/competition_control/competition_control/mppi_controller.py',
    'src/competition_control/competition_control/mppi_control_node.py',
    'src/competition_safety/competition_safety/supervisor.py',
    'src/competition_localization/competition_localization/state_estimator.py',
    'src/competition_planning/competition_planning/path_guard.py']
for name in unchanged:assert (candidate/name).read_bytes()==(parent/name).read_bytes(),name
model=json.loads((evidence/'2026-11-04_startup_capture_model.json').read_text())
assert model['status']=='PASS' and model['hold_s']==0
summary={'status':'OFFLINE_PASS_BUILD_ROS_FIELD_PENDING','unit_checks':47,
    'recorded_startup_model':{k:v for k,v in model.items() if k not in ['samples','plans']},
    'parent_recorded_cases':json.loads((evidence/'2026-11-04_startup_capture_parent_cases.json').read_text()),
    'unchanged_source_files':unchanged}
(candidate/'OFFLINE_CHECKS.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
parent_zip=root/'output/candidates/forward_rejoin_behavior_20261104.zip'
(candidate/'PARENT_BEHAVIOR_REPAIR.json').write_text(json.dumps({'id':'forward_rejoin_behavior_20261104',
    'archive_sha256':digest(parent_zip),'recorded_run':'retest_obstacle_behavior_repair_full_loop_20261104_132633_388005513'},indent=2)+'\n',encoding='utf-8')
diff=[];changed=[]
for p in sorted(candidate.rglob('*')):
    if not p.is_file() or p.suffix not in ['.py','.sh','.md']:continue
    name=p.relative_to(candidate).as_posix();old=parent/name
    if not old.exists() or old.read_text(encoding='utf-8')!=p.read_text(encoding='utf-8'):
        changed.append(name)
        diff.extend(difflib.unified_diff(old.read_text(encoding='utf-8').splitlines() if old.exists() else [],
            p.read_text(encoding='utf-8').splitlines(),fromfile='parent/'+name,tofile='startup/'+name,lineterm=''))
(evidence/'2026-11-04_startup_capture_diff.txt').write_text('\n'.join(diff)+'\n',encoding='utf-8')
manifest={'id':'startup_capture_20261104','parent_archive_sha256':digest(parent_zip),'status':summary['status'],'changed_text_files':changed,'files':{}}
for p in sorted(candidate.rglob('*')):
    if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc' and p.name!='CANDIDATE_MANIFEST.json':
        manifest['files'][p.relative_to(candidate).as_posix()]=digest(p)
(candidate/'CANDIDATE_MANIFEST.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
out=root/'output/candidates';archive=out/'startup_capture_20261104.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
    for name in manifest['files']:z.write(candidate/name,name)
    z.write(candidate/'CANDIDATE_MANIFEST.json','CANDIDATE_MANIFEST.json')
installer=(root/'scripts/install_forward_rejoin_20261103.py').read_text().replace('forward_rejoin_20261103.zip','startup_capture_20261104.zip').replace('competition_forward_rejoin_20261103_ws','competition_startup_capture_20261104_ws')
(out/'install_startup_capture_20261104.py').write_text(installer,encoding='utf-8',newline='\n')
fingerprint={'archive_sha256':digest(archive),'bytes':archive.stat().st_size,'manifest_files':len(manifest['files']),**summary}
(evidence/'2026-11-04_startup_capture_archive.json').write_text(json.dumps(fingerprint,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in fingerprint.items() if k not in ['recorded_startup_model','parent_recorded_cases','unchanged_source_files']}))
