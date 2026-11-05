#!/usr/bin/env python3
"""Read a car dependency snapshot over SSH; does not start ROS nodes or write remotely."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile

REMOTE = r'''
import hashlib, io, json, os, platform, subprocess, sys, tarfile
from pathlib import Path
from datetime import datetime
home = Path('/home/agilex')
candidate = home / 'competition_startup_capture_20261104_ws'
manifest = json.loads((candidate/'CANDIDATE_MANIFEST.json').read_text())
candidate_hashes = {}
for rel in manifest['files']:
    p = (candidate/rel).resolve()
    if not p.is_relative_to(candidate):
        raise ValueError('manifest escape')
    candidate_hashes[rel] = hashlib.sha256(p.read_bytes()).hexdigest()
packages = ['rebuild_bringup','fast_lio','livox_ros_driver2','ranger_bringup','ranger_base','nav2_amcl','nav2_map_server','pointcloud_to_laserscan']
cmd = "source /home/agilex/competition_rebuild_ws/scripts/source_rebuild_runtime.sh; python3 -c 'import json; from ament_index_python.packages import get_package_prefix; print(json.dumps({p:get_package_prefix(p) for p in " + repr(packages).replace("'", '"') + "}))'"
prefix_run = subprocess.run(['bash','-c',cmd],capture_output=True,text=True)
if prefix_run.returncode:
    raise RuntimeError(prefix_run.stderr)
prefixes = json.loads(prefix_run.stdout.strip().splitlines()[-1])
roots = [home/'competition_rebuild_ws'/d for d in ['scripts','config/mapping','config/localization','maps/indoor_manual_final','src/rebuild_bringup']]
roots += [home/'agilex_ws/src'/d for d in ['FAST_LIO','livox_ros_driver2','ranger_ros2','ugv_sdk']]
roots += [Path(prefix)/'share'/pkg for pkg,prefix in prefixes.items()]
excluded_dirs = {'.git','build','install','log','__pycache__','.pytest_cache','PCD','pcd','Log','logs'}
excluded_suffixes = {'.pcd','.bag','.db3','.o','.a','.so','.pyc','.lvx','.png','.jpg','.jpeg','.mp4','.pdf'}
files = {}
for relative in [*manifest['files'], 'CANDIDATE_MANIFEST.json']:
    p = candidate/relative
    files[str(p)] = p.read_bytes()
skipped = []
for root in roots:
    if not root.exists():
        raise FileNotFoundError(str(root))
    for parent, dirs, names in os.walk(root):
        dirs[:] = sorted(d for d in dirs if d not in excluded_dirs)
        for name in sorted(names):
            p = Path(parent)/name
            if p.suffix.lower() in excluded_suffixes or p.stat().st_size > 20_000_000:
                skipped.append(str(p)); continue
            if p.is_file():
                files[str(p)] = p.read_bytes()
binary_hashes = {}
for pkg,prefix in prefixes.items():
    base = Path(prefix)/'lib'
    binaries = list((base/pkg).glob('*')) + list(base.glob('*'+pkg+'*.so*'))
    for p in binaries:
        if p.is_file():
            binary_hashes[str(p)] = {'resolved':str(p.resolve()),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
for pattern in ['*ugv*','*livox*']:
    for p in Path('/usr/local/lib').glob(pattern):
        if p.is_file():
            binary_hashes[str(p)] = {'resolved':str(p.resolve()),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
git = {}
for root in [home/'agilex_ws/src'/d for d in ['FAST_LIO','livox_ros_driver2','ranger_ros2','ugv_sdk']]:
    git[str(root)] = {}
    for key,args in [('head',['rev-parse','HEAD']),('status',['status','--porcelain'])]:
        result = subprocess.run(['git','-C',str(root),*args],capture_output=True,text=True)
        git[str(root)][key] = result.stdout.strip() if result.returncode == 0 else result.stderr.strip()
dpkg = subprocess.run(['dpkg-query','-W','-f=${Package}\t${Version}\n'],capture_output=True,text=True)
versions = [line for line in dpkg.stdout.splitlines() if line.startswith(('ros-humble-','libpcl','libeigen','livox'))]
meta = {'captured_at_car':datetime.now().astimezone().isoformat(),'hostname':platform.node(),'system':platform.platform(),
        'scope':'File snapshot only; no live navigation acceptance or new physical tests. Built binaries fingerprinted, not bundled.',
        'package_prefixes':prefixes,'candidate_id':manifest['id'],'candidate_hashes':candidate_hashes,
        'candidate_manifest_mismatches':[r for r,h in candidate_hashes.items() if h != manifest['files'][r]],
        'dependency_file_hashes':{p:hashlib.sha256(data).hexdigest() for p,data in files.items()},
        'binary_fingerprints':binary_hashes,'dependency_git':git,'ros_debian_versions':versions,'excluded_files':skipped}
with tarfile.open(fileobj=sys.stdout.buffer,mode='w|gz',dereference=True) as archive:
    for name,data in sorted(files.items()):
        info = tarfile.TarInfo('files/'+name.lstrip('/')); info.size=len(data)
        archive.addfile(info,io.BytesIO(data))
    data = (json.dumps(meta,ensure_ascii=False,indent=2)+'\n').encode()
    info = tarfile.TarInfo('SNAPSHOT.json'); info.size=len(data)
    archive.addfile(info,io.BytesIO(data))
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--identity', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reviewed-rviz-sha256', help='Explicitly reviewed car RViz display-state difference; all other differences still fail')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    archive = args.output / 'car_dependencies.tar.gz'
    if archive.exists():
        parser.error('Snapshot already exists; choose a new output directory')
    with archive.open('wb') as output:
        result = subprocess.run(['ssh','-T','-o','BatchMode=yes','-o','ConnectTimeout=20','-i',str(args.identity),args.host,'python3 -'], input=REMOTE.encode(),stdout=output,stderr=subprocess.PIPE)
    if result.returncode:
        raise RuntimeError(result.stderr.decode(errors='replace'))
    with tarfile.open(archive) as bundle:
        meta = json.load(bundle.extractfile('SNAPSHOT.json'))
        for path, expected in meta['dependency_file_hashes'].items():
            actual = hashlib.sha256(bundle.extractfile('files/'+path.lstrip('/')).read()).hexdigest()
            if actual != expected:
                raise ValueError('Archive content hash mismatch: '+path)
    root = Path(__file__).resolve().parents[1]
    candidate = root / 'candidates/startup_capture_20261104'
    local_mismatches = [rel for rel,h in meta['candidate_hashes'].items() if not (candidate/rel).is_file() or hashlib.sha256((candidate/rel).read_bytes()).hexdigest()!=h]
    meta['local_candidate_mismatches'] = local_mismatches
    reviewed_rel = 'src/forward_rejoin_bringup/rviz/navigation_debug_view.rviz'
    differences = set(local_mismatches) | set(meta['candidate_manifest_mismatches'])
    reviewed = bool(differences == {reviewed_rel} and args.reviewed_rviz_sha256 and meta['candidate_hashes'][reviewed_rel] == args.reviewed_rviz_sha256)
    meta['reviewed_display_only_exception'] = {'path':reviewed_rel,'sha256':args.reviewed_rviz_sha256} if reviewed else None
    meta['archive_sha256'] = hashlib.sha256(archive.read_bytes()).hexdigest()
    meta['archive_bytes'] = archive.stat().st_size
    (args.output/'SNAPSHOT.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'dependency_files':len(meta['dependency_file_hashes']),'candidate_files':len(meta['candidate_hashes']), 'remote_candidate_mismatches':meta['candidate_manifest_mismatches'],'local_candidate_mismatches':local_mismatches,'archive_bytes':meta['archive_bytes'],'archive_sha256':meta['archive_sha256']},ensure_ascii=False))
    if differences and not reviewed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
