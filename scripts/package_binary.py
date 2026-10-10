#!/usr/bin/env python3
"""Materialize and relocate a built ROS overlay; never package source symlinks."""
import argparse, hashlib, json, os, platform, shutil, subprocess, tarfile
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('workspace',type=Path);p.add_argument('output',type=Path);p.add_argument('--patchelf',default='patchelf');a=p.parse_args()
r=a.workspace.resolve();out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
os_release=dict(line.strip().split('=',1) for line in Path('/etc/os-release').read_text().splitlines() if '=' in line)
ubuntu=os_release['VERSION_ID'].strip('"');distro={'22.04':'humble','24.04':'jazzy'}[ubuntu]
assert platform.machine()=='x86_64'
stage=out/'stage'
if stage.exists():raise SystemExit(f'Stage already exists: {stage}. Use a new output directory.')
stage.mkdir();shutil.copytree(r/'install',stage/'install',symlinks=False)
# Optional privately licensed geometry never belongs in a public runtime bundle.
for folder in (stage/'install').glob('*/share/niihan_description/models/construction_site_chunk5'):
 shutil.rmtree(folder)
libs=stage/'deps/lib';libs.mkdir(parents=True)
# Collect non-distribution shared library dependencies, including transitive ones.
elfs=[f for f in (stage/'install').rglob('*') if f.is_file() and f.open('rb').read(4)==b'\x7fELF']
queue=list(elfs);seen=set()
while queue:
 f=queue.pop()
 if f in seen:continue
 seen.add(f)
 result=subprocess.run(['ldd',str(f)],text=True,capture_output=True)
 for line in result.stdout.splitlines():
  if '=> not found' in line:raise SystemExit(f'Unresolved library in {f}: {line}')
  if '=>' not in line:continue
  name,target=line.strip().split('=>',1);target=target.strip().split(' ',1)[0]
  path=Path(target)
  if not path.is_absolute():continue
  # ROS and Ubuntu libraries are installed by apt; private GLIM dependencies ship.
  if target.startswith(('/usr/local/','/home/','/root/')) or str(r/'.deps') in target:
   dest=libs/name.strip()
   if not dest.exists():shutil.copy2(path.resolve(),dest);queue.append(dest)
for f in [*elfs,*libs.iterdir()]:
 rel=os.path.relpath(libs,f.parent)
 subprocess.run([a.patchelf,'--set-rpath',f'$ORIGIN:$ORIGIN/{rel}',str(f)],check=True)
# A binary runtime bundle must not depend on the producer's source tree.
for f in (stage/'install').rglob('*'):
 if not f.is_file():continue
 if f.suffix in ('.pth','.egg-link') and str(r/'src') in f.read_text():
  raise SystemExit(f'Unrelocatable Python source reference: {f}')
licenses=stage/'licenses';licenses.mkdir()
for name in ['glim','glim_ros2','navigation2','geometry2']:
 folder=r/'src'/name
 for f in folder.glob('*LICENSE*'):
  if f.is_file():shutil.copy2(f,licenses/(name+'-'+f.name))
for name in ['gtsam','gtsam_points']:
 for base in [r/'.deps/src'/name,r/'.deps/jazzy/src'/name]:
  if base.exists():
   for f in base.glob('*LICENSE*'):
    if f.is_file():shutil.copy2(f,licenses/(name+'-'+f.name))
commit=subprocess.check_output(['git','-C',str(r),'rev-parse','HEAD'],text=True).strip()
manifest={'schema':1,'source_commit':commit,'ubuntu':ubuntu,'ros_distro':distro,'architecture':'amd64','python':f'{os.sys.version_info.major}.{os.sys.version_info.minor}','packages':sorted(f.name for f in (stage/'install/share/colcon-core/packages').glob('*')) if (stage/'install/share/colcon-core/packages').exists() else sorted(f.name for f in (stage/'install').iterdir() if f.is_dir()),'submodules':subprocess.check_output(['git','-C',str(r),'submodule','status'],text=True),'build_type':'CPU, viewer off, march_native off','runtime_acceptance':'not certified by packaging alone'}
(stage/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
archive=out/f'niihan-ubuntu{ubuntu}-{distro}-amd64.tar.gz'
with tarfile.open(archive,'w:gz') as t:
 for child in sorted(stage.iterdir()):t.add(child,arcname=child.name)
h=hashlib.sha256(archive.read_bytes()).hexdigest();archive.with_name(archive.name+'.sha256').write_text(h+'  '+archive.name+'\n')
print(archive)
