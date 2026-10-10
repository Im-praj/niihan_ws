#!/usr/bin/env python3
"""Install revision-matched release bytes after checksum and archive validation."""
import argparse,hashlib,json,os,platform,posixpath,shutil,subprocess,tarfile,tempfile,urllib.error,urllib.request
from pathlib import Path

def verify_extract(archive,checksum,stage,expected):
    actual=hashlib.sha256(archive.read_bytes()).hexdigest()
    if actual!=checksum:raise ValueError('Binary checksum mismatch; installation aborted.')
    with tarfile.open(archive,'r:gz') as t:
        for m in t.getmembers():
            path=posixpath.normpath(m.name)
            if path.startswith(('../','/')) or path=='..' or path.split('/')[0] not in {'install','deps','licenses','manifest.json'}:
                raise ValueError(f'Unsafe archive path: {m.name}')
            if not (m.isfile() or m.isdir()):
                raise ValueError('Binary archive must contain only regular files and directories')
        # Read metadata before extraction; trusted releases still require exact matching.
        manifest=json.load(t.extractfile('manifest.json'))
        for key,value in expected.items():
            if manifest.get(key)!=value:raise ValueError(f'Binary {key} mismatch: expected {value}, got {manifest.get(key)}')
        t.extractall(stage)
    if not all((stage/x).is_dir() for x in ['install','deps','licenses']):raise ValueError('Incomplete runtime bundle')
    if not (stage/'install/local_setup.bash').is_file():raise ValueError('Missing ROS overlay setup')
    return manifest

def main():
    p=argparse.ArgumentParser();p.add_argument('workspace',type=Path);p.add_argument('--archive',type=Path);p.add_argument('--checksum');a=p.parse_args()
    r=a.workspace.resolve();distro=os.environ['NIIHAN_ROS_DISTRO'];ubuntu={'humble':'22.04','jazzy':'24.04'}[distro]
    assert platform.machine()=='x86_64'
    commit=subprocess.check_output(['git','-C',str(r),'rev-parse','HEAD'],text=True).strip()
    expected={'source_commit':commit,'ubuntu':ubuntu,'ros_distro':distro,'architecture':'amd64','python':f'{os.sys.version_info.major}.{os.sys.version_info.minor}'}
    cache=r/'.setup/downloads';cache.mkdir(parents=True,exist_ok=True)
    if a.archive:
        archive=a.archive.resolve();checksum=a.checksum or archive.with_name(archive.name+'.sha256').read_text().split()[0]
    else:
        name=f'niihan-ubuntu{ubuntu}-{distro}-amd64.tar.gz';base=f'https://github.com/Im-praj/niihan_ws/releases/download/binaries-{commit}/'
        archive=cache/name
        try:
            for file in [name+'.sha256',name]:
                print('Downloading',base+file,flush=True)
                with urllib.request.urlopen(base+file,timeout=120) as response,(cache/(file+'.part')).open('wb') as f:shutil.copyfileobj(response,f)
                (cache/(file+'.part')).replace(cache/file)
        except urllib.error.HTTPError as e:
            raise SystemExit(f'No binary published for this revision/platform (HTTP {e.code}). Check GitHub Actions/release status. Source fallback: ./setup.sh --source')
        checksum=(cache/(name+'.sha256')).read_text().split()[0]
    r.joinpath('.binary').mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(dir=r/'.binary',prefix='incoming-') as tmp:
        stage=Path(tmp);verify_extract(archive,checksum,stage,expected)
        backup=r/'.setup/backups'/next(tempfile._get_candidate_names());backup.mkdir(parents=True)
        # Preserve prior installs; replacing does not follow existing symlinks.
        for src,dest in [(stage/'install',r/'install'),(stage/'deps',r/'.binary/deps'),(stage/'licenses',r/'.binary/licenses'),(stage/'manifest.json',r/'.binary/manifest.json')]:
            if dest.exists() or dest.is_symlink():shutil.move(str(dest),str(backup/dest.name))
            shutil.move(str(src),str(dest))
    (r/'.setup/build-profile').write_text(distro+'\n');(r/'.setup/validated-commit').write_text(commit+'\n')
    print('Checksum and platform verified; binary installed. Previous install preserved:',backup)
if __name__=='__main__':main()
