"""Environment-scoped, atomic map checkpoints. A map NPZ is self-contained."""
import hashlib,json,os,re,time,uuid
from pathlib import Path
import numpy as np

def atomic_bytes(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_name('.'+path.name+'.'+uuid.uuid4().hex)
    try:
        with tmp.open('wb') as f:f.write(data);f.flush();os.fsync(f.fileno())
        os.replace(tmp,path)
    finally:
        if tmp.exists():tmp.unlink()

def environment_directory(root,name,world):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}',name):raise ValueError('Invalid environment_id')
    directory=Path(root).expanduser()/name;world_hash=hashlib.sha256(Path(world).read_bytes()).hexdigest()
    metadata=directory/'environment.json'
    if metadata.exists():
        info=json.loads(metadata.read_text())
        if info.get('world_sha256')!=world_hash:raise ValueError('Environment world changed; use a new environment_id')
    else:
        atomic_bytes(metadata,json.dumps({'schema':1,'environment_id':name,'world_sha256':world_hash}).encode())
    return directory

def current_map(directory):
    directory=Path(directory);pointer=directory/'current.json'
    if not pointer.exists():return None
    info=json.loads(pointer.read_text());path=(directory/info['file']).resolve()
    if not path.is_relative_to(directory.resolve()):raise ValueError('Map checkpoint escapes environment directory')
    if hashlib.sha256(path.read_bytes()).hexdigest()!=info['sha256']:raise ValueError('Saved map checksum mismatch')
    with np.load(path,allow_pickle=False) as saved:
        if not {'points','grid','resolution','origin','ground_z'}<=set(saved.files):raise ValueError('Incomplete map checkpoint')
        if saved['points'].ndim!=2 or saved['points'].shape[1]!=3 or not len(saved['points']):raise ValueError('Invalid saved point cloud')
    return str(path)

def checkpoint(directory,**arrays):
    directory=Path(directory);snap=directory/'checkpoints'/(time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8]+'.npz')
    snap.parent.mkdir(parents=True,exist_ok=True)
    with snap.open('wb') as f:np.savez_compressed(f,**arrays);f.flush();os.fsync(f.fileno())
    digest=hashlib.sha256(snap.read_bytes()).hexdigest()
    atomic_bytes(directory/'current.json',json.dumps({'schema':1,'file':str(snap.relative_to(directory)),'sha256':digest,'saved_at':time.time()}).encode())
    # Compatibility export for tools/evidence; launcher loads the immutable checkpoint.
    atomic_bytes(directory/'map_3d.npz',snap.read_bytes())
    return str(snap)
