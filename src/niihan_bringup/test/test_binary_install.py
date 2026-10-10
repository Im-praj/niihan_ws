"""Reject damaged, wrong-platform and unsafe binaries before replacing installs."""
import hashlib,importlib.util,io,json,tarfile
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('binary_installer',ROOT/'scripts/install_binary.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

def archive(tmp_path,member=None,metadata=None):
    p=tmp_path/'bundle.tar.gz'
    with tarfile.open(p,'w:gz') as t:
        for name in ['install','deps','licenses']:
            info=tarfile.TarInfo(name);info.type=tarfile.DIRTYPE;info.mode=0o755;t.addfile(info)
        for name,data in [('manifest.json',json.dumps(metadata or {'ros_distro':'humble'}).encode()),('install/local_setup.bash',b'# setup')]:
            info=tarfile.TarInfo(name);info.size=len(data);t.addfile(info,io.BytesIO(data))
        if member:t.addfile(member)
    return p,hashlib.sha256(p.read_bytes()).hexdigest()

def test_correct_archive_extracts(tmp_path):
    p,h=archive(tmp_path);stage=tmp_path/'stage';stage.mkdir()
    assert m.verify_extract(p,h,stage,{'ros_distro':'humble'})['ros_distro']=='humble'
    assert (stage/'install/local_setup.bash').is_file()

@pytest.mark.parametrize('kind',['checksum','platform','traversal','absolute','symlink'])
def test_invalid_archive_does_not_extract(tmp_path,kind):
    info=None
    if kind in ['traversal','absolute']:info=tarfile.TarInfo('../escape' if kind=='traversal' else '/escape')
    if kind=='symlink':info=tarfile.TarInfo('install/link');info.type=tarfile.SYMTYPE;info.linkname='/tmp'
    p,h=archive(tmp_path,info);stage=tmp_path/'stage';stage.mkdir()
    with pytest.raises(ValueError):m.verify_extract(p,'0'*64 if kind=='checksum' else h,stage,{'ros_distro':'jazzy' if kind=='platform' else 'humble'})
    assert not list(stage.iterdir())
