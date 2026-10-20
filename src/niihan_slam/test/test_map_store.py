import json
from pathlib import Path
import numpy as np
import pytest
from niihan_slam.map_store import checkpoint,current_map,environment_directory

def arrays(x=1.):
    return dict(points=np.array([[x,2.,3.]]),grid=np.zeros((2,2),dtype=np.int8),resolution=.1,origin=-.1,ground_z=0.)

def test_checkpoint_survives_restart_and_preserves_previous_trial(tmp_path):
    one=checkpoint(tmp_path,**arrays());two=checkpoint(tmp_path,**arrays(4.))
    assert one!=two and Path(one).exists()
    assert current_map(tmp_path)==two
    assert np.load(two)['points'][0,0]==4.
    assert np.load(tmp_path/'map_3d.npz')['points'][0,0]==4.

def test_corrupted_map_is_rejected_instead_of_silently_remapped(tmp_path):
    path=Path(checkpoint(tmp_path,**arrays()));path.write_bytes(b'corrupted')
    with pytest.raises(ValueError,match='checksum'):current_map(tmp_path)

def test_environment_change_requires_a_different_identity(tmp_path):
    world=tmp_path/'world.sdf';world.write_text('site A')
    directory=environment_directory(tmp_path/'maps','site-a',world)
    assert current_map(directory) is None
    world.write_text('site B')
    with pytest.raises(ValueError,match='new environment_id'):environment_directory(tmp_path/'maps','site-a',world)
    assert environment_directory(tmp_path/'maps','site-b',world)!=directory

def test_environment_id_cannot_escape_storage(tmp_path):
    with pytest.raises(ValueError):environment_directory(tmp_path,'../outside',tmp_path/'world')

def test_checkpoint_pointer_cannot_escape_storage(tmp_path):
    (tmp_path/'current.json').write_text(json.dumps({'file':'../outside.npz','sha256':'x'}))
    with pytest.raises(ValueError,match='escapes'):current_map(tmp_path)
