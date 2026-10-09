import json
from pathlib import Path
from types import SimpleNamespace
import time

import pytest

from pac_scene3d import (extract_sdf_primitives, scene_for_repo, quaternion_from_rpy,
                         qrotate, SDF_PATH)
from pac_live_state import (empty_sample, validate_live, parse_ros_pose_tf,
                            parse_ign_pose_text, TelemetryStore)
from pac_diagnostic_console import DiagnosticService, make_app, no_context

EXAMPLE=b'''<sdf version="1.8"><world name="ahead_workcell_v4_2_physical_scale">
<model name="pallet_base"><pose>2 1 0 0 0 1.5707963267948966</pose><link name="pallet_link">
<visual name="deck"><pose>1 0 0.5 0 0 0</pose><geometry><box><size>1.2 1.0 0.2</size></box></geometry>
<material><diffuse>0.5 0.4 0.3 1</diffuse></material></visual>
<visual name="mesh"><geometry><mesh><uri>file://abc.stl</uri></mesh></geometry></visual>
</link></model><model name="scale_camera"><link name="camera">
<visual><geometry><cylinder><radius>0.1</radius><length>0.6</length></cylinder></geometry></visual>
</link></model></world></sdf>'''


def test_sdf_primitives_from_local_real_visual_geometry():
    parsed=extract_sdf_primitives(EXAMPLE)
    assert parsed['world']=='ahead_workcell_v4_2_physical_scale'
    assert parsed['skipped_mesh_or_unsupported']==1
    assert len(parsed['primitives'])==2
    pallet=parsed['primitives'][0]
    assert pallet['size']==[1.2,1.0,0.2]
    assert pallet['color']=='#80664c'
    assert pallet['position']==[2.,2.,.5]
    assert abs(qrotate(quaternion_from_rpy(0,0,1.5707963267948966),(1,0,0))[1]-1)<1e-8


def test_missing_sdf_background_never_fabricated(tmp_path):
    r=scene_for_repo(tmp_path)
    assert not r['primitives'] and r['static_source']=='not_found'
    path=tmp_path/SDF_PATH;path.parent.mkdir(parents=True)
    path.write_bytes(EXAMPLE)
    result=scene_for_repo(tmp_path)
    assert len(result['primitives'])==2 and not result['error']


def test_gazebo_ros_tf_only_observed_poses():
    def transform(name,x):
        return SimpleNamespace(child_frame_id=name, transform=SimpleNamespace(
            translation=SimpleNamespace(x=x,y=.2,z=.9),
            rotation=SimpleNamespace(x=0,y=0,z=0,w=1)))
    msg=SimpleNamespace(transforms=[transform('box_01',-1.4),transform('non-interest',5),
                                    transform('hdr50_22::flange_link',.3)])
    poses=parse_ros_pose_tf(msg)
    assert len(poses)==2 and poses[0]['name']=='box_01'
    assert poses[0]['quaternion']==[0.,0.,0.,1.]
    assert poses[0]['x']==-1.4
    bad=SimpleNamespace(transforms=[transform('bad',5)])
    assert parse_ros_pose_tf(bad)==[]


def test_optional_quaternion_and_verified_box_size_stay_valid(tmp_path):
    sample=empty_sample();sample['collected_epoch_s']=time.time()
    sample['gazebo'].update(topic='/world/test/pose/info',observed_epoch_s=time.time(),entities=[
        {'name':'box_01','x':0.0,'y':1.0,'z':0.2,'yaw':0.0,
         'quaternion':[0,0,0,1],'size':[.5,.3,.4]}])
    validate_live(sample)
    path=tmp_path/'telemetry.json';path.write_text(json.dumps(sample))
    assert TelemetryStore(path).snapshot()['fresh']
    sample['gazebo']['entities'][0]['size']=[.5,0,.4]
    with pytest.raises(ValueError):validate_live(sample)


def test_pose_text_retains_full_3d_rotation():
    entries=parse_ign_pose_text('''pose {\n name: "box_01"\n position { x: 1 y: 2 z: 3 }\n orientation { x: 0.7071068 y: 0 z: 0 w: 0.7071068 }\n}\n''')
    assert abs(entries[0]['quaternion'][0]-.7071068)<1e-4


def test_scene_and_plotly_routes(tmp_path):
    import asyncio
    from aiohttp.test_utils import TestServer, TestClient
    class Logs:
        def __init__(self):self.repo=tmp_path
        def snapshot(self):return no_context()
    async def runner():
        path=tmp_path/SDF_PATH;path.parent.mkdir(parents=True)
        path.write_bytes(EXAMPLE)
        service=DiagnosticService(Logs(),api_key='')
        async with TestServer(make_app(service)) as server:
            async with TestClient(server) as c:
                res=await c.get('/api/scene3d');assert res.status==200
                value=await res.json();assert len(value['primitives'])==2
                s=await c.get('/assets/viewer3d.js');assert s.status==200
                code=await s.text();assert 'Plotly.react' in code
                s=await c.get('/assets/plotly.min.js',allow_redirects=False)
                if s.status == 302:  # source checkout: third-party bundle intentionally excluded
                    assert s.headers['Location'] == 'https://cdn.plot.ly/plotly-3.3.1.min.js'
                else:
                    assert s.status == 200
                    assert int(s.headers['Content-Length'])>1000000
                h=await (await c.get('/')).text()
                assert 'viewer3d-panel' in h and '/assets/plotly.min.js' in h
    asyncio.run(runner())
