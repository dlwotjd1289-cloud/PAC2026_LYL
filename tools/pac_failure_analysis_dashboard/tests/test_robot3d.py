from pathlib import Path
import struct
from pac_robot3d import extract_robot_urdf, stl_vertices, robot_for_repo, URDF


MODEL=b'''<robot name="hdr50_22"><link name="world"/><link name="base_link">
<visual><geometry><mesh filename="file://$(find hdr_description)/meshes/robots/hdr50_22/visual/base_body.stl" scale="0.001 0.001 0.001"/></geometry><material name="ivory"/></visual></link>
<joint name="world_joint" type="fixed"><parent link="world"/><child link="base_link"/><origin xyz="0 0 .40" rpy="0 0 0"/></joint>
<joint name="j1" type="revolute"><parent link="base_link"/><child link="lower_frame_link"/><origin xyz="0 0 .12" rpy="0 0 0"/><axis xyz="0 0 1"/></joint>
<material name="ivory"><color rgba="1 0.9 0.7 1"/></material></robot>'''


def fake_binary_stl(path):
    triangle=struct.pack('<12fH',0,0,1, 0,0,0, 1000,0,0, 0,1000,0, 0)
    assert len(triangle)==50
    path.write_bytes(b'Test STL'+bytes(72)+struct.pack('<I',1)+triangle)


def test_robot_original_urdf_axes_and_mesh_from_local_stl(tmp_path):
    root=tmp_path/'meshes';root.mkdir()
    stl=root/'base_body.stl';fake_binary_stl(stl)
    model=extract_robot_urdf(MODEL,root)
    assert len(model['joints'])==2
    assert model['joints'][0]['origin']==[0,0,.4]
    assert model['joints'][1]['axis']==[0,0,1]
    assert len(model['meshes'])==1
    m=model['meshes'][0]
    assert m['link']=='base_link' and m['scale']==[.001,.001,.001]
    assert m['triangles'][0][3]==1000.0
    assert m['color']=='#ffe6b2'


def test_nonlocal_mesh_paths_rejected(tmp_path):
    assert not extract_robot_urdf(MODEL.replace(b'base_body.stl',b'../../bad.stl'),tmp_path)['meshes']


def test_no_robot_file_no_geometry(tmp_path):
    result=robot_for_repo(tmp_path)
    assert result['source']=='not_found' and not result['meshes']


def test_robot_from_local_file(tmp_path):
    p=tmp_path/URDF;p.parent.mkdir(parents=True);p.write_bytes(MODEL)
    result=robot_for_repo(tmp_path)
    assert len(result['joints'])==2 and not result['meshes']
