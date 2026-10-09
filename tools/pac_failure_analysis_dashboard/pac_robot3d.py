#!/usr/bin/env python3
"""Read-only HDR50-22 kinematic tree and downsampled local STL visual geometry.

Uses the team's existing urdf.xacro and hdr_description files. NEVER invents
joint feedback or fetches mesh resources across the network. STL meshes are
transmitted only over the local dashboard HTTP listener, not packed in the ZIP.
"""
from __future__ import annotations
from functools import lru_cache
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
from pac_scene3d import quaternion_from_rpy

URDF=Path('ros2_ws/src/pac_bringup/urdf/hdr50_22_pedestal.urdf.xacro')
MAX_STL_BYTES=40*1024*1024
MAX_FACES_PER_STL=1250
MAX_MESHES=32


def floats(value, count, default):
    if not value:return list(default)
    try:nums=[float(x) for x in value.split()]
    except (ValueError,TypeError):return list(default)
    if len(nums)!=count or not all(math.isfinite(x) and abs(x)<1e5 for x in nums):
        return list(default)
    return nums


def origin(node):
    origin=node.find('origin') if node is not None else None
    if origin is None:return [0.,0.,0.],[0.,0.,0.,1.]
    return floats(origin.get('xyz'),3,[0,0,0]),list(quaternion_from_rpy(
        *floats(origin.get('rpy'),3,[0,0,0])))


def mesh_path(filename):
    prefix='file://$(find hdr_description)/meshes/robots/hdr50_22/visual/'
    if not filename or not filename.startswith(prefix):return None
    name=filename[len(prefix):]
    if '/' in name or '\\' in name or name in ('.','..') or not name.endswith('.stl'):
        return None
    return name


def stl_vertices(file, max_faces=MAX_FACES_PER_STL):
    """Deterministic even sampling of genuine binary STL triangles, no geometry fabrication."""
    if file.stat().st_size>MAX_STL_BYTES:return []
    raw=file.read_bytes()
    if len(raw)<84:return []
    count=struct.unpack_from('<I',raw,80)[0]
    if not count or 84+count*50>len(raw):return []
    stride=max(1, math.ceil(count/max_faces))
    result=[]
    for i in range(0,count,stride):
        off=84+i*50+12
        coords=struct.unpack_from('<9f',raw,off)
        if all(math.isfinite(v) and abs(v)<1e7 for v in coords):
            result.append([round(v,4) for v in coords])
    return result


def extract_robot_urdf(urdf,mesh_base=None):
    root=ET.fromstring(urdf)
    if root.tag!='robot':raise ValueError('URDF robot not found')
    mats={}
    for m in root.findall('material'):
        c=m.find('color');raw=c.get('rgba') if c is not None else None
        if raw:
            val=floats(raw,4,[.8,.8,.8,1])
            mats[m.get('name')]=''.join(f'{round(255*max(0,min(1,x))):02x}' for x in val[:3])
    joints=[]
    for j in root.findall('joint'):
        par=j.find('parent');child=j.find('child')
        if par is None or child is None:continue
        xyz,quat=origin(j);axis=j.find('axis')
        joints.append({'name':j.get('name',''), 'type':j.get('type','fixed'),
                       'parent':par.get('link'), 'child':child.get('link'),
                       'origin':xyz,'quaternion':quat,
                       'axis':floats(axis.get('xyz') if axis is not None else None,3,[1,0,0])})
    meshes=[];skipped=0;cache={}
    mesh_base=Path(mesh_base).resolve() if mesh_base else None
    for link in root.findall('link'):
        lname=link.get('name','')
        if len(meshes)>=MAX_MESHES:break
        for visual in link.findall('visual'):
            m=visual.find('geometry/mesh')
            if m is None:continue
            name=mesh_path(m.get('filename',''))
            if not name or not mesh_base:skipped+=1;continue
            file=(mesh_base/name).resolve()
            if not file.is_relative_to(mesh_base) or not file.is_file():
                skipped+=1;continue
            if name not in cache:
                try:cache[name]=stl_vertices(file)
                except (OSError,ValueError,struct.error):cache[name]=[]
            faces=cache[name]
            if not faces:skipped+=1;continue
            xyz,quat=origin(visual)
            scale=m.get('scale','0.001 0.001 0.001').replace('${mesh_scale}','0.001')
            nums=floats(scale,3,[0.001]*3)
            material=visual.find('material')
            color=mats.get(material.get('name') if material is not None else None,'e3dcc5')
            meshes.append({'link':lname, 'origin':xyz,'quaternion':quat,
                           'scale':nums,'color':'#'+color,
                           'source':name,'triangles':faces})
            if len(meshes)>=MAX_MESHES:break
    return {'source':'local_hdr50_22_urdf', 'joints':joints,
            'meshes':meshes,'skipped_meshes':skipped,'error':''}


HOME_SRDF=Path('ros2_ws/src/pac_bringup/urdf/hdr50_22_suction.srdf.xacro')


def reference_home(repo):
    """Actual team SRDF `home` posture, ONLY for a clearly labelled static preview."""
    path=Path(repo)/HOME_SRDF
    if not path.is_file() or path.stat().st_size>2*1024*1024:return []
    try:
        root=ET.fromstring(path.read_bytes())
        group=next((x for x in root.findall('group_state') if x.get('name')=='home'),None)
        if group is None:return []
        result=[]
        for j in group.findall('joint'):
            name=j.get('name','')
            v=float(j.get('value','nan'))
            if name in ('j1','j2','j3','j4','j5','j6') and math.isfinite(v):
                result.append({'name':name,'position':v,'velocity':None})
        return result if set(j['name'] for j in result)=={'j1','j2','j3','j4','j5','j6'} else []
    except (ValueError,OSError,ET.ParseError):
        return []


@lru_cache(maxsize=4)
def cached_robot(filename,mtime_ns,mesh_root):
    del mtime_ns
    return extract_robot_urdf(Path(filename).read_bytes(),mesh_root)


def robot_for_repo(repo):
    repo=Path(repo).expanduser().resolve()
    home=reference_home(repo)
    urdf=repo/URDF
    if not urdf.is_file():
        return {'source':'not_found','joints':[],'meshes':[],
                'skipped_meshes':0,'reference_joints':home,'error':'HDR50-22 URDF를 찾을 수 없습니다.'}
    choices=[repo/'ros2_ws/src/hdr_description/meshes/robots/hdr50_22/visual',
             repo/'ros2_ws/install/hdr_description/share/hdr_description/meshes/robots/hdr50_22/visual']
    mesh_root=next((p.resolve() for p in choices if p.is_dir()),None)
    try:
        if urdf.stat().st_size>2*1024*1024:raise ValueError('large URDF')
        result=cached_robot(str(urdf),urdf.stat().st_mtime_ns,str(mesh_root) if mesh_root else '').copy()
        result['reference_joints']=home
        result['mesh_path_ok']=bool(mesh_root)
        if not result['meshes']:
            result['error']='로봇 STL 파일을 찾지 못했습니다. URDF 링크 기준 간략형만 표시합니다.'
        return result
    except (ValueError,OSError,ET.ParseError):
        return {'source':'invalid','joints':[],'meshes':[],
                'skipped_meshes':0,'reference_joints':home,'error':'로봇 URDF를 읽을 수 없습니다.'}


def robot_mesh_file(repo, name):
    """Only existing visual STL filenames referenced in the team's local robot URDF."""
    import re
    if not isinstance(name,str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,100}\.stl',name):
        return None
    repo=Path(repo).expanduser().resolve()
    source=repo/URDF
    if not source.is_file() or source.stat().st_size>2*1024*1024:return None
    try:
        robot=ET.parse(source).getroot()
        allowed={mesh_path(e.get('filename')) for e in robot.findall('.//visual/geometry/mesh')}
        if name not in allowed:return None
        roots=[repo/'ros2_ws/src/hdr_description/meshes/robots/hdr50_22/visual',
               repo/'ros2_ws/install/hdr_description/share/hdr_description/meshes/robots/hdr50_22/visual']
        for root in roots:
            if not root.is_dir():continue
            resolved=root.resolve(); path=(resolved/name).resolve()
            if (path.is_relative_to(resolved) and path.is_file()
                    and not path.is_symlink() and 84<=path.stat().st_size<=MAX_STL_BYTES):
                return path
    except (ET.ParseError,OSError,ValueError):pass
    return None
