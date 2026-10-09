#!/usr/bin/env python3
"""Bounded, read-only SDF <visual> primitive extraction for the browser orbit viewer.

Background shapes come from the locally installed V4.6 workcell world, NEVER from
synthetic runtime poses. Dynamic entities arrive separately via ROS/Gazebo.
"""
from __future__ import annotations
from functools import lru_cache
import math
from pathlib import Path
import xml.etree.ElementTree as ET

SDF_PATH = Path('ros2_ws/src/pac_simulation/worlds/ahead_workcell_v4_6_two_cam.sdf')
MAX_SDF_BYTES = 12 * 1024 * 1024
MAX_PRIMITIVES = 1600


def quaternion_from_rpy(r, p, y):
    cr, sr = math.cos(r/2), math.sin(r/2)
    cp, sp = math.cos(p/2), math.sin(p/2)
    cy, sy = math.cos(y/2), math.sin(y/2)
    return (sr*cp*cy-cr*sp*sy,
            cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy,
            cr*cp*cy+sr*sp*sy)


def qmul(a, b):
    ax,ay,az,aw=a; bx,by,bz,bw=b
    return (aw*bx+ax*bw+ay*bz-az*by,
            aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw,
            aw*bw-ax*bx-ay*by-az*bz)


def qrotate(q, v):
    x,y,z,w=q; vx,vy,vz=v
    ux,uy,uz=(y*vz-z*vy, z*vx-x*vz, x*vy-y*vx)
    tx,ty,tz=(2*ux,2*uy,2*uz)
    return (vx + w*tx + y*tz-z*ty,
            vy + w*ty + z*tx-x*tz,
            vz + w*tz + x*ty-y*tx)


def pose(node):
    raw=((node.findtext('pose') if node is not None else '') or '').split()
    if not raw:return (0.,0.,0.),(0.,0.,0.,1.)
    if len(raw)!=6:raise ValueError('SDF pose must contain six values')
    values=[float(v) for v in raw]
    if not all(math.isfinite(v) and abs(v)<100000 for v in values):
        raise ValueError('invalid SDF pose')
    return tuple(values[:3]), quaternion_from_rpy(*values[3:])


def compose(parent, child):
    pp,pq=parent; cp,cq=child
    offset=qrotate(pq,cp)
    return tuple(a+b for a,b in zip(pp,offset)),qmul(pq,cq)


def rgb(material, model):
    value=material.findtext('diffuse') if material is not None else None
    if not value:value=material.findtext('ambient') if material is not None else None
    if value:
        try:
            parts=[float(n) for n in value.split()]
            if len(parts)>=3 and all(math.isfinite(n) for n in parts):
                return '#'+''.join(f'{round(255*max(0,min(1,n))):02x}' for n in parts[:3])
        except ValueError:pass
    name=model.lower()
    if 'floor' in name:return '#343e49'
    if 'conveyor' in name or 'scale' in name:return '#66798a'
    if 'pallet' in name:return '#aa8762'
    if 'buffer' in name:return '#567a85'
    if 'camera' in name:return '#8796a6'
    return '#8391a0'


def geometry_of(visual):
    geometry=visual.find('geometry')
    if geometry is None:return None
    for label in ('box','cylinder','sphere'):
        node=geometry.find(label)
        if node is None:continue
        if label=='box':v=node.findtext('size');num=3
        elif label=='cylinder':v=' '.join((node.findtext('radius') or '',node.findtext('length') or ''));num=2
        else:v=node.findtext('radius');num=1
        try:
            numbers=[float(n) for n in v.split()] if v else []
        except ValueError:return None
        if len(numbers)!=num or not all(math.isfinite(x) and 0<x<100 for x in numbers):return None
        return label,numbers
    return None


def extract_sdf_primitives(data):
    """Data is bytes from a local, preexisting world SDF. Unsupported meshes are counted."""
    if len(data)>MAX_SDF_BYTES:raise ValueError('world SDF exceeds safe size')
    root=ET.fromstring(data)
    world=root.find('world')
    if world is None:raise ValueError('world element missing')
    objects=[]; skipped=0; relative_skipped=0
    for model in world.findall('model'):
        model_name=model.get('name','unknown')[:100]
        model_pose=pose(model)
        if (model.find('pose') is not None and model.find('pose').get('relative_to')):
            relative_skipped+=1;continue
        for link in model.findall('link'):
            if (link.find('pose') is not None and link.find('pose').get('relative_to')):
                relative_skipped+=1;continue
            link_pose=compose(model_pose,pose(link))
            for visual in link.findall('visual'):
                if (visual.find('pose') is not None and visual.find('pose').get('relative_to')):
                    relative_skipped+=1;continue
                geom=geometry_of(visual)
                if geom is None:skipped+=1;continue
                if len(objects)>=MAX_PRIMITIVES:
                    raise ValueError('world SDF exceeds primitive budget')
                centre, q=compose(link_pose,pose(visual))
                shape,dims=geom
                objects.append({'name':(model_name+'/'+(visual.get('name') or 'visual'))[:150],
                                'model':model_name,'shape':shape,'size':dims,
                                'position':[round(x,6) for x in centre],
                                'quaternion':[round(x,8) for x in q],
                                'color':rgb(visual.find('material'),model_name)})
    return {'world':world.get('name',''), 'primitives':objects,
            'skipped_mesh_or_unsupported':skipped,
            'skipped_relative_frame':relative_skipped,'static_source':'local_v46_sdf'}


@lru_cache(maxsize=4)
def _read_scene(filename, mtime_ns, size):
    del mtime_ns
    if size>MAX_SDF_BYTES:raise ValueError('world SDF too large')
    with open(filename,'rb') as f:data=f.read(MAX_SDF_BYTES+1)
    return extract_sdf_primitives(data)


def scene_for_repo(repo, world_file=None, expected_world=''):
    """Bound world geometry; never show old V4.6 shapes for a different live world."""
    repo=Path(repo).expanduser().resolve()
    path=Path(world_file).resolve() if world_file else repo/SDF_PATH
    if world_file is None and expected_world and expected_world!='ahead_workcell_v4_2_physical_scale':
        return {'world':expected_world,'primitives':[],'static_source':'unmatched_world',
                'skipped_mesh_or_unsupported':0,'skipped_relative_frame':0,
                'error':'실행 월드와 기본 V4.6 SDF가 다릅니다. 통합 manifest에 world_sdf를 등록하세요.'}
    if not path.is_file():
        return {'world':'','primitives':[],'static_source':'not_found',
                'skipped_mesh_or_unsupported':0,'skipped_relative_frame':0,
                'error':'선택 월드 SDF 파일이 없어 배경 형상을 표시할 수 없습니다.'}
    try:
        stat=path.stat()
        result=_read_scene(str(path),stat.st_mtime_ns,stat.st_size).copy()
        if expected_world and result['world']!=expected_world:
            return {'world':expected_world,'primitives':[],'static_source':'world_mismatch',
                    'skipped_mesh_or_unsupported':0,'skipped_relative_frame':0,
                    'error':'SDF world 이름이 관측 월드와 달라 배경 표시를 차단했습니다.'}
        result['static_source']='bound_world_sdf' if world_file else 'local_v46_sdf'
        result['error']=''
        return result
    except (ValueError,OSError,ET.ParseError):
        return {'world':'','primitives':[],'static_source':'invalid',
                'skipped_mesh_or_unsupported':0,'skipped_relative_frame':0,
                'error':'작업셀 SDF를 안전하게 읽을 수 없습니다.'}
