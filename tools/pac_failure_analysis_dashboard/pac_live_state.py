#!/usr/bin/env python3
"""Read-only filesystem handoff for Gazebo/ROS2 observations.

Data is created by pac_ros_observer.py from live ROS2/Gazebo subscriptions.
No robot commands, PyBullet state, synthetic poses, or implied physical safety.
"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import time


SCHEMA_VERSION = '1.0'
MAX_BYTES = 192 * 1024
MAX_JOINTS = 64
MAX_ENTITIES = 80
MAX_AGE_S = 5.0


class BadTelemetry(ValueError):
    pass


def finite(v, bound=1e8):
    return type(v) in (float, int) and math.isfinite(v) and abs(v) <= bound


def text(v, length=256):
    return type(v) is str and len(v) <= length


def stamp(v):
    return v is None or finite(v, 1e11) and v >= 0


def require(condition, message):
    if not condition:
        raise BadTelemetry(message)


def _source(data, required, extras=()):
    require(type(data) is dict and set(data)==set(required)|set(extras), 'source keys')
    require(text(data['topic'],256), 'invalid topic')
    require(stamp(data['observed_epoch_s']), 'invalid timestamp')


def validate_live(data):
    require(type(data) is dict and set(data)=={
        'schema_version','collected_epoch_s','joint_states','gazebo','cameras','suction','controller'
    }, 'top-level keys')
    require(data['schema_version']==SCHEMA_VERSION, 'unknown schema')
    require(finite(data['collected_epoch_s'],1e11), 'collected time')
    js=data['joint_states']
    _source(js, ('topic','observed_epoch_s','joints'))
    joints=js['joints']
    require(type(joints) is list and len(joints)<=MAX_JOINTS, 'joints count')
    for j in joints:
        require(type(j) is dict and set(j)=={'name','position','velocity'}, 'joint keys')
        require(text(j['name'],120) and finite(j['position'],1e6)
                and (j['velocity'] is None or finite(j['velocity'],1e6)), 'joint value')
    gz=data['gazebo']
    _source(gz, ('topic','observed_epoch_s','entities','error'))
    require(text(gz['error'],200), 'gazebo error')
    entities=gz['entities']
    require(type(entities) is list and len(entities)<=MAX_ENTITIES, 'entities count')
    for e in entities:
        require(type(e) is dict and {'name','x','y','z','yaw'} <= set(e)
                and set(e) <= {'name','x','y','z','yaw','quaternion','size','size_source'}, 'entity keys')
        require(text(e['name'],120), 'entity name')
        require(all(finite(e[k],1e6) for k in ('x','y','z')), 'entity coordinates')
        require(e['yaw'] is None or finite(e['yaw'],1e6), 'entity orientation')
        if 'quaternion' in e:
            q=e['quaternion']
            require(type(q) is list and len(q)==4 and all(finite(x,2) for x in q)
                    and .3<sum(x*x for x in q)<1.7, 'entity quaternion')
        if 'size_source' in e:
            require(e['size_source'] in ('spawn_sdf','run_catalog') and 'size' in e, 'box size source')
        if 'size' in e:
            s=e['size']
            require(type(s) is list and len(s)==3 and all(finite(x,100) and x>0 for x in s),
                    'entity verified size')
    cameras=data['cameras']
    require(type(cameras) is dict and set(cameras)=={'scale','cctv'}, 'camera keys')
    for camera in cameras.values():
        _source(camera,('topic','observed_epoch_s'))
    suction=data['suction']
    _source(suction,('topic','observed_epoch_s','value'))
    require(text(suction['value'],200), 'suction value')
    controller=data['controller']
    _source(controller,('topic','observed_epoch_s','max_abs_position_error'))
    err=controller['max_abs_position_error']
    require(err is None or finite(err,1e5) and err>=0, 'controller error')
    return data


def empty_sample():
    return {
        'schema_version':SCHEMA_VERSION,'collected_epoch_s':0.0,
        'joint_states':{'topic':'','observed_epoch_s':None,'joints':[]},
        'gazebo':{'topic':'','observed_epoch_s':None,'entities':[],'error':''},
        'cameras':{'scale':{'topic':'','observed_epoch_s':None},
                   'cctv':{'topic':'','observed_epoch_s':None}},
        'suction':{'topic':'','observed_epoch_s':None,'value':''},
        'controller':{'topic':'','observed_epoch_s':None,'max_abs_position_error':None},
    }


def age_of(timestamp, now=None):
    if timestamp is None:
        return None
    now = time.time() if now is None else now
    return round(max(0.0, now-float(timestamp)),1)


class TelemetryStore:
    """Read only an explicit bounded local state file; no external fetches."""
    def __init__(self, path, max_age_s=MAX_AGE_S, secret=''):
        self.path=Path(path).expanduser().absolute()
        self.max_age_s=float(max_age_s)
        self.secret=secret

    def snapshot(self):
        blank={'available':False,'fresh':False,'age_s':None,
               'source_age_s':{},'data':empty_sample(), 'error':'',
               'camera_jpeg_ok':{'scale':False,'cctv':False},
               'camera_jpeg_age_s':{'scale':None,'cctv':None}}
        try:
            if self.path.is_symlink() or not self.path.is_file():
                return blank
            stat=self.path.stat()
            if stat.st_size>MAX_BYTES:
                raise BadTelemetry('telemetry JSON too large')
            with self.path.open('rb') as fp:
                raw=fp.read(MAX_BYTES+1)
            if len(raw)>MAX_BYTES:
                raise BadTelemetry('telemetry JSON too large')
            def pairs(values):
                out={}
                for key,value in values:
                    if key in out:raise BadTelemetry('duplicate key')
                    out[key]=value
                return out
            def invalid(_):raise BadTelemetry('nonfinite numeric value')
            data=json.loads(raw,object_pairs_hook=pairs,parse_constant=invalid)
            validate_live(data)
            # Keep output safe even when an external source publishes secret-like text.
            from pac_diagnostic_console import sanitize_strings
            data=sanitize_strings(data,self.secret)
            validate_live(data)
            now=time.time()
            age=max(age_of(stat.st_mtime,now),age_of(data['collected_epoch_s'],now))
            source_age={'joint_states':age_of(data['joint_states']['observed_epoch_s'],now),
                        'gazebo':age_of(data['gazebo']['observed_epoch_s'],now),
                        'scale':age_of(data['cameras']['scale']['observed_epoch_s'],now),
                        'cctv':age_of(data['cameras']['cctv']['observed_epoch_s'],now),
                        'controller':age_of(data['controller']['observed_epoch_s'],now),
                        'suction':age_of(data['suction']['observed_epoch_s'],now)}
            camera_jpeg_ok={}; camera_jpeg_age_s={}
            for name in ('scale','cctv'):
                image=self.path.with_name('camera_'+name+'.jpg')
                camera_jpeg_ok[name]=False
                camera_jpeg_age_s[name]=None
                try:
                    if image.is_symlink() or not image.is_file():continue
                    jpg_stat=image.stat()
                    if not 8<=jpg_stat.st_size<=1024*1024:continue
                    camera_jpeg_age_s[name]=age_of(jpg_stat.st_mtime,now)
                    if camera_jpeg_age_s[name]>self.max_age_s:continue
                    with image.open('rb') as f:
                        header=f.read(2);f.seek(-2,2);trailer=f.read(2)
                    camera_jpeg_ok[name]=(header==b'\xff\xd8' and trailer==b'\xff\xd9'
                        and source_age[name] is not None and source_age[name]<=self.max_age_s)
                except (OSError,ValueError):continue
            return {'available':True, 'fresh':age<=self.max_age_s and any(
                        source_age[k] is not None and source_age[k]<=self.max_age_s
                        for k in ('joint_states','gazebo')),
                    'age_s':age, 'source_age_s':source_age, 'data':data, 'error':'',
                    'camera_jpeg_ok':camera_jpeg_ok,'camera_jpeg_age_s':camera_jpeg_age_s}
        except (OSError,ValueError,TypeError,KeyError,UnicodeError,OverflowError,RecursionError) as exc:
            # No untrusted exception text (may contain filenames or external data).
            blank['error']='읽을 수 없거나 유효하지 않은 관측 파일'
            return blank


def add_telemetry_evidence(log_context, snapshot, secret=''):
    """Attach only fresh, attributed observations to the *immutable* API log snapshot."""
    context=json.loads(json.dumps(log_context))
    if not snapshot['available'] or not snapshot['fresh']:
        return context
    from pac_diagnostic_console import MAX_EVIDENCE, CONTEXT_SCHEMA, validate, clean
    data=snapshot['data']; ages=snapshot['source_age_s']
    entries=[]
    def push(key, message, source, observed):
        if observed is None or ages[key] is None or ages[key]>MAX_AGE_S:
            return
        message=clean(message,secret)[:1000]
        source=clean(source,secret)[:512]
        id_=hashlib.sha256((source+'\0'+message+'\0'+str(observed)).encode()).hexdigest()[:20]
        entries.append({'id':id_,'source':source,'text':message,
                        'file_modified_at':datetime.fromtimestamp(observed,timezone.utc).isoformat()})
    if data['joint_states']['joints']:
        push('joint_states',f"ROS2 /joint_states 수신: {len(data['joint_states']['joints'])}개 관절, "
             f"관측 후 {ages['joint_states']}초 (관절 값은 UI에서 확인).",
             'ros2:'+data['joint_states']['topic'],data['joint_states']['observed_epoch_s'])
    if data['gazebo']['entities']:
        first=data['gazebo']['entities'][:6]
        sampled='; '.join(f"{x['name']}=({x['x']:.3f},{x['y']:.3f},{x['z']:.3f})m" for x in first)
        push('gazebo',f"Gazebo pose/info 수신: {len(data['gazebo']['entities'])}개 관심 엔티티, "
             f"관측 후 {ages['gazebo']}초. 일부 좌표: {sampled}",
             'gazebo:'+data['gazebo']['topic'],data['gazebo']['observed_epoch_s'])
    if data['controller']['max_abs_position_error'] is not None:
        push('controller',f"ROS2 컨트롤러가 보고한 최대 절대 관절 위치 오차: "
             f"{data['controller']['max_abs_position_error']:.5f} rad. 실제 물리 파손 증거 아님.",
             'ros2:'+data['controller']['topic'],data['controller']['observed_epoch_s'])
    context['evidence']=context['evidence'][:MAX_EVIDENCE-len(entries)]+entries
    validate(context,CONTEXT_SCHEMA)
    return context


def parse_ign_pose_text(message, pattern=r'(?i)box|pallet|buffer|hdr50|robot|suction', limit=MAX_ENTITIES):
    """Parse only top-level pose { ... } blocks of `ign topic -e -n 1` text.

    Return coordinates as Gazebo reports them. Never invent model dimensions or poses.
    """
    matcher=re.compile(pattern)
    float_re=r'[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?'
    def field(s,name,default=None):
        m=re.search(r'\b'+re.escape(name)+r'\s*:\s*('+float_re+r')',s)
        return float(m.group(1)) if m else default
    def nested(s,name):
        m=re.search(r'\b'+re.escape(name)+r'\s*\{',s)
        if not m:return None
        begin=m.end();i=begin;depth=1
        while i<len(s) and depth:
            if s[i]=='{':depth+=1
            if s[i]=='}':depth-=1
            i+=1
        return s[begin:i-1] if depth==0 else None
    entities=[]
    # The protobuf text includes repeated pose { } blocks, potentially nested
    # substructures. Parse balanced blocks instead of splitting on a token.
    for match in re.finditer(r'(?m)^pose\s*\{', message):
        begin=match.end();i=begin;depth=1
        while i<len(message) and depth:
            if message[i]=='{':depth+=1
            elif message[i]=='}':depth-=1
            i+=1
        if depth:continue
        block=message[begin:i-1]
        m=re.search(r'(?m)^\s*name\s*:\s*"([^"\n]{1,120})"',block)
        if not m or not matcher.search(m.group(1)):
            continue
        coords=nested(block,'position')
        if coords is None:continue
        xyz=[field(coords,k,0.0) for k in ('x','y','z')]
        if not all(finite(x,1e6) for x in xyz):continue
        orientation=nested(block,'orientation')
        yaw=None
        if orientation is not None:
            x,y,z,w=[field(orientation,k,(1.0 if k=='w' else 0.0))
                      for k in ('x','y','z','w')]
            if all(finite(v,1e6) for v in (x,y,z,w)):
                yaw=2*math.atan2(z,w) if abs(x)+abs(y)<1e-8 else math.atan2(
                    2*(w*z+x*y),1-2*(y*y+z*z))
        entity={'name':m.group(1),'x':xyz[0],'y':xyz[1],'z':xyz[2],'yaw':yaw}
        if orientation is not None and all(finite(v,2) for v in (x,y,z,w)) and .3<x*x+y*y+z*z+w*w<1.7:
            entity['quaternion']=[x,y,z,w]
        entities.append(entity)
        if len(entities)>=limit:break
    return entities


def parse_ros_pose_tf(msg, pattern=r'(?i)box|pallet|buffer|hdr50|robot|suction', limit=MAX_ENTITIES):
    """Gazebo Pose_V -> tf2_msgs/TFMessage bridge; use only observed frame names and poses.

    This does not interpret ROS /tf as Gazebo truth; caller must provide the
    dedicated /world/.../pose/info bridge topic, not generic /tf.
    """
    matcher=re.compile(pattern)
    result=[]
    for t in msg.transforms:
        name=str(t.child_frame_id)[:120]
        if not matcher.search(name):continue
        v=t.transform.translation;q=t.transform.rotation
        xyz=[float(v.x),float(v.y),float(v.z)]
        rotation=[float(q.x),float(q.y),float(q.z),float(q.w)]
        if not all(finite(c,1e6) for c in xyz) or not all(finite(c,2) for c in rotation):continue
        if not .3<sum(c*c for c in rotation)<1.7:continue
        x,y,z,w=rotation
        yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        result.append({'name':name, 'x':xyz[0], 'y':xyz[1], 'z':xyz[2],
                       'yaw':yaw, 'quaternion':rotation})
        if len(result)>=limit:break
    return result
