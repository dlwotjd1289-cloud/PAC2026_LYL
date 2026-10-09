#!/usr/bin/env python3
"""Read-only, fail-closed contract between the integrated palletizer and dashboard.

An *optional* active-run manifest binds together one Gazebo world, ROS topics,
log run, and box metadata. It never starts or controls a simulator. A missing
manifest is allowed (single-world autodiscovery), but is NOT a proof that data
sources correspond to one experiment.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import re
import time

SCHEMA = '1.0'
DEFAULT_MANIFEST = Path('logs/pac_dashboard_integration/active_run.json')
MAX_MANIFEST_BYTES = 65536
MAX_CATALOG_BYTES = 512 * 1024
MAX_EVENTS_BYTES = 4 * 1024 * 1024
MAX_BOXES = 500
MAX_EVENTS = 3000
WORLD_PATTERN = re.compile(r'^[A-Za-z0-9_.-]{1,100}$')
RUN_PATTERN = re.compile(r'^[\w.:-]{1,128}$', re.UNICODE)
TOPIC_PATTERN = re.compile(r'^/[a-zA-Z0-9_/]+$')
TOPIC_KEYS = frozenset({'joint_states', 'controller', 'suction', 'scale_camera', 'cctv_camera', 'gazebo_pose', 'gazebo_ros'})
ZONE_NAMES = ('pallet', 'buffer', 'ng', 'pick', 'scale', 'conveyor')
KNOWN_EVENTS = frozenset({'pallet_placed', 'buffer_stored', 'ng_diverted', 'on_conveyor', 'at_pick', 'pick_completed', 'failed', 'held'})
EVENT_ZONE = {'pallet_placed': 'pallet', 'buffer_stored':'buffer', 'ng_diverted':'ng',
              'on_conveyor':'conveyor', 'at_pick':'pick'}


class BindingError(ValueError):
    pass


def manifest_path(repo, override=None):
    repo = Path(repo).expanduser().resolve()
    path = Path(override).expanduser() if override else repo / DEFAULT_MANIFEST
    path = path if path.is_absolute() else repo / path
    # The manifest itself must remain in the project logs directory.
    return _within(repo, path, repo / 'logs')


def _within(repo, path, root):
    repo=Path(repo).resolve(); root=Path(root).resolve()
    candidate=Path(path).expanduser()
    if not candidate.is_absolute():candidate=repo/candidate
    if candidate.is_symlink(): raise BindingError('symbolic-link input is prohibited')
    p=candidate.resolve()
    if not p.is_relative_to(root):
        raise BindingError('input path is outside permitted project directory')
    return p


def _read_json(path, max_size):
    if path.is_symlink() or path.stat().st_size>max_size:
        raise BindingError('file is symlinked or oversized')
    data=path.read_bytes()
    if len(data)>max_size: raise BindingError('input too large')
    def no_duplicates(items):
        result={}
        for k,v in items:
            if k in result:raise BindingError('duplicate key')
            result[k]=v
        return result
    def bad_constant(_):raise BindingError('invalid numeric constant')
    return json.loads(data,object_pairs_hook=no_duplicates,parse_constant=bad_constant)


def resolve_path(repo, raw, *, kind):
    if not isinstance(raw,str) or not raw or len(raw)>500:raise BindingError('invalid relative path')
    repo=Path(repo).resolve()
    if kind in ('log_dir','catalog','events'):
        root=repo/'logs'
    elif kind=='world_sdf':
        # Runtime-generated SDF can be stored under logs; installed SDF under ros2_ws.
        candidate=Path(raw)
        resolved=_within(repo,candidate,repo)
        if (not resolved.is_relative_to(repo/'ros2_ws') and
                not resolved.is_relative_to(repo/'logs')):
            raise BindingError('world SDF must be in ros2_ws or logs')
        return resolved
    else:raise BindingError('unsupported file kind')
    return _within(repo,raw,root)


def _zones(raw):
    if raw is None:return {}
    if type(raw) is not dict or len(raw)>len(ZONE_NAMES):raise BindingError('invalid zones')
    out={}
    for name,zone in raw.items():
        if name not in ZONE_NAMES or type(zone) is not dict or not {'x','y'}<=set(zone) or set(zone)-{'x','y','z'}:
            raise BindingError('invalid zone fields')
        dims={}
        for axis,interval in zone.items():
            if type(interval) is not list or len(interval)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>1e4 for v in interval) or interval[0]>=interval[1]:
                raise BindingError('invalid zone coordinate bounds')
            dims[axis]=tuple(float(v) for v in interval)
        out[name]=dims
    return out


def load_binding(repo, manifest=None):
    """Return validated active run metadata, None if no manifest; never infer a run."""
    repo=Path(repo).expanduser().resolve()
    path=manifest_path(repo,manifest)
    if not path.exists():return None
    try:data=_read_json(path,MAX_MANIFEST_BYTES)
    except (OSError,ValueError,UnicodeError,TypeError) as exc:raise BindingError('active run manifest cannot be read') from exc
    if type(data) is not dict or data.get('schema_version')!=SCHEMA:raise BindingError('unsupported integration manifest')
    allowed={'schema_version','run_id','world','world_sdf','log_dir','box_catalog','events_file','topics','zones'}
    if set(data)-allowed:raise BindingError('unsupported integration fields')
    run_id=data.get('run_id','');world=data.get('world','')
    if not isinstance(run_id,str) or not RUN_PATTERN.fullmatch(run_id):raise BindingError('invalid run id')
    if not isinstance(world,str) or not WORLD_PATTERN.fullmatch(world):raise BindingError('invalid world id')
    topics=data.get('topics',{})
    if type(topics) is not dict or set(topics)-TOPIC_KEYS:raise BindingError('invalid ROS/Gazebo topic keys')
    for key,topic in topics.items():
        if not isinstance(topic,str) or not TOPIC_PATTERN.fullmatch(topic) or '//' in topic or len(topic)>180:
            raise BindingError('invalid topic name')
    if 'gazebo_pose' in topics and not re.fullmatch(r'/world/'+re.escape(world)+r'/(?:dynamic_)?pose/info',topics['gazebo_pose']):
        raise BindingError('Gazebo topic belongs to a different world')
    if 'gazebo_ros' in topics and not re.fullmatch(r'/world/'+re.escape(world)+r'/pose/info',topics['gazebo_ros']):
        raise BindingError('Gazebo ROS bridge belongs to a different world')
    files={}
    for src,key in [('world_sdf','world_sdf'),('log_dir','log_dir'),('box_catalog','catalog'),('events_file','events')]:
        if src in data:files[src]=resolve_path(repo,data[src],kind=key)
    return {'run_id':run_id,'world':world,'topics':dict(topics),'zones':_zones(data.get('zones')),
            'files':files,'source':str(path),'mtime_ns':path.stat().st_mtime_ns}


def run_identity(binding):
    if not binding:return None
    return (binding['run_id'],binding['world'],tuple(sorted(binding['topics'].items())),binding['source'])


def load_catalog(binding,repo):
    """Box dimensions from this bound run ONLY. Existing boxes may predate observer."""
    if not binding:return {}
    catalog={}
    file=binding['files'].get('box_catalog')
    if file and file.is_file():
        try:
            content=_read_json(file,MAX_CATALOG_BYTES)
            if type(content) is not dict or len(content)>MAX_BOXES:raise BindingError('bad catalog')
            for name,info in content.items():
                if not isinstance(name,str) or not 1<=len(name)<=120:continue
                size=info.get('size_m') if type(info) is dict else None
                if type(size) is list and len(size)==3 and all(type(v) in (int,float) and math.isfinite(v) and 0<v<4 for v in size):
                    catalog[name]=[float(v) for v in size]
        except (OSError,ValueError,TypeError):pass
    folder=binding['files'].get('log_dir')
    if folder is not None:
        folder=folder/'boxes'
        if folder.is_dir() and not folder.is_symlink():
            from pac_box_geometry import parse_spawned_box_sdf,MAX_SDF_BYTES
            for file in list(folder.iterdir())[:MAX_BOXES]:
                if not re.fullmatch(r'box_\d{2,3}\.sdf',file.name) or file.is_symlink() or not file.is_file() or file.stem in catalog:continue
                try:
                    if file.stat().st_size>MAX_SDF_BYTES:continue
                    size=parse_spawned_box_sdf(file.read_bytes())
                    catalog[file.stem]=size
                except (ValueError,OSError):pass
    return catalog


def load_events(binding):
    """Executor claims only; not proof of actual physical completion."""
    if not binding:return {}
    path=binding['files'].get('events_file')
    if path is None or not path.is_file() or path.is_symlink():return {}
    try:
        stat=path.stat()
        if stat.st_size>MAX_EVENTS_BYTES: return {}
        content=path.read_text(encoding='utf-8')
    except (OSError,UnicodeError):return {}
    items={}
    for line in content.splitlines()[-MAX_EVENTS:]:
        if not line or len(line)>4096:continue
        try:e=json.loads(line)
        except (ValueError,TypeError):continue
        if type(e) is not dict or e.get('run_id')!=binding['run_id'] or e.get('world')!=binding['world']:continue
        name=e.get('box_id','');status=e.get('event','')
        if not isinstance(name,str) or not 1<=len(name)<=120 or status not in KNOWN_EVENTS:continue
        if type(e.get('success',False)) is not bool:continue
        stamp=e.get('timestamp_epoch_s')
        if type(stamp) not in (int,float) or not math.isfinite(stamp) or stamp<0:continue
        if name not in items or stamp>=items[name]['timestamp_epoch_s']:
            items[name]={'event':status,'success':bool(e.get('success',False)),
                         'timestamp_epoch_s':float(stamp),'source':'executor_event',
                         'reason':str(e.get('reason',''))[:200] if type(e.get('reason','')) is str else ''}
    return items


def locate_box(entity,zones):
    """Geometry-only location estimate. Never return a confirmed process status."""
    if not isinstance(entity,dict):return {'zone':'unknown','basis':'no_pose','candidates':[]}
    coords={axis:entity.get(axis) for axis in ('x','y','z')}
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in coords.values()):
        return {'zone':'unknown','basis':'invalid_pose','candidates':[]}
    candidates=[key for key,b in zones.items() if all(b[axis][0]<=coords[axis]<=b[axis][1] for axis in b)]
    if len(candidates)!=1:return {'zone':'unknown' if not candidates else 'ambiguous','basis':'center_only','candidates':candidates}
    zone=candidates[0];size=entity.get('size');quat=entity.get('quaternion');yaw=entity.get('yaw')
    # When yaw/size is unknown, the *centre* alone is a rough position estimate.
    basis='center_only'
    if type(size) is list and len(size)==3 and type(yaw) in (int,float) and math.isfinite(yaw):
        half_x=(abs(math.cos(yaw))*size[0]+abs(math.sin(yaw))*size[1])/2
        half_y=(abs(math.sin(yaw))*size[0]+abs(math.cos(yaw))*size[1])/2
        half_z=size[2]/2
        bounds=zones[zone]
        if all((axis not in bounds or
                (bounds[axis][0]<=coords[axis]-radius and coords[axis]+radius<=bounds[axis][1]))
               for axis,radius in [('x',half_x),('y',half_y),('z',half_z)]):basis='bounded_footprint'
    return {'zone':zone,'basis':basis,'candidates':[zone]}


def summarize_run(binding,live,repo=None):
    """Join fresh simulator observations to optional executor *claims* by box ID.

    A position in a zone is not proof that the intended operation succeeded.
    """
    root=repo or Path(binding['source']).resolve().parents[2] if binding else repo
    now=time.time()
    output={'bound':bool(binding),'run_id':binding['run_id'] if binding else '',
            'world':binding['world'] if binding else '', 'status':'not_bound' if not binding else 'no_live_poses',
            'boxes':[],'zones':{z:{'observed':0,'executor_reported':0,'discrepancy':0} for z in ZONE_NAMES},
            'unobserved_catalog_boxes':[],'reported_not_observed':[],
            'configured_zones':sorted(binding['zones']) if binding else [],
            'observed_epoch_s':None,'warnings':[]}
    if not binding:
        output['warnings'].append('통합 실행 manifest가 없어 구역과 실행 로그를 검증할 수 없습니다.')
        return output
    events=load_events(binding)
    if not live or not live.get('available') or live['source_age_s'].get('gazebo') is None or live['source_age_s']['gazebo']>5 or not live['fresh']:
        output['reported_not_observed']=sorted(name for name,e in events.items() if e['success'])
        output['warnings'].append('Gazebo 위치 정보 미수신/오래됨: 실행기 기록만 존재하며 실제 박스 구역은 확인할 수 없습니다.')
        return output
    topic=live['data']['gazebo']['topic']
    if not re.fullmatch(r'/world/'+re.escape(binding['world'])+r'/(?:dynamic_)?pose/info',topic):
        output['status']='world_mismatch'
        output['warnings'].append('Gazebo 위치 토픽의 월드가 선택된 실행 월드와 다릅니다. 데이터를 합치지 않습니다.')
        return output
    output['observed_epoch_s']=live['data']['gazebo']['observed_epoch_s']
    catalog=load_catalog(binding,root)
    observed_names=set()
    for e in live['data']['gazebo']['entities']:
        name=e['name']
        if not (re.fullmatch(r'(?i)box[_-]\d+',name) or name in catalog or name in events):continue
        observed_names.add(name)
        loc=locate_box(e,binding['zones'])
        evt=events.get(name)
        event_status=evt['event'] if evt and evt['success'] else 'unconfirmed'
        event_zone=EVENT_ZONE.get(event_status)
        mismatch=bool(event_zone and loc['zone'] not in ('unknown','ambiguous',event_zone))
        if loc['zone'] in output['zones']:output['zones'][loc['zone']]['observed']+=1
        if event_zone in output['zones']:output['zones'][event_zone]['executor_reported']+=1
        if mismatch and loc['zone'] in output['zones']:output['zones'][loc['zone']]['discrepancy']+=1
        output['boxes'].append({'id':name,'pose':[e['x'],e['y'],e['z']],
                                'size_m':e.get('size',catalog.get(name)),
                                'size_source':e.get('size_source','run_catalog' if name in catalog else 'unknown'),
                                'location':loc, 'process_status':event_status,
                                'event_source':'executor_event' if evt else 'none',
                                'event_reason':evt['reason'] if evt else '',
                                'event_matches_location':None if not event_zone or loc['zone'] in ('unknown','ambiguous') else not mismatch,
                                'confidence':'observed_pose_and_executor_report' if event_zone and not mismatch and loc['zone']==event_zone else 'partial'})
    output['unobserved_catalog_boxes']=sorted(set(catalog)-observed_names)[:MAX_BOXES]
    output['reported_not_observed']=sorted(name for name,e in events.items() if e['success'] and name not in observed_names)[:MAX_BOXES]
    output['status']='live' if binding['zones'] else 'live_no_zones'
    if not binding['zones']:output['warnings'].append('구역 좌표가 등록되지 않아 위치별 박스 개수를 확정할 수 없습니다.')
    if output['unobserved_catalog_boxes']:output['warnings'].append('등록된 박스 중 Gazebo 현재 위치가 없는 항목이 있습니다 (누락·미투입·시뮬레이션 종료 여부 미확인).')
    if output['reported_not_observed']:output['warnings'].append('실행기 완료 기록이 있으나 Gazebo 현재 위치가 확인되지 않는 박스가 있습니다.')
    return output


def add_zone_evidence(context,summary,secret=''):
    """Add one attributed, bounded summary. No claim of physical task success."""
    if (not summary or summary.get('status') not in ('live','live_no_zones')
            or context.get('run_id')!=summary.get('run_id')):
        return context
    import hashlib
    from datetime import datetime,timezone
    from pac_diagnostic_console import CONTEXT_SCHEMA,MAX_EVIDENCE,validate,clean
    output=json.loads(json.dumps(context))
    zones=summary.get('zones',{})
    counts=', '.join(f'{name}: 위치 관측 {data["observed"]} / 실행기 성공기록 {data["executor_reported"]}'
                      for name,data in zones.items() if data['observed'] or data['executor_reported'])
    if not counts:counts='구역별 관측 박스 없음 또는 구역 미등록'
    ng=[b for b in summary.get('boxes',[]) if b['process_status']=='ng_diverted' and b.get('event_reason')]
    reasons=' · NG 실행기 보고사유: '+ '; '.join(f"{b['id']}={b['event_reason']}" for b in ng[:3]) if ng else ''
    message=clean('통합 실행 '+summary['run_id']+' · '+counts+reasons+
                  ' · 위치는 Gazebo 추정 구역이며 실행기 기록은 물리적 완료의 독립 확인이 아님.',secret)[:1000]
    source='gazebo:active_run/'+summary['world']
    id_=hashlib.sha256((source+'\0'+message).encode()).hexdigest()[:20]
    entry={'id':id_,'text':message,'source':source,
           'file_modified_at':datetime.fromtimestamp(summary.get('observed_epoch_s') or time.time(),timezone.utc).isoformat()}
    output['evidence']=(output['evidence'][:MAX_EVIDENCE-1]+[entry])[:MAX_EVIDENCE]
    validate(output,CONTEXT_SCHEMA)
    return output
