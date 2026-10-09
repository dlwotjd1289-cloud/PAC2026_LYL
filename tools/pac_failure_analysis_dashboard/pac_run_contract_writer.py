#!/usr/bin/env python3
"""Optional integration hooks for the future AHEAD orchestration code.

The orchestrator calls register_active_run ONCE and report_event after it has
independently confirmed a step. Neither function commands ROS or Gazebo.
"""
from __future__ import annotations
import json
import os
from pathlib import Path
import time
from pac_integration import DEFAULT_MANIFEST, load_binding, resolve_path, manifest_path


def register_active_run(repo, *, run_id, world, world_sdf, log_dir,
                        zones=None, topics=None, box_catalog=None, events_file=None):
    repo=Path(repo).resolve()
    path=manifest_path(repo)
    data={'schema_version':'1.0','run_id':run_id,'world':world,
          'world_sdf':str(world_sdf),'log_dir':str(log_dir),
          'zones':zones or {},'topics':topics or {}}
    if box_catalog:data['box_catalog']=str(box_catalog)
    if events_file:data['events_file']=str(events_file)
    # Check all bounded paths BEFORE writing. This does not create files in
    # arbitrary places or silently accept out-of-tree logs.
    resolve_path(repo,str(world_sdf),kind='world_sdf')
    resolve_path(repo,str(log_dir),kind='log_dir')
    if box_catalog:resolve_path(repo,str(box_catalog),kind='catalog')
    if events_file:resolve_path(repo,str(events_file),kind='events')
    path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_name('.'+path.name+'.tmp-'+str(os.getpid()))
    with temp.open('x',encoding='utf-8') as file:
        json.dump(data,file,ensure_ascii=False,allow_nan=False)
        file.flush();os.fsync(file.fileno())
    try:
        # Validate BEFORE replacing an existing valid active-run file.
        load_binding(repo,temp)
        os.replace(temp,path)
        return load_binding(repo)
    finally:
        if temp.exists():temp.unlink()


def report_event(repo,box_id,event,*,success,reason='',stamp=None):
    binding=load_binding(repo)
    if not binding or 'events_file' not in binding['files']:
        raise ValueError('active run events_file not registered')
    if type(success) is not bool:
        raise ValueError('success must be a boolean')
    path=binding['files']['events_file']
    path.parent.mkdir(parents=True,exist_ok=True)
    if path.is_symlink():raise ValueError('events_file must not be a symlink')
    payload={'run_id':binding['run_id'],'world':binding['world'],
             'box_id':box_id,'event':event,'success':success,
             'timestamp_epoch_s':time.time() if stamp is None else stamp,'reason':reason}
    line=json.dumps(payload,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n'
    if len(line.encode())>4096:raise ValueError('event line too large')
    # Separate line-oriented event contract. For multi-process writers, use
    # external coordination or funnel reporting through one orchestrator.
    fd=os.open(path,os.O_CREAT|os.O_WRONLY|os.O_APPEND|os.O_NOFOLLOW,0o600)
    try:os.write(fd,line.encode());os.fsync(fd)
    finally:os.close(fd)
    return payload
