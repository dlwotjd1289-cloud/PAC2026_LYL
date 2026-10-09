#!/usr/bin/env python3
"""Read-only integration preflight. Does not start, modify or terminate processes."""
from __future__ import annotations
import argparse
from pathlib import Path
import json
import os
import re
import subprocess
import urllib.request
from pac_integration import load_binding,BindingError,DEFAULT_MANIFEST
from pac_live_state import TelemetryStore
from pac_gazebo_stream import choose_runtime_pose_topic


def read_lines(command,seconds=6):
    try:
        result=subprocess.run(command,text=True,capture_output=True,timeout=seconds)
        return result.stdout.splitlines() if result.returncode==0 else []
    except (OSError,subprocess.TimeoutExpired):return []


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo',type=Path,default=Path.home()/'AHEAD/pac2026_integrated')
    parser.add_argument('--port',type=int,default=4183)
    parser.add_argument('--binding-file',type=Path,default=None)
    args=parser.parse_args()
    repo=args.repo.expanduser().resolve()
    print('PAC2026 integration V5.3 check · 읽기전용')
    print('project:',repo)
    for k in ('IGN_IP','GZ_PARTITION','IGN_PARTITION','ROS_LOCALHOST_ONLY','ROS_DOMAIN_ID','RMW_IMPLEMENTATION'):
        print(f'  {k}={os.getenv(k,"(unset)")}')
    try:bound=load_binding(repo,args.binding_file)
    except (BindingError,OSError) as ex:
        bound=None;print('manifest invalid:',str(ex))
    if bound:
        print('active run:',bound['run_id'],'world:',bound['world'])
        print('run logs:',bound['files'].get('log_dir','unregistered'))
        print('zone definitions:',', '.join(bound['zones']) or 'NONE (status will not be inferred)')
    else:print('active run manifest not registered:',args.binding_file or repo/DEFAULT_MANIFEST)
    gazebo=read_lines(['ign','topic','-l'])
    poses=[t for t in gazebo if re.fullmatch(r'/world/[^/]+/(?:dynamic_)?pose/info',t)]
    print('Gazebo pose topics:',poses or 'NONE')
    print('selected:',choose_runtime_pose_topic(poses,world=bound['world'] if bound else '') or
          'NONE/AMBIGUOUS')
    ros=read_lines(['ros2','topic','list','--no-daemon','-t'],12)
    print('ROS joint/camera topics:',[t for t in ros if any(s in t for s in ('joint_states','camera','image','controller'))] or 'NONE')
    store=TelemetryStore(repo/'logs/pac_dashboard_integrated_v53/telemetry.json')
    live=store.snapshot()
    print('telemetry available:',live['available'],'fresh:',live['fresh'],'file_age_s:',live['age_s'])
    for key,age in live.get('source_age_s',{}).items():print('  source:',key,'age_s:',age)
    print('  Gazebo error:',live['data']['gazebo']['error'])
    try:
        with urllib.request.urlopen(f'http://127.0.0.1:{args.port}/api/integration',timeout=3) as r:
            state=json.load(r)
        print('dashboard integration:',state.get('status'),'run:',state.get('run_id'),
              'boxes:',len(state.get('boxes',[])))
        for name,v in state.get('zones',{}).items():
            if v['observed'] or v['executor_reported'] or v['discrepancy']:
                print('  zone',name,v)
        for warning in state.get('warnings',[]):print('  WARNING:',warning)
    except (OSError,ValueError) as ex:
        print('dashboard API not reached:',type(ex).__name__)
    print('Note: no detected topics is not proof Gazebo is absent; check domain/partition of launch process.')
    return 0

if __name__=='__main__':raise SystemExit(main())
