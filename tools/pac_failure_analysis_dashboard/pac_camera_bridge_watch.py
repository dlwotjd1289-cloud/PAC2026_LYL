#!/usr/bin/env python3
"""Read-only, late-attaching optional Gazebo RGB camera bridge supervisor.

Launches ONLY own ros_gz_bridge process; no Gazebo/MoveIt process management.
"""
from __future__ import annotations
import argparse
import os
import re
import signal
import subprocess
import time
from pathlib import Path

from pac_integration import load_binding,BindingError

DEFAULT_CAMERAS={'scale_camera':'/pac/scale_camera/image','cctv_camera':'/pac/top_camera/image'}


def desired_cameras(binding):
    topics=(binding or {}).get('topics',{})
    return {k:topics.get(k,default) for k,default in DEFAULT_CAMERAS.items()}


def topic_lines(command,timeout=5):
    try:
        p=subprocess.run(command,capture_output=True,text=True,timeout=timeout,check=False)
    except (OSError,subprocess.TimeoutExpired):return []
    return p.stdout.splitlines() if p.returncode==0 else []


def has_publisher(topic):
    lines=topic_lines(['ros2','topic','info',topic],4)
    for line in lines:
        m=re.search(r'Publisher count:\s*(\d+)',line)
        if m:return int(m.group(1))>0
    return False


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repo',type=Path,required=True)
    p.add_argument('--binding-file',type=Path,default=None)
    p.add_argument('--interval',type=float,default=8.0)
    p.add_argument('--log-file',type=Path,default=None)
    args=p.parse_args()
    if args.interval<3:raise SystemExit('interval >= 3s required')
    subprocess.run(['true'],check=False)
    owned=None; selected=()
    stop=False
    def halt(_sig,_frame):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGTERM,halt);signal.signal(signal.SIGINT,halt)
    def close_owned():
        nonlocal owned,selected
        if owned is not None:
            if owned.poll() is None:
                owned.terminate()
                try:owned.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    owned.kill();owned.wait(timeout=2)
            owned=None; selected=()
    try:
        while not stop:
            try:binding=load_binding(args.repo,args.binding_file)
            except (BindingError,OSError):binding=None
            gazebo=set(topic_lines(['ign','topic','-l']))
            cameras=desired_cameras(binding)
            available=[t for t in cameras.values() if t in gazebo]
            wanted=tuple(sorted(available))
            if owned and (owned.poll() is not None or wanted!=selected):close_owned()
            if owned is None and wanted:
                ros_topics=set(x.split(' [')[0] for x in topic_lines(['ros2','topic','list','-t'],5))
                # Avoid spawning any bridge for topics with an existing publisher.
                needed=[t for t in wanted if t not in ros_topics or not has_publisher(t)]
                if needed:
                    args_bridge=[f'{t}@sensor_msgs/msg/Image[ignition.msgs.Image' for t in needed]
                    destination=args.log_file or (args.repo/'logs/pac_dashboard_integration/camera_bridge.log')
                    destination=Path(destination);destination.parent.mkdir(parents=True,exist_ok=True)
                    with destination.open('ab') as log:
                        owned=subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge',*args_bridge],
                            stdin=subprocess.DEVNULL,stdout=log,stderr=log)
                    selected=wanted
                    print('READONLY camera bridge attached:',', '.join(needed),flush=True)
            # No simulation? Stay alive, await the integrated launch.
            for _ in range(int(args.interval*10)):
                if stop:break
                time.sleep(.1)
    finally:close_owned()
    return 0


if __name__=='__main__':raise SystemExit(main())
