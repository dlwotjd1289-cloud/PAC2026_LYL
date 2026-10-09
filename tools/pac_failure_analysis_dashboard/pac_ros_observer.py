#!/usr/bin/env python3
"""READ-ONLY ROS2/Gazebo telemetry observer for the PAC2026 dashboard.

Run inside the user's ROS2 Humble environment. This sidecar NEVER publishes ROS
or Gazebo messages and NEVER commands the robot. It writes bounded snapshots
and reduced-resolution JPEGs locally; the HTTP dashboard reads the files.

`discover` prints real current topics. `run` subscribes ONLY to discovered
compatible types (or exact topics explicitly supplied by the operator).
"""
from __future__ import annotations

import argparse
import copy
import json
import select
import shutil
import os
from pathlib import Path
import re
import subprocess
import threading
import time

from pac_live_state import empty_sample, validate_live, parse_ign_pose_text, parse_ros_pose_tf
from pac_box_geometry import SpawnedBoxSizes
from pac_gazebo_stream import PoseTextDecoder, usable_pose_topic, choose_runtime_pose_topic
from pac_integration import load_binding, run_identity, BindingError


def choose_topic(graph, wanted, msgtype, choices):
    """Choose topic from current ROS graph, not a fabricated topic name."""
    eligible=sorted(t for t,types in graph.items() if msgtype in types)
    if wanted:
        if wanted not in eligible:
            raise ValueError(f'{wanted} 은(는) ROS 그래프에 {msgtype} 타입으로 존재하지 않습니다.')
        return wanted
    for name in choices:
        if name in eligible:return name
    return None


def discover_gazebo_topic(cli='ign', requested='', world=''):
    try:
        p=subprocess.run([cli,'topic','-l'],capture_output=True,text=True,
                         timeout=4,check=False)
    except (OSError,subprocess.TimeoutExpired):return ''
    if p.returncode:return ''
    return choose_runtime_pose_topic(p.stdout.splitlines(),world=world,requested=requested)


def prefer_top_level(entities, priority_names=()):
    """Model poses first; hundreds of Gazebo link/visual poses cannot crowd out boxes."""
    top=[e for e in entities if '::' not in e['name']]
    preferred=set(priority_names)
    ranked=sorted(top,key=lambda e:(not (e['name'] in preferred or bool(re.match(r'(?i)^box[_-]\d+$',e['name']))),e['name']))
    seen=set();result=[]
    for e in ranked:
        if e['name'] in seen:continue
        seen.add(e['name']);result.append(e)
        if len(result)>=80:break
    return result


def atomic_write(path, content):
    path=Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_name(path.name+'.tmp')
    with temp.open('wb') as f:
        f.write(content)
    try:os.chmod(temp,0o600)
    except OSError:pass
    os.replace(temp,path)


def image_to_jpeg(msg, max_width=720):
    """JPEG from a REAL ROS2 sensor_msgs/Image. Unknown encoding => no image."""
    import numpy as np
    import cv2
    channels={'rgb8':3,'bgr8':3,'rgba8':4,'bgra8':4,'mono8':1}
    encoding=msg.encoding.lower()
    if encoding not in channels:raise ValueError('unsupported camera encoding: '+encoding[:32])
    channels_n=channels[encoding]
    h,w,step=int(msg.height),int(msg.width),int(msg.step)
    if h<1 or w<1 or h>4500 or w>6000 or step<w*channels_n or step*h>48*1024*1024:
        raise ValueError('invalid camera image dimensions')
    buf=np.frombuffer(bytes(msg.data),dtype=np.uint8)
    if len(buf)<step*h:raise ValueError('incomplete camera image')
    rows=buf[:step*h].reshape(h,step)
    if channels_n==1:
        img=rows[:,:w].copy()
    else:
        img=rows[:,:w*channels_n].reshape(h,w,channels_n).copy()
        cvt={'rgb8':cv2.COLOR_RGB2BGR, 'rgba8':cv2.COLOR_RGBA2BGR,
             'bgra8':cv2.COLOR_BGRA2BGR}
        if encoding in cvt:img=cv2.cvtColor(img,cvt[encoding])
    if w>max_width:
        img=cv2.resize(img,(max_width,max(1,round(h*max_width/w))),
                       interpolation=cv2.INTER_AREA)
    ok,encoded=cv2.imencode('.jpg',img,[int(cv2.IMWRITE_JPEG_QUALITY),65])
    if not ok:raise ValueError('JPEG encoding failed')
    return encoded.tobytes()


class ReadOnlyObserver:
    def __init__(self, node, args):
        self.node=node
        self.args=args
        self.state_file=Path(args.state_file).expanduser().absolute()
        self.lock=threading.Lock()
        self.state=empty_sample()
        self.box_sizes=SpawnedBoxSizes(args.repo,manifest=args.binding_file)
        self.callbacks={}
        self.subscriptions={}
        self.camera_last={'scale':0.0,'cctv':0.0}
        self.stop=threading.Event()
        self.gazebo_topic=''
        self.binding=None
        self.binding_error=''
        self.bound_identity=None
        self.world=args.world
        self.generation=0
        self._last_discovery_error=''
        self.ros_pose_last=0.0
        self.last_stream_frame=0.0
        self.gazebo_stream_error=''
        self.gazebo_named_warning=False
        self.node.create_timer(3.0,self.discover_subscriptions)
        self.node.create_timer(3.0,self.box_sizes.refresh)
        self.node.create_timer(args.write_interval,self.write_state)
        self.refresh_binding()
        self.discover_subscriptions()
        self.gazebo_thread=threading.Thread(target=self.gazebo_loop,daemon=True)
        self.gazebo_thread.start()

    def refresh_binding(self):
        try:
            bound=load_binding(self.args.repo,self.args.binding_file)
            if bound is not None and self.args.world and self.args.world!=bound['world']:
                raise BindingError('--world disagrees with active run manifest')
            if bound is not None and self.args.gazebo_pose_topic and not self.args.gazebo_pose_topic.startswith(
                    '/world/'+bound['world']+'/'):
                raise BindingError('configured pose topic disagrees with active run world')
            message=''
        except (BindingError,OSError) as ex:
            bound=None
            message='통합 manifest 오류: '+str(ex)[:140]
        ident=run_identity(bound)
        if message and message != self.binding_error:
            self.node.get_logger().warning(message)
        self.binding_error=message
        if ident!=self.bound_identity:
            # Invalidate ALL sources on run switch, including old camera timestamps.
            self.generation+=1
            with self.lock:
                self.state=empty_sample()
                if message:self.state['gazebo']['error']=message
            for sub in self.subscriptions.values():
                self.node.destroy_subscription(sub)
            self.subscriptions={}
            self.gazebo_topic=''
            self.ros_pose_last=0.0
            self.last_stream_frame=0.0
            self.bound_identity=ident
        self.binding=bound
        self.world=bound['world'] if bound else self.args.world
        return not bool(message)

    def discover_subscriptions(self):
        if not self.refresh_binding():return
        from sensor_msgs.msg import JointState, Image
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage
        try:
            from control_msgs.msg import JointTrajectoryControllerState
        except ImportError:
            JointTrajectoryControllerState=None
        graph=dict(self.node.get_topic_names_and_types())
        sensors=[x for x,t in graph.items() if 'sensor_msgs/msg/Image' in t]
        topics=self.binding['topics'] if self.binding else {}
        def wanted(key,original):return original or topics.get(key,'')
        choices={
            'joint_states':(wanted('joint_states',self.args.joint_topic),'sensor_msgs/msg/JointState',
                            ['/joint_states']+[x for x in graph if x.endswith('/joint_states')]),
            'scale':(wanted('scale_camera',self.args.scale_camera_topic),'sensor_msgs/msg/Image',
                     ['/pac/scale_camera/image']+[x for x in sensors if 'scale' in x.lower()]),
            'cctv':(wanted('cctv_camera',self.args.cctv_camera_topic),'sensor_msgs/msg/Image',
                    ['/pac/top_camera/image']+[x for x in sensors if
                      'top_camera' in x.lower() or 'cctv' in x.lower()]),
            'suction':(wanted('suction',self.args.suction_topic),'std_msgs/msg/String',
                       ['/pac/suction/state']+[x for x in graph if x.endswith('/suction/state')]),
            'gazebo_ros':(wanted('gazebo_ros',self.args.gazebo_ros_topic),'tf2_msgs/msg/TFMessage',
                          [x for x,types in graph.items()
                           if re.fullmatch(r'/world/[^/]+/pose/info',x)
                           and (not self.world or x.startswith('/world/'+self.world+'/'))
                           and 'tf2_msgs/msg/TFMessage' in types]),
            'controller':(wanted('controller',self.args.controller_topic),'control_msgs/msg/JointTrajectoryControllerState',
                          [x for x,types in graph.items()
                           if 'control_msgs/msg/JointTrajectoryControllerState' in types]),
        }
        types={'joint_states':JointState,'scale':Image,'cctv':Image,'suction':String,
               'gazebo_ros':TFMessage, 'controller':JointTrajectoryControllerState}
        for key,(requested,msgtype,candidates) in choices.items():
            if types[key] is None:continue
            if key in self.subscriptions:
                current=self.subscriptions[key]
                existing=self.state['cameras'][key]['topic'] if key in ('scale','cctv') else (
                    self.state['gazebo']['topic'] if key=='gazebo_ros' else self.state[key]['topic'])
                # A subscribed ROS topic can disappear/reappear with the next launch.
                if existing in graph and msgtype in graph[existing] and (not requested or requested==existing):
                    continue
                self.node.destroy_subscription(current)
                del self.subscriptions[key]
                with self.lock:
                    blank=empty_sample()
                    if key in ('scale','cctv'):self.state['cameras'][key]=blank['cameras'][key]
                    elif key=='gazebo_ros':self.state['gazebo']=blank['gazebo'];self.ros_pose_last=0
                    else:self.state[key]=blank[key]
            try:topic=choose_topic(graph,requested,msgtype,candidates)
            except ValueError as ex:
                msg=str(ex)
                if msg!=self._last_discovery_error:
                    self.node.get_logger().warning(msg)
                    self._last_discovery_error=msg
                continue
            if not topic:continue
            # Do not pick a random ROS topic when multiple candidates exist.
            if not requested and key=='gazebo_ros' and len(candidates)>1:
                continue
            if key in ('scale','cctv'):
                cb=lambda msg,k=key:self.on_camera(k,msg)
            elif key=='joint_states':cb=self.on_joints
            elif key=='suction':cb=self.on_suction
            elif key=='gazebo_ros':cb=self.on_gazebo_tf
            else:cb=self.on_controller
            from rclpy.qos import qos_profile_sensor_data
            subscription=self.node.create_subscription(types[key],topic,cb,qos_profile_sensor_data)
            self.subscriptions[key]=subscription
            with self.lock:
                if key in ('scale','cctv'):self.state['cameras'][key]['topic']=topic
                elif key=='gazebo_ros':self.state['gazebo']['topic']=topic
                else:self.state[key]['topic']=topic
            self.node.get_logger().info(f'[읽기전용] {key} ← {topic} ({msgtype})')

    def on_joints(self,msg):
        now=time.time()
        joints=[]
        for i,name in enumerate(msg.name[:64]):
            if i>=len(msg.position):continue
            p=float(msg.position[i]); v=float(msg.velocity[i]) if i<len(msg.velocity) else None
            joints.append({'name':str(name)[:120],'position':p,'velocity':v})
        with self.lock:
            self.state['joint_states'].update(observed_epoch_s=now,joints=joints)

    def on_suction(self,msg):
        with self.lock:
            # This is a ROS topic state string, NOT an independent vacuum sensor.
            self.state['suction'].update(observed_epoch_s=time.time(),value=str(msg.data)[:200])

    def on_controller(self,msg):
        positions=getattr(msg.error,'positions',())
        error=max((abs(float(v)) for v in positions),default=None)
        with self.lock:
            self.state['controller'].update(observed_epoch_s=time.time(),max_abs_position_error=error)

    def on_camera(self,key,msg):
        now=time.time()
        if now-self.camera_last[key]<0.7:return  # limit CPU/Gazebo load
        self.camera_last[key]=now
        try:
            raw=image_to_jpeg(msg,self.args.max_image_width)
            target=self.state_file.with_name(f'camera_{key}.jpg')
            atomic_write(target,raw)
        except (ImportError,ValueError,TypeError,OSError) as ex:
            self.node.get_logger().warning(f'{key}: camera JPEG conversion unavailable ({type(ex).__name__})')
            return
        with self.lock:self.state['cameras'][key]['observed_epoch_s']=time.time()

    def on_gazebo_tf(self,msg):
        now=time.time()
        if now-self.ros_pose_last<0.18:return  # about 5 Hz
        poses=self.box_sizes.apply(parse_ros_pose_tf(msg,self.args.entity_pattern))
        with self.lock:
            # Fortress Pose_V -> TFMessage can have empty child_frame_id.
            # NEVER call an empty result 'fresh' or disable native fallback.
            if poses and now-self.last_stream_frame>2:
                # A ROS Pose_V->TFMessage bridge may omit model names/boxes.
                # Prefer the native named Gazebo stream whenever available.
                self.ros_pose_last=now
                self.state['gazebo'].update(observed_epoch_s=now,entities=poses,error='')
            elif not self.gazebo_named_warning:
                self.gazebo_named_warning=True
                self.node.get_logger().warning(
                    'ROS Gazebo Pose_V/TFMessage에서 모델 이름이 비었습니다. '
                    '원본 Gazebo 텍스트 스트림을 우선 수집합니다.')

    def gazebo_loop(self):
        """Read named Gazebo poses from only ONE selected world, auto-rebind on relaunch."""
        while not self.stop.is_set():
            if self.binding_error:
                self.stop.wait(2.0);continue
            generation=self.generation
            expected=self.world
            topic_override=self.args.gazebo_pose_topic or (
                self.binding['topics'].get('gazebo_pose','') if self.binding else '')
            topic=discover_gazebo_topic(self.args.gazebo_cli,topic_override,world=expected)
            # Reject ambiguity, or a vanished world; never retain a stale topic.
            if topic!=self.gazebo_topic:
                self.gazebo_topic=topic
                self.last_stream_frame=0.0
                with self.lock:
                    self.state['gazebo'].update(topic=topic,observed_epoch_s=None,entities=[],error='')
            if not topic:
                with self.lock:
                    self.state['gazebo']['error']=(
                        '활성 Gazebo 위치 토픽을 발견하지 못했습니다. 월드 다중 실행이면 manifest에 월드/토픽을 지정하세요.')
                self.stop.wait(3.0)
                continue
            cli=shutil.which(self.args.gazebo_cli)
            if not cli:
                with self.lock:self.state['gazebo']['error']='ign/gz CLI 실행 파일 없음'
                self.stop.wait(3.0);continue
            proc=None
            try:
                proc=subprocess.Popen([cli,'topic','-e','-t',topic],
                    stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,bufsize=0)
                decoder=PoseTextDecoder();fd=proc.stdout.fileno()
                last_bytes=time.monotonic()
                while not self.stop.is_set() and proc.poll() is None:
                    if self.generation!=generation or self.world!=expected or self.gazebo_topic!=topic:
                        break
                    readable,_,_=select.select([fd],[],[],.2)
                    if readable:
                        chunk=os.read(fd,65536)
                        if not chunk:break
                        last_bytes=time.monotonic()
                        frames=decoder.feed(chunk.decode('utf-8','replace'))
                    else:
                        last=decoder.flush_idle(.35)
                        frames=[last] if last else []
                    for frame in frames[-1:]:
                        if time.time()-self.last_stream_frame<self.args.gazebo_interval:
                            continue
                        pattern=self.args.entity_pattern
                        if self.box_sizes.allowed_names:
                            # Exact registered IDs supplement (not replace) the default regex.
                            names='|'.join(re.escape(n) for n in sorted(self.box_sizes.allowed_names)[:500])
                            pattern='(?:'+pattern+'|^(?:'+names+')$)'
                        poses=self.box_sizes.apply(parse_ign_pose_text(frame,pattern,limit=400))
                        poses=prefer_top_level(poses,self.box_sizes.allowed_names)
                        if poses and self.generation==generation:
                            now=time.time()
                            with self.lock:
                                self.last_stream_frame=now
                                self.state['gazebo'].update(topic=topic,observed_epoch_s=now,
                                    entities=poses,error='')
                    # Data sources may disappear without the transport echo exiting.
                    if time.monotonic()-last_bytes>7:
                        break
                if self.generation==generation and time.time()-self.last_stream_frame>7:
                    with self.lock:self.state['gazebo']['error']=(
                        'Gazebo pose 수신이 중단됐거나 아직 시작되지 않았습니다. 연결을 재탐색합니다.')
            except (OSError,ValueError,subprocess.SubprocessError) as ex:
                with self.lock:self.state['gazebo']['error']='Gazebo 원본 스트림 수신 오류: '+type(ex).__name__
            finally:
                if proc is not None:
                    if proc.poll() is None:
                        proc.terminate()
                        try:proc.wait(timeout=1.0)
                        except subprocess.TimeoutExpired:
                            proc.kill();proc.wait(timeout=1.0)
                    if proc.stdout:proc.stdout.close()
            self.stop.wait(1.5)

    def write_state(self):
        with self.lock:data=copy.deepcopy(self.state)
        data['collected_epoch_s']=time.time()
        try:
            validate_live(data)
            atomic_write(self.state_file,json.dumps(data,ensure_ascii=False,separators=(',',':'),
                                                  allow_nan=False).encode('utf-8'))
        except (OSError,ValueError,TypeError) as ex:
            self.node.get_logger().warning('상태 파일 저장 실패: '+type(ex).__name__)

    def close(self):
        self.stop.set()
        self.gazebo_thread.join(timeout=2)


def make_parser():
    parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    sub=parser.add_subparsers(dest='command',required=True)
    for name in ('discover','run'):
        p=sub.add_parser(name)
        p.add_argument('--repo',type=Path,default=Path.home()/'AHEAD/pac2026_integrated')
        p.add_argument('--state-file',type=Path)
        for key in ('joint-topic','scale-camera-topic','cctv-camera-topic','suction-topic','controller-topic'):
            p.add_argument('--'+key,default='')
        p.add_argument('--gazebo-cli',choices=('ign','gz'),default='ign')
        p.add_argument('--world',default='',help='다중 월드일 때 연결할 정확한 Gazebo 월드 이름')
        p.add_argument('--binding-file',type=Path,default=None,help='통합 실행기 active_run.json (기본: repo/logs/pac_dashboard_integration/active_run.json)')
        p.add_argument('--gazebo-pose-topic',default='')
        p.add_argument('--gazebo-ros-topic',default='',help='ROS2로 브리지된 전용 Gazebo Pose_V TFMessage 토픽')
        p.add_argument('--entity-pattern',default=r'(?i)box|pallet|buffer|hdr50|robot|suction')
        p.add_argument('--max-image-width',type=int,default=720)
        p.add_argument('--write-interval',type=float,default=0.25)
        p.add_argument('--gazebo-interval',type=float,default=0.2)
    return parser


def main():
    args=make_parser().parse_args()
    args.state_file=args.state_file or args.repo/'logs'/'pac_dashboard_live'/'telemetry.json'
    if not 160<=args.max_image_width<=1920 or not 0.2<=args.write_interval<=10 or not 0.15<=args.gazebo_interval<=20:
        raise SystemExit('이미지 크기·수집 간격 인자가 허용 범위를 벗어났습니다.')
    try:re.compile(args.entity_pattern)
    except re.error as ex:raise SystemExit('잘못된 엔티티 필터 정규식') from ex
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as ex:
        raise SystemExit('ROS2 Humble 환경을 source한 뒤 실행하세요 (rclpy 필요).') from ex
    rclpy.init()
    node=Node('pac_dashboard_readonly_observer')
    try:
        if args.command=='discover':
            print('--- 실제 ROS2 토픽 목록 (계측 시점) ---')
            for topic,types in sorted(node.get_topic_names_and_types()):
                if any(x in ','.join(types) for x in ('Image','JointState','ControllerState','String','TFMessage')):
                    print(topic, ','.join(types))
            print('--- Gazebo pose/info ---')
            if args.gazebo_pose_topic:print(args.gazebo_pose_topic)
            else:
                p=subprocess.run([args.gazebo_cli,'topic','-l'],capture_output=True,text=True,
                                 timeout=5,check=False)
                print('\n'.join(x for x in p.stdout.splitlines() if '/pose/info' in x) or '(없음)')
                print('활성 선택:',discover_gazebo_topic(args.gazebo_cli,args.gazebo_pose_topic,world=args.world) or '(모호/없음)')
            return 0
        print('ROS2/Gazebo 읽기전용 관측 시작; 파일:',args.state_file)
        print('진단 서비스의 실시간 상태는 실제 토픽이 발견될 때만 갱신됩니다.')
        observer=ReadOnlyObserver(node,args)
        try:rclpy.spin(node)
        except KeyboardInterrupt:pass
        finally:observer.close()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__=='__main__':
    raise SystemExit(main())
