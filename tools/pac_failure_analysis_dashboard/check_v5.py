#!/usr/bin/env python3
"""Read-only, key-free V5 diagnostics for the local PAC dashboard."""
from __future__ import annotations
import argparse
import json
import urllib.error
import urllib.request


def get_json(url):
    try:
        with urllib.request.urlopen(url,timeout=3) as r:
            return json.load(r)
    except (OSError,ValueError,urllib.error.URLError):
        return None


def diagnosis(root):
    robot=get_json(root+'/api/robot3d')
    scene=get_json(root+'/api/scene3d')
    state=get_json(root+'/api/state')
    print('=== PAC Gazebo V5 진단 (조회 전용) ===')
    if scene is None or robot is None or state is None:
        print('대시보드 HTTP 접속 실패. start_dashboard_v5.sh 터미널 출력을 확인하세요.')
        return 1
    print('월드:',scene.get('world') or '(없음)', '고정 형상:',len(scene.get('primitives',[])),
          '오류:',scene.get('error') or '없음')
    print('로봇 URDF 관절:',len(robot.get('joints',[])),
          'STL 시각 형상:',len(robot.get('meshes',[])),
          'SRDF home 참고 관절:',len(robot.get('reference_joints',[])))
    if robot.get('error'):print('모델:',robot['error'])
    live=state.get('telemetry') or {}
    ages=live.get('source_age_s') or {}
    data=live.get('data') or {}
    js=data.get('joint_states') or {}
    gz=data.get('gazebo') or {}
    print('관측 파일:', '있음' if live.get('available') else '없음/유효성 오류',
          '최신:',live.get('fresh'),'관측 파일 나이:',live.get('age_s'))
    print('ROS 관절:', len(js.get('joints',[])),'개', '토픽:',js.get('topic'),
          '수신 나이:',ages.get('joint_states'))
    print('Gazebo 엔티티:',len(gz.get('entities',[])),'개','토픽:',gz.get('topic'),
          '수신 나이:',ages.get('gazebo'))
    print('Gazebo 오류:',gz.get('error') or '없음')
    print('카메라 JPEG 유효성:',live.get('camera_jpeg_ok',{}))
    print('카메라 JPEG 최근 수정 경과:',live.get('camera_jpeg_age_s',{}))
    if not all(live.get('camera_jpeg_ok',{}).values()):
        print('조치: 브리지 logs/pac_dashboard_3d_v5/camera_bridge.log 및 observer.log 확인 (영상 없으면 0/2가 정상)')
    if not robot.get('joints'):
        print('조치: repo 경로의 ros2_ws/src/pac_bringup/urdf/hdr50_22_pedestal.urdf.xacro 확인')
    elif not robot.get('meshes'):
        print('조치: ros2_ws/src/hdr_description의 STL 설치 경로 확인 (없으면 간략형).')
    if ages.get('joint_states') is None or ages['joint_states']>5:
        print('조치: ROS 2 환경에서 `timeout 6 ros2 topic echo /joint_states --once` 실행하고 결과 확인')
    if ages.get('gazebo') is None or ages['gazebo']>5:
        print('조치: `ign topic -l | grep pose/info` 및 logs/pac_dashboard_3d_v5/observer.log 확인')
    return 0


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',type=int,default=4178)
    args=p.parse_args()
    if not 1024<=args.port<=65535:raise SystemExit('port must be 1024..65535')
    raise SystemExit(diagnosis(f'http://127.0.0.1:{args.port}'))
