#!/usr/bin/env python3
"""PAC2026 read-only, plan-only negative test for the diagnostic dashboard.

Modes:
  sample      - synthetic, explicitly labeled log to verify UI/log handling.
  moveit-ik   - sends ONLY moveit_msgs/srv/GetPositionIK for an unreachable
                reference pose. NO trajectory action, no robot movement, no
                Gazebo model modification, no process termination.

New output directories are created under logs/pac_dashboard_test_scenarios.
Historical generator logs are never modified.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time
import uuid

GROUP = 'hdr_manipulator'
TIP = 'suction_tcp'
FRAME = 'world'
TARGET = (25.0, 25.0, 25.0)  # m, deliberately far from the robot work envelope


def now():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds')


def negative_ik_result(wait_s=8.0, request_s=2.0):
    """One read-only service request. ROS imports only in the real-IK mode."""
    import rclpy
    from moveit_msgs.srv import GetPositionIK
    from rclpy.node import Node
    rclpy.init(args=None)
    node = Node('pac_dashboard_plan_only_negative_test')
    try:
        client = node.create_client(GetPositionIK, '/compute_ik')
        if not client.wait_for_service(timeout_sec=wait_s):
            raise RuntimeError('MoveIt /compute_ik 서비스를 찾지 못했습니다. Gazebo 뒤에 MoveIt을 실행하세요.')
        req = GetPositionIK.Request()
        ik = req.ik_request
        ik.group_name = GROUP
        ik.ik_link_name = TIP
        ik.pose_stamped.header.frame_id = FRAME
        ik.pose_stamped.pose.position.x = TARGET[0]
        ik.pose_stamped.pose.position.y = TARGET[1]
        ik.pose_stamped.pose.position.z = TARGET[2]
        ik.pose_stamped.pose.orientation.w = 1.0
        ik.robot_state.is_diff = True
        ik.avoid_collisions = True
        ik.timeout.sec = int(request_s)
        ik.timeout.nanosec = int((request_s-int(request_s))*1e9)
        started = time.monotonic()
        # Does not invoke MoveGroup or ExecuteTrajectory. This is ONLY /compute_ik.
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=wait_s+request_s+1.0)
        if not future.done():
            raise RuntimeError('MoveIt /compute_ik 결과 수신 시간 초과: 결과를 알 수 없어 실패로 기록하지 않습니다.')
        if future.exception() is not None:
            raise RuntimeError(f'MoveIt IK 서비스 예외: {type(future.exception()).__name__}')
        res = future.result()
        if res is None:
            raise RuntimeError('MoveIt /compute_ik 응답이 없습니다.')
        code = int(res.error_code.val)
        code_names = {int(v): k for k,v in vars(type(res.error_code)).items()
                      if k.isupper() and type(v) is int}
        known = {-31:'NO_IK_SOLUTION',-15:'INVALID_GROUP_NAME',
                 -16:'INVALID_GOAL_CONSTRAINTS',-17:'INVALID_ROBOT_STATE',
                 -18:'INVALID_LINK_NAME',-19:'INVALID_OBJECT_NAME',
                 -21:'FRAME_TRANSFORM_FAILURE',-6:'TIMED_OUT',
                 -1:'PLANNING_FAILED',1:'SUCCESS'}
        return code, (code_names.get(code) or known.get(code) or 'UNKNOWN_CODE'), round(time.monotonic()-started, 3)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def create_record(repo, *, mode, code, code_name, elapsed_s=None):
    repo = Path(repo).expanduser().resolve()
    if not repo.is_dir():
        raise ValueError(f'저장소 경로가 없습니다: {repo}')
    root = repo / 'logs' / 'pac_dashboard_test_scenarios'
    if root.is_symlink() or (repo / 'logs').is_symlink():
        raise ValueError('심볼릭 링크 logs 디렉터리는 사용하지 않습니다.')
    root.mkdir(parents=True, exist_ok=True)
    run_id = f'TEST_IK_{mode.upper().replace("-","_")}_{datetime.now().strftime("%Y%m%d_%H%M%S")}_{uuid.uuid4().hex[:6]}'
    run = root / run_id
    run.mkdir(exist_ok=False)
    mode_caption = '합성 실패 로그(실제 ROS 요청 없음)' if mode == 'sample' else '실제 MoveIt IK 서비스 응답(로봇 이동 없음)'
    lines = [
        f'{now()} [TEST-ONLY] 시작: {mode_caption}',
        f'{now()} [TEST-ONLY] request=/compute_ik; group={GROUP}; tip={TIP}; frame={FRAME}; target_xyz_m={TARGET}; execute=false',
        f'{now()} [TEST-ONLY] MOVEIT IK FAIL: code={code} ({code_name}); '
        f'{"synthetic response / no ROS request" if mode == "sample" else "actual GetPositionIK response"}; '
        'NO ROBOT MOTION COMMANDED',
    ]
    log_file = run / 'box_TEST_moveit.log'
    log_file.write_text('\n'.join(lines)+'\n', encoding='utf-8')
    record = {
        'schema_version':'1.0', 'run_id':run_id, 'mode':mode,
        'is_test': True, 'is_physical_failure': False,
        'description':mode_caption, 'created_at':now(),
        'endpoint':'/compute_ik' if mode == 'moveit-ik' else 'none',
        'request':{'group':GROUP,'ik_link_name':TIP,'frame_id':FRAME,'target_xyz_m':TARGET,
                   'avoid_collisions':True, 'execute':False},
        'response':{'error_code':code,'error_name':code_name,'duration_s':elapsed_s},
        'robot_command_sent':False, 'gazebo_modified':False,
        'log_file': str(log_file.relative_to(repo)),
    }
    (run / 'test_manifest.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return run, record


def main(argv=None):
    ap=argparse.ArgumentParser(description='PAC V5.1 테스트용 의도적 IK 실패 시나리오(로봇 이동 없음)')
    ap.add_argument('--repo',type=Path,default=Path.home()/'AHEAD'/'pac2026_integrated')
    ap.add_argument('--mode',choices=['sample','moveit-ik'],default='moveit-ik')
    ap.add_argument('--wait',type=float,default=8.0,help='MoveIt 서비스 대기 초')
    args=ap.parse_args(argv)
    if args.mode=='sample':
        code,name,elapsed = -31,'NO_IK_SOLUTION',None
    else:
        print('MoveIt /compute_ik 도달불가능 목표 검사 중 — 경로 실행·실제 로봇 이동은 하지 않습니다.',flush=True)
        try:
            code,name,elapsed = negative_ik_result(args.wait)
        except Exception as exc:
            print(f'중단: {type(exc).__name__}: {exc}', file=sys.stderr)
            print('실제 MoveIt 응답을 받지 못해 TEST 실패 기록을 생성하지 않았습니다.',file=sys.stderr)
            return 2
        if code == 1:
            print('예상 밖 SUCCESS 응답: 거부 사례가 아니므로 실패 로그를 생성하지 않습니다.',file=sys.stderr)
            return 3
    run, r = create_record(args.repo,mode=args.mode,code=code,code_name=name,elapsed_s=elapsed)
    print(json.dumps({'ok':True,'test':True,'mode':args.mode,'run_dir':str(run),
        'error_code':code,'error_name':name,'robot_command_sent':False},ensure_ascii=False,indent=2))
    print('이 기록은 실제 박스 적재 실패가 아닙니다. 테스트 전용 4180 포트에서 확인하세요.')
    return 0


if __name__=='__main__':
    raise SystemExit(main())
