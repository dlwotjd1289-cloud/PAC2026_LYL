#!/usr/bin/env python3
"""Verify the isolated test dashboard's log→failure stage→offline question chain.

This script issues only local HTTP GET/POST requests; no robot actions.
"""
from __future__ import annotations

import argparse
import json
from urllib import request, error
from urllib.parse import urlsplit


def fetch(url, payload=None):
    data=None if payload is None else json.dumps(payload,ensure_ascii=False).encode('utf-8')
    headers={'Content-Type':'application/json'} if payload is not None else {}
    req=request.Request(url,data=data,headers=headers)
    with request.urlopen(req,timeout=8) as response:
        if response.status != 200:
            raise RuntimeError(f'HTTP {response.status}')
        raw=response.read(128*1024+1)
        if len(raw)>128*1024:
            raise RuntimeError('응답이 너무 큽니다.')
        return json.loads(raw)


def check(base, real_required=False):
    u=urlsplit(base)
    if u.scheme!='http' or u.hostname not in ('127.0.0.1','localhost','::1') or u.username:
        raise ValueError('로컬 대시보드 주소만 허용됩니다.')
    base=base.rstrip('/')
    state=fetch(base+'/api/state')
    ctx=state.get('context') or {}
    stage=state.get('failure_stage') or {}
    line=(ctx.get('alert') or {}).get('text','')
    src=(ctx.get('alert') or {}).get('source','')
    rid=ctx.get('run_id','')
    assert rid.startswith('TEST_IK_') and src.startswith('logs/pac_dashboard_test_scenarios/'), '테스트 전용 실행이 선택되지 않음'
    assert ctx.get('observation')=='failure_logged', '실패 기록 감지가 안 됨'
    assert '[TEST-ONLY]' in line and 'MOVEIT IK FAIL:' in line, '시나리오 메시지 확인 실패'
    assert stage.get('stage')=='moveit_ik', 'IK 실패 단계 분류가 안 됨'
    mode='sample' if 'synthetic response' in line else 'moveit-ik'
    if real_required:
        assert mode=='moveit-ik' and 'actual GetPositionIK response' in line, '실제 MoveIt 응답이 아닌 합성 로그'
    reply=fetch(base+'/api/ask',{'question':'이 테스트의 마지막 실패 원인이 뭐야?'})
    assert reply.get('schema_version')=='1.0' and reply.get('source')=='local', '고정 JSON 응답 아님'
    assert reply.get('control')=={'mode':'read_only','command_executed':False}, '읽기 전용 안전 확인 실패'
    assert reply.get('status')=='insufficient_data', '실패 근본 원인이 미확정이어야 함'
    assert reply.get('confirmed_facts') and reply['confirmed_facts'][0]['id']==ctx['alert']['id'], '근거 ID 연결 안 됨'
    print('PASS: TEST 실행 로그 선택:',rid)
    print('PASS: TEST 실패 알람 인식:',mode)
    print('PASS: 실패 단계:',stage['stage'])
    print('PASS: 고정 JSON 오프라인 답변 + 근거 ID 검증')
    print('PASS: 로봇 명령 실행 없음')
    print('결론: 대시보드의 시험 로그 감지·설명 경로 정상. 실제 로봇 적재 실패 시험은 아닙니다.')
    return 0


def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--url',default='http://127.0.0.1:4180')
    ap.add_argument('--require-real',action='store_true',help='실제 MoveIt /compute_ik 응답만 성공으로 인정')
    args=ap.parse_args(argv)
    try: return check(args.url,args.require_real)
    except (AssertionError,ValueError,RuntimeError,OSError,json.JSONDecodeError) as exc:
        print(f'FAIL: {exc}')
        return 1


if __name__=='__main__':
    raise SystemExit(main())
