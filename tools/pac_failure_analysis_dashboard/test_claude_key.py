#!/usr/bin/env python3
"""Optional one-request real API smoke test. NO workcell logs are sent.

This invokes a real billable Claude API request only when explicitly run.
The key is read with getpass; it is not stored or printed.
"""
import getpass
import json
import os
import sys
import urllib.error
import urllib.request

MODEL=os.environ.get('ANTHROPIC_MODEL','claude-haiku-5-5')
KEY=os.environ.get('ANTHROPIC_API_KEY') or getpass.getpass('Claude API key (hidden): ').strip()
if not KEY:
    print('API 키가 입력되지 않았습니다.');sys.exit(2)
body=json.dumps({'model':MODEL, 'max_tokens':1024, 'output_config':{'effort':'low'},
                 'messages':[{'role':'user','content':'한국어로 연결 성공이라고 한 문장만 답해줘.'}]},
                ensure_ascii=False).encode('utf-8')
req=urllib.request.Request('https://api.anthropic.com/v1/messages',data=body,
    headers={'x-api-key':KEY, 'anthropic-version':'2023-06-01',
             'content-type':'application/json'},method='POST')
try:
    with urllib.request.urlopen(req,timeout=35) as res:
        response=json.load(res)
except urllib.error.HTTPError as exc:
    print('API 호출 실패: HTTP',exc.code,'(401=키, 402=크레딧, 404=모델, 429=호출 제한)')
    sys.exit(1)
except (OSError,ValueError) as exc:
    print('API 연결 실패: 인터넷 연결·방화벽·응답 시간을 확인하세요. 상세 예외는 공개하지 않습니다.')
    sys.exit(1)
if response.get('type') != 'message' or response.get('role') != 'assistant':
    print('API 응답은 왔지만 메시지 형식이 예상과 다릅니다.');sys.exit(1)
if response.get('stop_reason') != 'end_turn' or not any(
    part.get('type') == 'text' and part.get('text') for part in response.get('content', [])
    if isinstance(part,dict)):
    print('Claude API 인증은 통과했으나 답변이 완료되지 않았습니다.');sys.exit(1)
print('Claude API 연결 성공')
print('모델:',response.get('model','알 수 없음'))
print('응답 종료:',response.get('stop_reason','알 수 없음'))
usage=response.get('usage') or {}
print('사용 토큰:', '입력', usage.get('input_tokens','?'), '출력',usage.get('output_tokens','?'))
