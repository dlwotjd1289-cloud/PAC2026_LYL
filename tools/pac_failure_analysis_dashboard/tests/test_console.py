import asyncio
import json
import time
import uuid
from pathlib import Path

import pytest
from aiohttp import ClientSession
from aiohttp.test_utils import TestServer, TestClient

import pac_diagnostic_console as pac


def model(kind='diagnosis', ids=None):
    ids = ids or []
    return {
        'schema_version': pac.VERSION, 'kind': kind, 'answer': '실패 여부는 기록으로만 확인됩니다.',
        'fact_ids': ids, 'hypotheses': [], 'checks': [],
        'actions': [{'instruction':'관제 로그를 확인하세요.', 'requires_operator': True}],
        'missing_information': [],
    }


def response(answer=None, status='completed'):
    answer = answer if answer is not None else model()
    return {'status': status, 'output': [{'type':'message', 'content': [
        {'type': 'output_text', 'text': json.dumps(answer, ensure_ascii=False)}]}]}


def context(run='R1', evidence=None):
    c = pac.no_context()
    c['run_id'] = run
    c['evidence'] = evidence or []
    return c


class Evidence:
    def __init__(self, c=None):self.value = c if c is not None else context()
    def snapshot(self):return self.value


@pytest.mark.parametrize('bad', [
    None, [], 'hello', {'status':'completed','output':None},
    {'status':'completed','output':[None]},
    {'status':'completed','output':[{'type':'message','content':None}]},
    {'status':'completed','output':[{'type':'message','content':[None]}]},
    {'status':'completed','output':[{'type':'message','content':['x']}]},
    {'status':'completed','output':[{'type':'message','content':[{'type':'output_text','text':None}]}]},
    {'status':'completed','output':[{'type':'message','content':[{'type':'output_text','text':'[]'}]}]},
    {'status':'completed','output':[{'type':'message','content':[{'type':'output_text','text':'{}'}]}]},
    {'status':'incomplete','output':[]},
])
def test_provider_malformed_returns_typed_error(bad):
    with pytest.raises(pac.ProviderError):pac.parse_provider(bad)


def test_provider_refusal():
    with pytest.raises(pac.ProviderError) as exc:
        pac.parse_provider({'status':'completed','output':[{'type':'message','content':[{'type':'refusal','refusal':'no'}]}]})
    assert exc.value.status == 'refused'


def test_provider_valid():
    assert pac.parse_provider(response(model('general'))) == model('general')


def test_duplicate_json_or_extra_fields_rejected():
    with pytest.raises(pac.InvalidOutput):pac.strict_json('{"x":1,"x":2}')
    with pytest.raises(pac.InvalidOutput):pac.strict_json('{"x":NaN}')
    with pytest.raises(pac.ProviderError):pac.parse_provider(response({**model(), 'extra':'bad'}))
    with pytest.raises(pac.ProviderError):pac.parse_provider(response({**model(), 'actions':[{'instruction':'delete logs','requires_operator':False}]}))


def test_sanitize_only_values_preserves_schema():
    key='sk-ABCDEF12345678901234567890'
    answer=model()
    answer['answer']='error password=12345, context={} '+key+' Bearer '+key
    answer['checks']=[{'what':'api_key=secret!', 'why':'log contains sk-ZZZZZZZZZZZZZZZZ'}]
    sanitized=pac.sanitize_strings(answer, key)
    pac.validate(sanitized,pac.MODEL_SCHEMA)
    assert key not in json.dumps(sanitized)
    assert '12345' not in str(sanitized)
    assert set(sanitized)==set(answer)
    assert sanitized['checks'][0]['why'].endswith('[REDACTED]')


def test_log_prio_single_run_and_age_not_heartbeat(tmp_path):
    repo=tmp_path/'repo'; root=repo/'logs'/'v44_generator_cycle'
    run=root/'one';run.mkdir(parents=True)
    important=run/'moveit_executor.log'; important.write_text('MOVEIT FAIL: Cartesian fraction 0.64\n')
    # Many more recently modified unrelated background logs must not remove executor log.
    for i in range(16):
        p=run/f'topic_{i:02d}.log'
        p.write_text(f'PERCEPTION PASS #{i}\n')
        import os
        os.utime(p, (time.time()+i, time.time()+i))
    next_run=root/'other';next_run.mkdir();(next_run/'executor.log').write_text('ERROR old other run\n')
    log=pac.LogEvidence(repo,run)
    c=log.snapshot()
    assert c['run_id']=='one'
    assert c['observation']=='failure_logged'
    assert c['alert']['source'].endswith('moveit_executor.log')
    assert all('other' not in e['source'] for e in c['evidence'])
    pac.validate(c,pac.CONTEXT_SCHEMA)


def test_invalid_run_dir_outside_logs(tmp_path):
    repo=tmp_path/'repo'; (repo/'logs').mkdir(parents=True)
    with pytest.raises(ValueError):pac.LogEvidence(repo, tmp_path/'elsewhere')


def test_invented_evidence_fails():
    m=model(ids=['made-up'])
    with pytest.raises(pac.InvalidOutput):pac.assemble(m,context(),str(uuid.uuid4()))


def test_ask_model_contains_single_current_run_and_context_reset():
    async def scenario():
        captured=[]
        async def fake(_http,_key,body):
            captured.append(body)
            return response(model('general'))
        ev=Evidence(context('old'))
        service=pac.DiagnosticService(ev,'sk-dummy-SECRET-for-test', transport=fake)
        sid=str(uuid.uuid4())
        first=await service.ask('첫 질문',sid)
        assert first['status']=='ok'
        service.context=context('new')
        second=await service.ask('두 번째 질문',sid)
        assert second['status']=='ok'
        assert captured[0]['input'][0]['role']=='user'
        assert not json.loads(captured[1]['input'][0]['content'])['previous_turns']
        assert json.loads(captured[1]['input'][0]['content'])['context']['run_id']=='new'
    asyncio.run(scenario())


def test_slow_api_does_not_block_state_or_alert_and_busy():
    async def scenario():
        gate=asyncio.Event()
        async def fake(_http,_key,_body):
            await gate.wait()
            return response(model('general'))
        service=pac.DiagnosticService(Evidence(context()),'sk-DUMMY-API-KEY',transport=fake)
        async with TestServer(pac.make_app(service)) as server:
            async with TestClient(server) as client:
                sid=str(uuid.uuid4())
                t=asyncio.create_task(client.post('/api/ask',json={'question':'왜 실패?', 'session_id':sid}))
                await asyncio.sleep(.05)
                busy=await client.post('/api/ask',json={'question':'중복', 'session_id':sid})
                assert busy.status==429
                assert (await busy.json())['status']=='busy'
                c=context('new_run')
                c['observation']='failure_logged';c['alert']={'id':'abc','text':'executor FAIL','source':'executor.log'}
                service.evidence.value=c
                await asyncio.sleep(2.2)  # background poll should run, not wait for API
                state=await client.get('/api/state')
                data=await state.json()
                assert data['context']['alert']['text']=='executor FAIL'
                gate.set()
                r=await t
                assert (await r.json())['context']['run_id']=='R1'  # request snapshot immutability
    asyncio.run(scenario())


def test_http_same_schema_and_invalid_input():
    async def scenario():
        async def fake(_http,_key,_body):return response(model('general'))
        service=pac.DiagnosticService(Evidence(context()),'sk-DUMMY-API-KEY',transport=fake)
        async with TestServer(pac.make_app(service)) as server:
            async with TestClient(server) as client:
                for payload in ['{no-json}', '{"question":"a","question":"b"}',
                                '{"question":"x","extra":1}']:
                    r=await client.post('/api/ask',data=payload,headers={'Content-Type':'application/json'})
                    assert r.status==400
                    pac.validate(await r.json(),pac.REPLY_SCHEMA)
                r=await client.post('/api/ask',json={'question':'무슨 상태야?'})
                assert r.status==200
                o=await r.json();pac.validate(o,pac.REPLY_SCHEMA)
                assert o['source']=='openai'
                assert o['control']=={'mode':'read_only','command_executed':False}
                # No second website may POST with an arbitrary Origin.
                r=await client.post('/api/ask',json={'question':'hey'},headers={'Origin':'http://evil.test'})
                assert r.status==403
                r=await client.get('/api/state',headers={'Host':'evil.test'})
                assert r.status==403
    asyncio.run(scenario())


@pytest.mark.parametrize('error', [
    pac.ProviderError('unavailable','401 key invalid'),
    pac.ProviderError('unavailable','429 rate limit'),
    pac.ProviderError('unavailable','timed out'),
    pac.ProviderError('refused','refusal'),
    pac.ProviderError('invalid_response','incomplete')
])
def test_provider_errors_have_uniform_schema(error):
    async def scenario():
        async def fake(_http,_key,_body):raise error
        service=pac.DiagnosticService(Evidence(),'sk-DUMMY-API-KEY', transport=fake)
        result=await service.ask('왜?')
        pac.validate(result,pac.REPLY_SCHEMA)
        assert result['status']==error.status
        assert result['source']=='local'
    asyncio.run(scenario())


def test_model_echoed_secret_is_hidden():
    key='sk-SECRET-1234567890123456'
    async def scenario():
        async def fake(_http,_key,_body):
            m=model('general')
            m['answer']='password=abc123 and Bearer '+key
            m['actions']=[{'instruction':'api_key: abcdefgh', 'requires_operator':True}]
            return response(m)
        service=pac.DiagnosticService(Evidence(),key,transport=fake)
        result=await service.ask('안녕')
        pac.validate(result,pac.REPLY_SCHEMA)
        assert 'abc123' not in str(result) and key not in str(result) and 'abcdefgh' not in str(result)
        assert result['source']=='openai'
    asyncio.run(scenario())


def test_same_schema_cli_and_http(tmp_path):
    async def scenario():
        async def fake(_http,_key,_body):return response(model('general'))
        service=pac.DiagnosticService(Evidence(),'sk-DUMMY-API-KEY',transport=fake)
        async with TestServer(pac.make_app(service)) as server:
            url=str(server.make_url('/')).rstrip('/')
            result=await asyncio.to_thread(pac.cli_ask,url,'질문')
            pac.validate(result,pac.REPLY_SCHEMA)
            assert '답변 요약' not in pac.terminal_format(result)  # terminal heading differs; same data
            assert '확인된 근거' in pac.terminal_format(result)
            assert result['source']=='openai'
    asyncio.run(scenario())


def test_no_slot_handled_without_false_alarm(tmp_path):
    repo=tmp_path/'repo';run=repo/'logs'/'v44_generator_cycle'/'r1';run.mkdir(parents=True)
    (run/'planner.log').write_text('PLAN FAIL: NO_SLOT (handled buffered)\n')
    context=pac.LogEvidence(repo,run).snapshot()
    assert context['observation']!='failure_logged'
    assert pac.failure_stage(context)['stage']=='none'
    (run/'moveit_executor.log').write_text('MOVEIT PICK&PLACE FAIL: descend: Cartesian path only 64% feasible')
    context=pac.LogEvidence(repo,run).snapshot()
    assert context['observation']=='failure_logged'
    assert pac.failure_stage(context)['stage']=='moveit_cartesian_plan'
    assert '구체 원인은 미확정' in pac.failure_stage(context)['description']


def test_auto_run_chooses_active_log_in_old_directory(tmp_path):
    import os
    repo=tmp_path/'repo';root=repo/'logs'/'v44_generator_cycle';root.mkdir(parents=True)
    old=root/'old';new=root/'new';old.mkdir();new.mkdir()
    a=old/'executor.log';a.write_text('MOVEIT PICK&PLACE FAIL: old run resumed\n')
    b=new/'executor.log';b.write_text('PASS\n')
    now=time.time()
    os.utime(old,(now-360,now-360));os.utime(new,(now-10,now-10))
    os.utime(a,(now,now));os.utime(b,(now-10,now-10))
    assert pac.LogEvidence(repo).snapshot()['run_id']=='old'
