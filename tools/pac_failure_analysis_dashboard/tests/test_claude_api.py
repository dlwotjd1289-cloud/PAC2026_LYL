import asyncio
import json
import uuid

import aiohttp
import pytest
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

import pac_diagnostic_console as pac


def model(kind='general', evidence_ids=()):
    return {
        'schema_version': '1.0', 'kind': kind,
        'answer': '선택 실행 기록에서 확인된 사실만 정리했습니다.',
        'fact_ids': list(evidence_ids), 'hypotheses': [], 'checks': [],
        'actions': [{'instruction': '상태를 확인하세요.', 'requires_operator': True}],
        'missing_information': [],
    }


def provider_response(payload=None, reason='end_turn'):
    return {
        'type':'message', 'role':'assistant', 'model':'claude-haiku-5-5',
        'stop_reason':reason, 'content':[{'type':'text','text':json.dumps(payload or model(),ensure_ascii=False)}],
    }


class LogEvidence:
    repo = None

    def snapshot(self):
        ctx = pac.no_context()
        ctx['run_id'] = 'TEST_CLAUDE'
        ctx['observation'] = 'failure_logged'
        ctx['evidence'] = [{
            'id': 'real_123', 'source':'logs/v44_generator_cycle/run/box_01_moveit.log',
            'text':'MOVEIT PICK&PLACE FAIL: no descent', 'file_modified_at':pac.utc(),
        }]
        ctx['alert'] = {'id':'real_123','source':ctx['evidence'][0]['source'],
                        'text':ctx['evidence'][0]['text']}
        return ctx


def test_claude_model_payload_and_schema_supported():
    req = pac.claude_request_body('claude-haiku-5-5','마지막 실패가 뭐야?', pac.no_context(), [])
    assert req['model'] == 'claude-haiku-5-5'
    assert req['messages'][0]['role'] == 'user'
    assert req['system'] == pac.INSTRUCTIONS
    assert req['output_config']['format']['type'] == 'json_schema'
    assert 'store' not in req and 'instructions' not in req and 'input' not in req
    schema=req['output_config']['format']['schema']
    assert schema['additionalProperties'] is False
    assert schema['properties']['hypotheses']['items']['additionalProperties'] is False
    assert schema['properties']['actions']['items']['properties']['requires_operator']['enum']==[True]
    assert 'maxLength' not in str(schema) and 'maxItems' not in str(schema)
    assert 'maxItems' in str(pac.MODEL_SCHEMA)  # local validation MUST stay strict


def test_claude_response_valid_thinking_and_errors():
    resp=provider_response()
    resp['content'].insert(0,{'type':'thinking','thinking':'internal'})
    assert pac.parse_claude(resp)['kind']=='general'
    for bad in (None, [], {},
                {'type':'message','role':'user','stop_reason':'end_turn','content':[]},
                {'type':'message','role':'assistant','stop_reason':'max_tokens','content':[]},
                {'type':'message','role':'assistant','stop_reason':'end_turn','content':None}):
        with pytest.raises(pac.ProviderError):pac.parse_claude(bad)
    bad=provider_response()
    bad['content'].append({'type':'text','text':'{}'})
    with pytest.raises(pac.ProviderError):pac.parse_claude(bad)
    bad=provider_response()
    bad['content'][0]['text']='{"answer":"hello"}'
    with pytest.raises(pac.ProviderError):pac.parse_claude(bad)
    bad=provider_response()
    bad['stop_reason']='refusal'
    with pytest.raises(pac.ProviderError) as exc:pac.parse_claude(bad)
    assert exc.value.status == 'refused'


@pytest.mark.parametrize('status',[400,401,403,404,429,529,302])
def test_claude_http_statuses_no_key_leak(monkeypatch,status):
    async def scenario():
        async def fake(_):return web.Response(status=status,text='secret=sk-ant-dont-repeat-this')
        app=web.Application();app.router.add_post('/v1/messages',fake)
        async with TestServer(app) as server:
            monkeypatch.setattr(pac, 'ANTHROPIC_API_URL',str(server.make_url('/v1/messages')))
            async with aiohttp.ClientSession() as session:
                with pytest.raises(pac.ProviderError) as exc:
                    await pac.claude_call(session,'sk-ant-TEST-SECRET',{})
                assert exc.value.status=='unavailable'
                assert 'sk-ant' not in str(exc.value)
    asyncio.run(scenario())


def test_claude_http_headers_and_payload(monkeypatch):
    async def scenario():
        async def fake(request):
            assert request.headers['x-api-key']=='sk-ant-TEST-SECRET'
            assert request.headers['anthropic-version']=='2023-06-01'
            assert request.headers['content-type']=='application/json'
            req=await request.json()
            assert req['model']=='claude-haiku-5-5'
            return web.json_response(provider_response())
        app=web.Application();app.router.add_post('/v1/messages',fake)
        async with TestServer(app) as server:
            monkeypatch.setattr(pac, 'ANTHROPIC_API_URL',str(server.make_url('/v1/messages')))
            async with aiohttp.ClientSession() as session:
                req=pac.claude_request_body('claude-haiku-5-5','안녕',pac.no_context(),[])
                result=await pac.claude_call(session,'sk-ant-TEST-SECRET',req)
                assert pac.parse_claude(result)['kind']=='general'
    asyncio.run(scenario())


def test_claude_dashboard_real_http_local_mock(monkeypatch):
    async def scenario():
        async def fake_provider(request):
            req=await request.json()
            assert req['model']=='claude-haiku-5-5'
            context=json.loads(req['messages'][0]['content'])['context']
            assert context['run_id']=='TEST_CLAUDE'
            return web.json_response(provider_response(model('diagnosis',['real_123'])))
        upstream=web.Application();upstream.router.add_post('/v1/messages',fake_provider)
        async with TestServer(upstream) as u:
            monkeypatch.setattr(pac,'ANTHROPIC_API_URL',str(u.make_url('/v1/messages')))
            service=pac.DiagnosticService(LogEvidence(),'sk-ant-TEST-SECRET',
                                          pac.DEFAULT_CLAUDE_MODEL,provider='claude')
            async with TestServer(pac.make_app(service)) as server:
                async with TestClient(server) as client:
                    status=await client.get('/api/state')
                    meta=await status.json()
                    assert meta['provider']=='claude' and meta['api_configured'] is True
                    assert 'TEST-SECRET' not in json.dumps(meta)
                    response=await client.post('/api/ask',json={'question':'마지막 실패?'})
                    assert response.status==200
                    obj=await response.json()
                    pac.validate(obj,pac.REPLY_SCHEMA)
                    assert obj['source']=='claude'
                    assert [e['id'] for e in obj['confirmed_facts']]==['real_123']
                    assert obj['control']=={'mode':'read_only','command_executed':False}
    asyncio.run(scenario())


def test_claude_dashboard_fake_reference_rejected():
    async def scenario():
        async def fake(_, _key, payload):
            return provider_response(model('diagnosis',['invented_id']))
        service=pac.DiagnosticService(LogEvidence(),'sk-ant-TEST-SECRET',
                                      pac.DEFAULT_CLAUDE_MODEL,transport=fake,provider='claude')
        response=await service.ask('실패 원인을 확인해 줘')
        assert response['status']=='invalid_response'
        assert response['source']=='local'
        assert response['confirmed_facts']==[]
    asyncio.run(scenario())


def test_claude_dashboard_offline_stays_local():
    async def scenario():
        service=pac.DiagnosticService(LogEvidence(),api_key='',model=pac.DEFAULT_CLAUDE_MODEL,
                                      provider='claude')
        result=await service.ask('마지막 실패?')
        assert result['source']=='local' and result['control']['command_executed'] is False
    asyncio.run(scenario())
