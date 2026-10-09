import asyncio
import json
import pytest
import aiohttp
from aiohttp import web
from aiohttp.test_utils import TestServer

import pac_diagnostic_console as pac


@pytest.mark.parametrize('status,expected',[(401,'unavailable'),(429,'unavailable'),(500,'unavailable'),(302,'unavailable')])
def test_statuses_with_fake_http_transport(monkeypatch,status,expected):
    async def scenario():
        async def fake_provider(_):return web.Response(status=status,text='DO NOT EXPOSE API KEY')
        app=web.Application();app.router.add_post('/v1/responses',fake_provider)
        async with TestServer(app) as server:
            monkeypatch.setattr(pac,'API_URL',str(server.make_url('/v1/responses')))
            async with aiohttp.ClientSession() as session:
                with pytest.raises(pac.ProviderError) as exc:
                    await pac.openai_call(session,'sk-TEST-KEY',pac.request_body('gpt-4o-mini','안녕',pac.no_context(),[]))
                assert exc.value.status==expected
                assert 'KEY' not in str(exc.value)
    asyncio.run(scenario())


def test_timeout_real_http_path(monkeypatch):
    async def scenario():
        async def fake_provider(_):
            await asyncio.sleep(.2)
            return web.Response(text='{}')
        app=web.Application();app.router.add_post('/v1/responses',fake_provider)
        async with TestServer(app) as server:
            monkeypatch.setattr(pac,'API_URL',str(server.make_url('/v1/responses')))
            orig=aiohttp.ClientTimeout
            monkeypatch.setattr(aiohttp,'ClientTimeout',lambda total:orig(total=.03))
            async with aiohttp.ClientSession() as session:
                with pytest.raises(pac.ProviderError) as exc:
                    await pac.openai_call(session,'sk-TEST-KEY',{})
                assert exc.value.status=='unavailable'
    asyncio.run(scenario())


def test_api_strict_model_payload():
    b=pac.request_body('gpt-4o-mini','왜 실패?',pac.no_context(),[])
    assert b['store'] is False
    assert b['text']['format']['type']=='json_schema'
    assert b['text']['format']['strict'] is True
    assert b['text']['format']['schema']==pac.MODEL_SCHEMA
    assert b['input'][0]['role']=='user'
