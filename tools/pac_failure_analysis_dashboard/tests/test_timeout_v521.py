import asyncio
import pathlib
import time

import aiohttp
import pytest
from aiohttp import web
from aiohttp.test_utils import TestServer

import pac_diagnostic_console as pac


def test_timeout_setting_and_browser_limit():
    import inspect
    src=inspect.getsource(pac.claude_call)
    assert pac.CLAUDE_REQUEST_TIMEOUT_S == 300
    assert 'timeout=CLAUDE_REQUEST_TIMEOUT_S' in src
    assert 'ClientTimeout(total=45)' not in src
    page=(pathlib.Path(pac.__file__).parent/'web'/'dashboard.html').read_text()
    assert 'controller.abort(),315000' in page
    assert 'Claude 사용 시 최대 300초(5분)' in page
    assert 'timeout=CLAUDE_REQUEST_TIMEOUT_S + 30' in pathlib.Path(pac.__file__).read_text()


def test_network_error_is_not_called_a_timeout():
    class FailingSession:
        def post(self, *args, **kwargs):
            raise aiohttp.ClientConnectorError(None, OSError('disconnected'))
    async def scenario():
        with pytest.raises(pac.ProviderError) as x:
            await pac.claude_call(FailingSession(), 'private-key', {})
        assert x.value.status=='unavailable'
        assert '네트워크/SSL/연결 오류' in str(x.value)
        assert 'private-key' not in str(x.value)
        assert '응답 시간 초과' not in str(x.value)
    asyncio.run(scenario())


def test_real_timeout_is_labelled_distinctly(monkeypatch):
    orig=aiohttp.ClientTimeout
    monkeypatch.setattr(aiohttp,'ClientTimeout',lambda total:orig(total=0.04))
    async def scenario():
        async def slow(_):
            await asyncio.sleep(0.35)
            return web.json_response({})
        app=web.Application();app.router.add_post('/v1/messages',slow)
        async with TestServer(app) as server:
            monkeypatch.setattr(pac,'ANTHROPIC_API_URL',str(server.make_url('/v1/messages')))
            async with aiohttp.ClientSession() as s:
                with pytest.raises(pac.ProviderError) as x:
                    await pac.claude_call(s, 'private-key', {})
                assert x.value.status=='unavailable'
                assert '응답 시간 초과' in str(x.value)
                assert '서버 제한 300초' in str(x.value)
                assert 'private-key' not in str(x.value)
    asyncio.run(scenario())
