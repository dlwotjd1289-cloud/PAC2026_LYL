"""Local HTTP transport tests; no real provider or credentials are used."""
import asyncio
import ssl
import time

import aiohttp
import pytest
from aiohttp import web
from aiohttp.test_utils import TestServer

import pac_diagnostic_console as pac


async def truncated(request):
    response = web.StreamResponse(headers={'Content-Length': '1000'})
    await response.prepare(request)
    await response.write(b'{"incomplete":')
    request.transport.close()
    return response


@pytest.mark.parametrize('mode', ['recover', 'repeat', 'auth', 'budget'])
def test_receive_retry_local_http(monkeypatch, mode):
    async def scenario():
        calls = []

        async def handler(request):
            calls.append(time.monotonic())
            assert request.headers['Accept-Encoding'] == 'identity'
            assert (await request.json())['stream'] is False
            if mode == 'auth':
                return web.Response(status=401, text='private-test-key')
            if mode == 'budget':
                await asyncio.sleep(0.09)
            if len(calls) == 1 or mode == 'repeat':
                return await truncated(request)
            # Deliver JSON over separate packets to cover complete-body assembly.
            response = web.StreamResponse(headers={'Content-Type': 'application/json'})
            await response.prepare(request)
            await response.write(b'{"ok":')
            await asyncio.sleep(0.005)
            await response.write(b'true}')
            await response.write_eof()
            return response

        app = web.Application()
        app.router.add_post('/v1/messages', handler)
        if mode == 'budget':
            monkeypatch.setattr(pac, 'CLAUDE_REQUEST_TIMEOUT_S', 0.14)
        async with TestServer(app) as server:
            monkeypatch.setattr(pac, 'ANTHROPIC_API_URL', str(server.make_url('/v1/messages')))
            async with aiohttp.ClientSession() as session:
                started = time.monotonic()
                if mode == 'recover':
                    assert await pac.claude_call(session, 'private-test-key', {}) == {'ok': True}
                else:
                    with pytest.raises(pac.ProviderError) as exc:
                        await pac.claude_call(session, 'private-test-key', {})
                    assert exc.value.status == 'unavailable'
                    assert 'private-test-key' not in str(exc.value)
                    if mode == 'repeat':
                        assert '1회 재시도' in str(exc.value)
                    elif mode == 'auth':
                        assert '키 인증' in str(exc.value)
                    else:
                        assert '응답 시간 초과' in str(exc.value)
                        assert time.monotonic() - started < 0.18
        assert len(calls) == (1 if mode == 'auth' else 2)

    asyncio.run(scenario())


def test_tls_verification_failure_is_not_retried():
    class FailedTLS:
        calls = 0

        def post(self, *args, **kwargs):
            self.calls += 1
            assert kwargs.get('ssl') is not False
            raise aiohttp.ClientConnectorCertificateError(None, ssl.CertificateError('test cert'))

    async def scenario():
        session = FailedTLS()
        with pytest.raises(pac.ProviderError) as exc:
            await pac.claude_call(session, 'private-test-key', {})
        assert session.calls == 1
        assert '네트워크/SSL/연결 오류' in str(exc.value)
        assert 'private-test-key' not in str(exc.value)

    asyncio.run(scenario())
