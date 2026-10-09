import asyncio
import json
import time
from pathlib import Path
import uuid

from aiohttp.test_utils import TestServer,TestClient

import pac_diagnostic_console as pac
from pac_live_state import empty_sample, TelemetryStore


class NoLog:
    def snapshot(self):return pac.no_context()


def sample():
    stamp=time.time();d=empty_sample();d['collected_epoch_s']=stamp
    d['joint_states'].update(topic='/joint_states', observed_epoch_s=stamp,
                              joints=[{'name':'axis1','position':.123,'velocity':None}])
    d['gazebo'].update(topic='/world/pac/pose/info',observed_epoch_s=stamp,
                       entities=[{'name':'box_12','x':.35,'y':.2,'z':.91,'yaw':None}])
    d['cameras']['cctv'].update(topic='/pac/top_camera/image',observed_epoch_s=stamp)
    return d


def test_dashboard_and_api_use_only_live_observations(tmp_path):
    async def scenario():
        path=tmp_path/'live'/'telemetry.json';path.parent.mkdir()
        path.write_text(json.dumps(sample()))
        (path.parent/'camera_cctv.jpg').write_bytes(b'\xff\xd8fake-test-jpeg\xff\xd9')
        captured=[]
        async def fake(_http,_key,body):
            ctx=json.loads(body['input'][0]['content'])['context']
            captured.append(ctx)
            ids=[x['id'] for x in ctx['evidence'] if x['source'].startswith('gazebo:')]
            ans={'schema_version':'1.0','kind':'diagnosis','answer':'Gazebo 좌표는 확인됨. 실제 로봇 안전성은 확인 불가.',
                 'fact_ids':ids,'hypotheses':[], 'checks':[], 'actions':[], 'missing_information':['실제 힘 센서 없음']}
            return {'status':'completed','output':[{'type':'message','content':[{'type':'output_text','text':json.dumps(ans)}]}]}
        service=pac.DiagnosticService(NoLog(), 'sk-DUMMY-API-KEY',transport=fake,
                                      telemetry=TelemetryStore(path))
        async with TestServer(pac.make_app(service)) as server:
            async with TestClient(server) as client:
                s=await client.get('/api/state');v=await s.json()
                assert v['telemetry']['fresh']
                assert v['telemetry']['data']['gazebo']['entities'][0]['x']==.35
                assert v['telemetry']['data']['joint_states']['joints'][0]['position']==.123
                html=await client.get('/');h=await html.text()
                assert 'pose-canvas' in h and 'camera-cctv' in h and 'PyBullet 재현 없음' in h
                r=await client.get('/api/camera/cctv')
                assert r.status==200 and (await r.read())==b'\xff\xd8fake-test-jpeg\xff\xd9'
                r=await client.get('/api/camera/bad-name')
                assert r.status==404
                r=await client.post('/api/ask',json={'question':'Gazebo 상태 알려줘'})
                out=await r.json()
                pac.validate(out,pac.REPLY_SCHEMA)
                assert out['confirmed_facts'] and out['confirmed_facts'][0]['source'].startswith('gazebo:')
                assert len(captured)==1
                # Changing fresh telemetry to stale prevents JPEG serving and API evidence.
                stale=sample();stale['collected_epoch_s']-=360
                for x in ('joint_states','gazebo'):
                    stale[x]['observed_epoch_s']-=360
                stale['cameras']['cctv']['observed_epoch_s']-=360
                path.write_text(json.dumps(stale))
                await asyncio.sleep(2.2)
                r=await client.get('/api/state');v=await r.json()
                assert not v['telemetry']['fresh']
                r=await client.get('/api/camera/cctv')
                assert r.status==404
    asyncio.run(scenario())
