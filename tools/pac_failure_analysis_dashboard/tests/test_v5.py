"""V5 regressions: data provenance, offline questions, bounded mesh access."""
from pathlib import Path
import json
import time
import asyncio

from aiohttp.test_utils import TestClient, TestServer
from pac_diagnostic_console import (DiagnosticService,local_diagnostic,no_context,
                                    validate,REPLY_SCHEMA,make_app)
from pac_live_state import empty_sample,TelemetryStore
from pac_robot3d import URDF,robot_mesh_file


def new_live():
    now=time.time()
    data=empty_sample()
    data['collected_epoch_s']=now
    data['joint_states'].update(topic='/joint_states',observed_epoch_s=now,
        joints=[{'name':f'j{i}','position':i/10,'velocity':0.0} for i in range(1,7)])
    data['gazebo'].update(topic='/world/mock/dynamic_pose/info',observed_epoch_s=now,
        entities=[{'name':'box_01','x':-1.,'y':1.2,'z':.8,'yaw':0.},
                  {'name':'hdr50_22','x':0.,'y':0.,'z':.4,'yaw':0.}])
    return data


def test_offline_question_uses_actual_recent_data(tmp_path):
    state=tmp_path/'telemetry.json';state.write_text(json.dumps(new_live()))
    live=TelemetryStore(state).snapshot()
    assert live['fresh']
    for question,term in [('현재 로봇 관절은?','j1='),('현재 박스는 어디에 있어?','box_01'),
                          ('수신 지연 얼마나 됐어?','명령 왕복')]:
        answer=local_diagnostic(question,no_context(),live,'sess')
        validate(answer,REPLY_SCHEMA)
        assert answer['status']=='ok' and term in answer['answer']
        assert answer['control']=={'mode':'read_only','command_executed':False}
    no_action=local_diagnostic('로봇 움직여',no_context(),live,'sess')
    assert no_action['status']=='refused' and not no_action['control']['command_executed']


def test_offline_failure_from_old_log_is_labeled_historical(tmp_path):
    context=no_context()
    context.update(run_id='previous_run',last_log_age_s=3000,observation='failure_logged')
    e={'id':'f123','source':'logs/previous_run/box.log','text':'MOVEIT PICK&PLACE FAIL: no descent',
       'file_modified_at':'2026-10-01T00:00:00Z'}
    context['alert']={k:e[k] for k in ('id','text','source')};context['evidence']=[e]
    r=local_diagnostic('왜 실패했어?',context,None,'sess')
    assert r['status']=='insufficient_data' and '이전 실행' in r['answer']
    assert r['confirmed_facts']==[e]
    assert not r['hypotheses']


def test_camera_count_demands_real_recent_jpeg_not_just_topic(tmp_path):
    state=tmp_path/'telemetry.json';data=new_live();now=time.time()
    data['cameras']['scale'].update(topic='/pac/scale_camera/image',observed_epoch_s=now)
    state.write_text(json.dumps(data))
    live=TelemetryStore(state).snapshot()
    assert live['camera_jpeg_ok']=={'scale':False,'cctv':False}
    r=local_diagnostic('카메라 연결됐어?',no_context(),live,'sess')
    assert '0/2' in r['answer'] and r['status']=='insufficient_data'
    # This is a JPEG file consistency check, not image-content authentication.
    (tmp_path/'camera_scale.jpg').write_bytes(b'\xff\xd8' + b'1'*30+b'\xff\xd9')
    live=TelemetryStore(state).snapshot()
    assert live['camera_jpeg_ok']['scale'] is True
    r=local_diagnostic('카메라 연결됐어?',no_context(),live,'sess')
    assert '1/2' in r['answer'] and r['status']=='insufficient_data'
    assert live['camera_jpeg_ok']['cctv'] is False


def test_old_joints_not_exposed_as_current(tmp_path):
    data=new_live();data['joint_states']['observed_epoch_s']=time.time()-30
    state=tmp_path/'telemetry.json';state.write_text(json.dumps(data))
    live=TelemetryStore(state).snapshot()
    response=local_diagnostic('로봇 관절 알려줘',no_context(),live,'sess')
    assert response['status']=='insufficient_data' and '실시간 관절값을 확인하지' in response['answer']


def test_safe_robot_stl_name_and_reference(tmp_path):
    source=tmp_path/URDF;source.parent.mkdir(parents=True)
    source.write_text('''<robot name="hdr_robot"><link name="base_link"><visual><geometry>
      <mesh filename="file://$(find hdr_description)/meshes/robots/hdr50_22/visual/arm.stl" scale="0.001 0.001 0.001"/>
      </geometry></visual></link></robot>''')
    visual=tmp_path/'ros2_ws/src/hdr_description/meshes/robots/hdr50_22/visual'
    visual.mkdir(parents=True)
    (visual/'arm.stl').write_bytes(b'0'*90)
    (visual/'secret.stl').write_bytes(b'0'*90)
    assert robot_mesh_file(tmp_path,'arm.stl')==(visual/'arm.stl').resolve()
    assert robot_mesh_file(tmp_path,'secret.stl') is None
    assert robot_mesh_file(tmp_path,'../secret.stl') is None
    assert robot_mesh_file(tmp_path,'/etc/passwd') is None


def test_offline_api_http_no_key_is_json(tmp_path):
    class Logs:
        repo=tmp_path
        def snapshot(self):return no_context()
    async def run():
        service=DiagnosticService(Logs(),api_key='')
        async with TestServer(make_app(service)) as server:
            async with TestClient(server) as client:
                rs=await client.post('/api/ask',json={'question':'로봇 정지시켜줘'})
                assert rs.status==200
                data=await rs.json()
                assert data['status']=='refused' and data['source']=='local'
                assert data['control']['command_executed'] is False
                assert (await client.get('/api/state')).status==200
                assert (await client.get('/assets/viewer3d_three.js')).status==200
                assert (await client.get('/assets/viewer3d_boot.js')).status==200
                assert (await client.get('/assets/robot-mesh/unknown.stl')).status==404
    asyncio.run(run())


def test_safety_query_does_not_convert_joint_feedback_into_stop_confirmation(tmp_path):
    state=tmp_path/'telemetry.json';state.write_text(json.dumps(new_live()))
    live=TelemetryStore(state).snapshot()
    result=local_diagnostic('로봇이 안전 정지했어?',no_context(),live,'sess')
    assert result['status']=='insufficient_data'
    assert '안전' in result['answer'] and '단정할 수 없습니다' in result['answer']
