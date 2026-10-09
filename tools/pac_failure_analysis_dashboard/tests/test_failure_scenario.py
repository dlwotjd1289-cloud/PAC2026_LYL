"""End-to-end failure LOG→UI→offline answer regression tests; no robot motion."""
import asyncio
import json
import time
from pathlib import Path

from aiohttp.test_utils import TestServer, TestClient

from pac_diagnostic_console import DiagnosticService,LogEvidence,make_app,validate,REPLY_SCHEMA,failure_stage
from scenario_fail_ik import create_record,main, TARGET, GROUP, TIP


def test_synthetic_injection_isolated_and_explicit(tmp_path):
    repo = tmp_path/'repo'; (repo/'logs'/'v44_generator_cycle'/'OLD_RUN').mkdir(parents=True)
    (repo/'logs'/'v44_generator_cycle'/'OLD_RUN'/'box_01_moveit.log').write_text(
        'REAL OLD MOVEIT PICK&PLACE FAIL\n')
    run, manifest = create_record(repo, mode='sample',code=-31,code_name='NO_IK_SOLUTION')
    assert manifest['is_test'] and not manifest['is_physical_failure']
    assert not manifest['robot_command_sent'] and not manifest['gazebo_modified']
    assert manifest['request']['target_xyz_m']==TARGET
    assert manifest['request']['group']==GROUP and manifest['request']['ik_link_name']==TIP
    assert list((repo/'logs'/'v44_generator_cycle'/'OLD_RUN').iterdir()) == [
        repo/'logs'/'v44_generator_cycle'/'OLD_RUN'/'box_01_moveit.log']
    ctx=LogEvidence(repo,run).snapshot()
    assert ctx['run_id'].startswith('TEST_IK_SAMPLE_')
    assert ctx['observation']=='failure_logged'
    assert ctx['alert']['source'].startswith('logs/pac_dashboard_test_scenarios/')
    assert 'synthetic response' in ctx['alert']['text']
    assert failure_stage(ctx)['stage']=='moveit_ik'


def test_no_false_fail_for_camera_pose_error_metric(tmp_path):
    repo=tmp_path/'repo'; run=repo/'logs'/'pac_dashboard_test_scenarios'/'TEST_CAMERA_METRIC'
    run.mkdir(parents=True)
    (run/'camera_moveit.log').write_text('''[camera] far CCTV coarse pose error vs ground truth 1.7 mm
[INFO] camera pose error: 0.0015 m
''')
    snap=LogEvidence(repo,run).snapshot()
    assert snap['observation']!='failure_logged' and not snap['alert']['id']


def test_real_negative_result_manifest_label_does_not_claim_motion(tmp_path):
    repo=tmp_path/'repo';repo.mkdir()
    run,record=create_record(repo,mode='moveit-ik',code=-31,code_name='NO_IK_SOLUTION',elapsed_s=0.321)
    assert record['endpoint']=='/compute_ik' and record['response']['duration_s']==0.321
    assert not record['robot_command_sent'] and not record['is_physical_failure']
    assert 'actual GetPositionIK response' in (run/'box_TEST_moveit.log').read_text()


def test_fail_scenario_rejected_when_repo_missing_or_unsafe_symlink(tmp_path):
    import pytest
    with pytest.raises(ValueError): create_record(tmp_path/'nope',mode='sample',code=-31,code_name='NO_IK_SOLUTION')
    repo=tmp_path/'repo';repo.mkdir();(repo/'logs').symlink_to(tmp_path)
    with pytest.raises(ValueError):create_record(repo,mode='sample',code=-31,code_name='NO_IK_SOLUTION')


def test_http_full_chain_and_fixed_reply_schema(tmp_path):
    repo=tmp_path/'repo';repo.mkdir()
    run,_=create_record(repo,mode='sample',code=-31,code_name='NO_IK_SOLUTION')
    evidence=LogEvidence(repo,run)
    async def exercise():
        service=DiagnosticService(evidence,api_key='')
        async with TestServer(make_app(service)) as server:
            async with TestClient(server) as client:
                # initial request and asynchronous loop refresh: >=0.4 sec
                await asyncio.sleep(0.5)
                resp=await client.get('/api/state')
                assert resp.status==200
                state=await resp.json()
                assert state['context']['run_id']==run.name
                assert state['context']['observation']=='failure_logged'
                assert state['failure_stage']['stage']=='moveit_ik'
                assert state['context']['alert']['source'].startswith('logs/pac_dashboard_test_scenarios/')
                html=await (await client.get('/')).text()
                assert 'test-banner' in html and '의도적 실패 테스트' in html
                rr=await client.post('/api/ask',json={'question':'마지막 실패 원인은 뭐야?'})
                assert rr.status==200
                data=await rr.json()
                validate(data,REPLY_SCHEMA)
                assert data['source']=='local' and data['status']=='insufficient_data'
                assert '합성 로그 테스트' in data['answer']
                assert '실제 박스 적재 실패' in data['answer']
                assert 'NO_IK_SOLUTION' in data['answer']
                assert data['control']=={'mode':'read_only','command_executed':False}
                assert len(data['confirmed_facts'])==1
                assert data['confirmed_facts'][0]['id']==state['context']['alert']['id']
                rr=await client.post('/api/ask',json={'question':'로봇 움직여'})
                assert (await rr.json())['status']=='refused'
    asyncio.run(exercise())


def test_command_line_sample_generates_test_record(tmp_path,capsys):
    repo=tmp_path/'repo';repo.mkdir()
    assert main(['--repo',str(repo),'--mode','sample'])==0
    msg=capsys.readouterr().out
    assert '"mode": "sample"' in msg
    root=repo/'logs'/'pac_dashboard_test_scenarios'
    dirs=list(root.iterdir())
    assert len(dirs)==1 and (dirs[0]/'test_manifest.json').is_file()
