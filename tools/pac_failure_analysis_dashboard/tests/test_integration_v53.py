import json
import math
from pathlib import Path
import time
import pytest

from pac_integration import (DEFAULT_MANIFEST,BindingError,load_binding,load_catalog,
    load_events,locate_box,summarize_run,add_zone_evidence)
from pac_gazebo_stream import choose_runtime_pose_topic
from pac_box_geometry import SpawnedBoxSizes
from pac_live_state import empty_sample,validate_live
from pac_scene3d import scene_for_repo
from pac_diagnostic_console import LogEvidence,no_context,CONTEXT_SCHEMA,validate
from pac_camera_bridge_watch import desired_cameras
from pac_run_contract_writer import register_active_run,report_event


@pytest.fixture
def bound_run(tmp_path):
    repo=tmp_path
    (repo/'logs'/'run_alpha'/'boxes').mkdir(parents=True)
    (repo/'ros2_ws'/'worlds').mkdir(parents=True)
    world=repo/'ros2_ws'/'worlds'/'demo.sdf'
    world.write_text('''<sdf version="1.8"><world name="workcell_alpha"><model name="floor"><static>true</static><link name="l"><visual name="v"><geometry><box><size>2 2 .1</size></box></geometry></visual></link></model></world></sdf>''')
    (repo/'logs'/'run_alpha'/'boxes.json').write_text(json.dumps({'box_01':{'size_m':[.3,.2,.15]},'sku_502':{'size_m':[.4,.3,.2]}}))
    binding=register_active_run(repo,run_id='TEST_ALPHA',world='workcell_alpha',
        world_sdf='ros2_ws/worlds/demo.sdf',log_dir='logs/run_alpha',
        box_catalog='logs/run_alpha/boxes.json',events_file='logs/run_alpha/events.jsonl',
        topics={'gazebo_pose':'/world/workcell_alpha/dynamic_pose/info',
                'joint_states':'/hdr/joint_states'},
        zones={'pallet':{'x':[-1,0],'y':[-1,1],'z':[0,2]},
               'buffer':{'x':[0,1],'y':[-1,1],'z':[0,2]},
               'conveyor':{'x':[2,5],'y':[-1,1],'z':[0,2]}})
    return repo,binding


def sample(x=-.5,y=0,z=.5,topic='/world/workcell_alpha/dynamic_pose/info'):
    d=empty_sample();now=time.time()
    d['collected_epoch_s']=now
    d['gazebo'].update(topic=topic,observed_epoch_s=now,entities=[
        {'name':'box_01','x':x,'y':y,'z':z,'yaw':0,'size':[.3,.2,.15],'size_source':'run_catalog'}])
    validate_live(d)
    return {'available':True,'fresh':True,'age_s':0.1,
            'source_age_s':{'gazebo':0.1},'data':d}


def test_no_run_manifest_is_not_assumed(tmp_path):
    (tmp_path/'logs').mkdir()
    assert load_binding(tmp_path) is None
    result=summarize_run(None,None)
    assert result['status']=='not_bound' and result['boxes']==[]


def test_world_pose_explicit_or_single_and_ambiguous():
    t=['/world/a/pose/info','/world/a/dynamic_pose/info','/world/b/dynamic_pose/info']
    assert choose_runtime_pose_topic(t)==''
    assert choose_runtime_pose_topic(t,world='a')=='/world/a/dynamic_pose/info'
    assert choose_runtime_pose_topic(t,world='b')=='/world/b/dynamic_pose/info'
    assert choose_runtime_pose_topic(t,world='b',requested='/world/a/dynamic_pose/info')==''
    assert choose_runtime_pose_topic([],world='a')==''
    assert choose_runtime_pose_topic(['/world/a/pose/info'])=='/world/a/pose/info'


def test_manifest_roundtrip_and_late_catalog(bound_run):
    root,run=bound_run
    assert load_binding(root)['run_id']=='TEST_ALPHA'
    assert load_catalog(run,root)['box_01']==[.3,.2,.15]
    box=SpawnedBoxSizes(root,manifest=root/DEFAULT_MANIFEST,start_epoch=time.time()+1000)
    box.refresh()
    poses=[{'name':'box_01','x':0,'y':0,'z':.3,'yaw':0},
           {'name':'sku_502','x':0,'y':0,'z':.3,'yaw':0}]
    box.apply(poses)
    assert poses[0]['size_source']=='run_catalog'
    assert poses[1]['size']==[.4,.3,.2]


def test_same_run_events_and_zone_agreement(bound_run):
    root,bound=bound_run
    report_event(root,'box_01','pallet_placed',success=True,stamp=time.time()-1)
    output=summarize_run(bound,sample(),root)
    assert output['status']=='live'
    assert output['zones']['pallet']['observed']==1
    assert output['zones']['pallet']['executor_reported']==1
    assert output['boxes'][0]['event_matches_location'] is True
    assert output['boxes'][0]['confidence']=='observed_pose_and_executor_report'
    assert 'sku_502' in output['unobserved_catalog_boxes']


def test_event_claim_does_not_confirm_without_pose(bound_run):
    root,bound=bound_run
    report_event(root,'box_01','pallet_placed',success=True)
    output=summarize_run(bound,None,root)
    assert output['status']=='no_live_poses' and output['boxes']==[]


def test_event_discrepancy_detected(bound_run):
    root,bound=bound_run
    report_event(root,'box_01','pallet_placed',success=True)
    output=summarize_run(bound,sample(.5),root)
    assert output['zones']['buffer']['observed']==1
    assert output['zones']['buffer']['discrepancy']==1
    assert output['boxes'][0]['event_matches_location'] is False


def test_position_only_not_completion(bound_run):
    root,bound=bound_run
    output=summarize_run(bound,sample(),root)
    assert output['boxes'][0]['process_status']=='unconfirmed'
    assert output['zones']['pallet']['executor_reported']==0


def test_ambiguous_overlap_reports_unknown():
    zones={'pallet':{'x':[0,2],'y':[0,2]},'buffer':{'x':[1,3],'y':[0,2]}}
    assert locate_box({'x':1.5,'y':1,'z':.2},zones)['zone']=='ambiguous'
    assert locate_box({'x':9,'y':1,'z':.2},zones)['zone']=='unknown'


def test_mismatch_world_blocks_all_boxes(bound_run):
    root,bound=bound_run
    output=summarize_run(bound,sample(topic='/world/other/dynamic_pose/info'),root)
    assert output['status']=='world_mismatch' and output['boxes']==[]


def test_manifest_world_topic_mismatch_rejected(bound_run):
    root,_=bound_run
    path=root/DEFAULT_MANIFEST
    data=json.loads(path.read_text());data['topics']['gazebo_pose']='/world/other/dynamic_pose/info'
    path.write_text(json.dumps(data))
    with pytest.raises(BindingError):load_binding(root)


def test_manifest_bad_path_rejected_without_overwriting_old(bound_run):
    root,run=bound_run
    with pytest.raises(BindingError):
        register_active_run(root,run_id='INTRUDER',world='workcell_alpha',
            world_sdf='../../tmp/injected.sdf',log_dir='logs/run_alpha')
    assert load_binding(root)['run_id']=='TEST_ALPHA'


def test_manifest_duplicate_keys_rejected(bound_run):
    root,_=bound_run
    (root/DEFAULT_MANIFEST).write_text('{"schema_version":"1.0","run_id":"x","run_id":"y","world":"a"}')
    with pytest.raises(BindingError):load_binding(root)


def test_manifest_symlink_rejected(bound_run):
    root,_=bound_run
    manifest=root/DEFAULT_MANIFEST
    manifest.unlink();manifest.symlink_to(root/'logs/run_alpha/boxes.json')
    with pytest.raises(BindingError):load_binding(root)


def test_log_selection_follows_active_manifest(bound_run):
    root,bound=bound_run
    (root/'logs/run_alpha/robot_moveit.log').write_text('MOVEIT PICK&PLACE FAIL: IK failed\n')
    old=root/'logs/v44_generator_cycle/OLD';old.mkdir(parents=True)
    (old/'old_moveit.log').write_text('ERROR: previous run\n')
    evidence=LogEvidence(root).snapshot()
    assert evidence['run_id']=='TEST_ALPHA'
    assert all('OLD' not in e['source'] for e in evidence['evidence'])
    assert evidence['observation']=='failure_logged'


def test_run_switch_clears_log_identity(bound_run):
    root,bound=bound_run
    other=root/'logs/run_beta';other.mkdir()
    register_active_run(root,run_id='TEST_BETA',world='workcell_alpha',
        world_sdf='ros2_ws/worlds/demo.sdf',log_dir='logs/run_beta')
    evidence=LogEvidence(root).snapshot()
    assert evidence['run_id']=='TEST_BETA' and evidence['evidence']==[]


def test_world_sdf_wrong_name_not_rendered(bound_run):
    root,bound=bound_run
    result=scene_for_repo(root,bound['files']['world_sdf'],'workcell_other')
    assert result['primitives']==[] and result['static_source']=='world_mismatch'
    good=scene_for_repo(root,bound['files']['world_sdf'],'workcell_alpha')
    assert good['world']=='workcell_alpha' and len(good['primitives'])==1


def test_camera_topics_from_manifest(bound_run):
    _,bound=bound_run
    assert desired_cameras(bound)=={'scale_camera':'/pac/scale_camera/image','cctv_camera':'/pac/top_camera/image'}
    bound['topics']['cctv_camera']='/custom/image'
    assert desired_cameras(bound)['cctv_camera']=='/custom/image'


def test_event_from_other_run_ignored(bound_run):
    root,bound=bound_run
    (root/'logs/run_alpha/events.jsonl').write_text(json.dumps({'run_id':'OLD','world':'workcell_alpha',
        'box_id':'box_01','event':'pallet_placed','success':True,'timestamp_epoch_s':time.time()})+'\n')
    assert not load_events(bound)


def test_evidence_requires_matching_run(bound_run):
    root,bound=bound_run
    summary=summarize_run(bound,sample(),root)
    other=no_context();other['run_id']='DIFFERENT'
    assert add_zone_evidence(other,summary)['evidence']==[]
    current=no_context();current['run_id']='TEST_ALPHA'
    enriched=add_zone_evidence(current,summary)
    validate(enriched,CONTEXT_SCHEMA)
    assert len(enriched['evidence'])==1
    assert '물리적 완료' in enriched['evidence'][0]['text']


def test_http_active_run_zone_state(bound_run):
    import asyncio
    import pac_diagnostic_console as pac
    from pac_live_state import TelemetryStore
    from aiohttp.test_utils import TestClient,TestServer

    async def scenario():
        root,bound=bound_run
        report_event(root,'box_01','pallet_placed',success=True)
        path=root/'logs/pac_dashboard_integrated_v53/telemetry.json'
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(sample()['data']))
        svc=pac.DiagnosticService(pac.LogEvidence(root),provider='claude',telemetry=TelemetryStore(path))
        async with TestServer(pac.make_app(svc)) as server:
            async with TestClient(server) as client:
                # The background supervisor recomputes the summary independently.
                await asyncio.sleep(.65)
                r=await client.get('/api/state');assert r.status==200
                state=await r.json()
                assert state['integration']['run_id']=='TEST_ALPHA'
                assert state['integration']['zones']['pallet']['observed']==1
                assert state['integration']['zones']['pallet']['executor_reported']==1
                r=await client.get('/api/integration');assert r.status==200
                integ=await r.json()
                assert integ['boxes'][0]['process_status']=='pallet_placed'
                assert 'evidence' in state['context']
                assert any('통합 실행' in e['text'] for e in state['context']['evidence'])
    asyncio.run(scenario())


def test_no_camera_stale_count_in_live_snapshot(bound_run):
    root,_=bound_run
    from pac_live_state import TelemetryStore
    path=root/'logs/pac_dashboard_integrated_v53/telemetry.json'
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(sample()['data']))
    store=TelemetryStore(path)
    snap=store.snapshot()
    assert snap['fresh']
    assert snap['camera_jpeg_ok']=={'scale':False,'cctv':False}


def test_reported_unobserved_not_counted_as_pallet(bound_run):
    root,bound=bound_run
    report_event(root,'sku_502','pallet_placed',success=True)
    view=summarize_run(bound,sample(),root)
    assert 'sku_502' in view['reported_not_observed']
    assert view['zones']['pallet']['observed']==1
    assert view['zones']['pallet']['executor_reported']==0


def test_manifest_world_sdf_from_logs_allowed(bound_run):
    root,_=bound_run
    generated=root/'logs/run_alpha/generated_world.sdf'
    generated.write_text((root/'ros2_ws/worlds/demo.sdf').read_text())
    out=register_active_run(root,run_id='TEST_LOG_WORLD',world='workcell_alpha',
        world_sdf='logs/run_alpha/generated_world.sdf',log_dir='logs/run_alpha')
    assert out['files']['world_sdf']==generated


def test_offline_zone_question_only_with_current_bound_data(bound_run):
    from pac_diagnostic_console import local_diagnostic
    root,bound=bound_run
    view=summarize_run(bound,sample(),root)
    c=no_context();c['run_id']='TEST_ALPHA';c=add_zone_evidence(c,view)
    response=local_diagnostic('팔레트랑 버퍼 박스 몇 개?',c,sample(),'00000000-0000-0000-0000-000000000000',view)
    assert response['status']=='ok'
    assert '팔레트: 위치 관측 1개' in response['answer']
    assert response['confirmed_facts'] and response['confirmed_facts'][0]['source'].startswith('gazebo:')
    assert response['control']['command_executed'] is False


def test_offline_zone_question_no_manifest_no_counts():
    from pac_diagnostic_console import local_diagnostic
    response=local_diagnostic('팔레트에 몇 개?',no_context(),None,'',None)
    assert response['status']=='insufficient_data'
    assert '실행별 구역' in response['answer']
