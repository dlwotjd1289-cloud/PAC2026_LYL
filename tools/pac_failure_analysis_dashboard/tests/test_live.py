import json
import os
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from pac_live_state import (TelemetryStore,BadTelemetry,empty_sample,validate_live,
                            parse_ign_pose_text,add_telemetry_evidence)
from pac_ros_observer import choose_topic, atomic_write, image_to_jpeg
from pac_diagnostic_console import no_context, CONTEXT_SCHEMA,validate


SAMPLE='''header { stamp { sec: 10 } }
pose {
  id: 5
  name: "box_007"
  position { x: -0.831 y: 0.60 z: 1.019 }
  orientation { x: 0 y: 0 z: 0.7071068 w: 0.7071068 }
}
pose {
  id: 8
  name: "pallet"
  position { x: 2 y: 0 z: 0.15 }
}
pose {
  id: 12
  name: "conveyor_motor"
  position { x: -2 y: 2 z: 2 }
}
'''


def test_gazebo_real_pose_text_parser():
    items=parse_ign_pose_text(SAMPLE)
    assert len(items)==2
    assert items[0]['name']=='box_007'
    assert abs(items[0]['x']+.831)<1e-8
    assert abs(items[0]['yaw']-1.57079632679)<1e-5
    assert items[1]['name']=='pallet'
    assert items[1]['yaw'] is None
    assert parse_ign_pose_text('nothing')==[]
    assert parse_ign_pose_text(SAMPLE,'nomatch')==[]


def test_discovery_no_guessed_topic():
    graph={'/robot/joint_states':['sensor_msgs/msg/JointState'],
           '/mycam/scale':['sensor_msgs/msg/Image']}
    assert choose_topic(graph,'','sensor_msgs/msg/JointState',['/joint_states','/robot/joint_states'])=='/robot/joint_states'
    assert choose_topic(graph,'','sensor_msgs/msg/Image',['/pac/scale_camera/image']) is None
    with pytest.raises(ValueError):choose_topic(graph,'/not-here','sensor_msgs/msg/Image',[])


def test_image_conversion_is_sensor_msg_data_not_synthetic():
    import numpy as np
    msg=SimpleNamespace(encoding='rgb8',height=60,width=120,step=360,
                        data=np.full((60,120,3),[10,40,80],dtype=np.uint8).tobytes())
    jpg=image_to_jpeg(msg)
    assert jpg[:2]==b'\xff\xd8' and jpg[-2:]==b'\xff\xd9'
    msg.encoding='16UC1'
    with pytest.raises(ValueError):image_to_jpeg(msg)


def sample(now=None):
    now=now or time.time()
    s=empty_sample();s['collected_epoch_s']=now
    s['joint_states'].update(topic='/joint_states',observed_epoch_s=now,
        joints=[{'name':'joint1','position':.5,'velocity':0.05}])
    s['gazebo'].update(topic='/world/current/pose/info',observed_epoch_s=now,
        entities=[{'name':'box_07','x':1.5,'y':-2.0,'z':.8,'yaw':0.1}])
    s['cameras']['scale'].update(topic='/pac/scale_camera/image',observed_epoch_s=now)
    s['controller'].update(topic='/arm_controller/state',observed_epoch_s=now,
                            max_abs_position_error=0.003)
    return s


def test_store_read_and_fresh_evidence(tmp_path):
    path=tmp_path/'telemetry.json';path.write_text(json.dumps(sample()))
    store=TelemetryStore(path)
    live=store.snapshot()
    assert live['fresh'] and live['available']
    assert live['source_age_s']['joint_states']<=2
    context=add_telemetry_evidence(no_context(),live)
    validate(context,CONTEXT_SCHEMA)
    assert len(context['evidence'])==3
    assert all(item['source'].startswith(('ros2:','gazebo:')) for item in context['evidence'])
    assert context['stop_confirmed'] is None and context['suction']=='unknown'


def test_stale_snapshot_not_evidence(tmp_path):
    path=tmp_path/'telemetry.json';path.write_text(json.dumps(sample(time.time()-60)))
    store=TelemetryStore(path)
    live=store.snapshot()
    assert live['available'] and not live['fresh']
    assert add_telemetry_evidence(no_context(),live)['evidence']==[]


def test_empty_present_but_no_live_signal_not_connected(tmp_path):
    path=tmp_path/'telemetry.json';s=empty_sample();s['collected_epoch_s']=time.time()
    path.write_text(json.dumps(s))
    live=TelemetryStore(path).snapshot()
    assert live['available'] and not live['fresh']


def test_reject_unexpected_data_bad_types_and_secret(tmp_path):
    path=tmp_path/'telemetry.json'
    for replacement in (lambda s:s.update(synthetic=True),
                        lambda s:s['joint_states']['joints'][0].update(position=float('nan')),
                        lambda s:s['gazebo']['entities'][0].update(x='1.2')):
        s=sample();replacement(s)
        path.write_text(json.dumps(s))
        assert TelemetryStore(path).snapshot()['available']==False
    s=sample();s['suction'].update(topic='/pac/suction/state',value='password=abc000')
    path.write_text(json.dumps(s))
    live=TelemetryStore(path).snapshot()
    assert live['available']
    assert 'abc000' not in str(live['data'])


def test_symlink_rejected(tmp_path):
    raw=tmp_path/'raw.json';raw.write_text(json.dumps(sample()))
    path=tmp_path/'telemetry.json';path.symlink_to(raw)
    assert TelemetryStore(path).snapshot()['available'] is False


def test_camera_and_state_filename_are_fixed(tmp_path):
    p=tmp_path/'sub'/'camera_cctv.jpg'
    atomic_write(p,b'bytes')
    assert p.read_bytes()==b'bytes'
    assert not p.with_name(p.name+'.tmp').exists()
