from pathlib import Path
from types import SimpleNamespace
import tempfile
import time
import threading

from pac_gazebo_stream import PoseTextDecoder,usable_pose_topic
from pac_live_state import parse_ign_pose_text, empty_sample
from pac_robot3d import robot_for_repo, HOME_SRDF, URDF
from pac_ros_observer import prefer_top_level,ReadOnlyObserver


def pose(name, x):
    return f'pose {{\n  name: "{name}"\n  position {{\n    x: {x}\n    y: 1.2\n    z: 0.9\n  }}\n  orientation {{\n    w: 1\n  }}\n}}\n'


def test_stream_named_poses_separators_and_partial_chunks():
    dec=PoseTextDecoder()
    data=pose('box_01',-4.0)+'---\n'+pose('box_02',-3.0)+'---\n'
    result=[]
    for i in range(0,len(data),23):result+=dec.feed(data[i:i+23])
    assert len(result)==2
    assert [parse_ign_pose_text(t)[0]['name'] for t in result]==['box_01','box_02']


def test_stream_accepts_header_frame_boundary_without_separator():
    dec=PoseTextDecoder()
    msg='header {\n  stamp {\n    sec: 1\n  }\n}\n'+pose('box_01',1)
    ready=dec.feed(msg+'header {\n  stamp {\n    sec: 2\n  }\n}\n'+pose('box_02',2))
    assert len(ready)==1 and 'box_01' in ready[0]
    dec.last_input-=2
    assert 'box_02' in dec.flush_idle()


def test_stream_rejects_unfinished_message():
    dec=PoseTextDecoder()
    dec.feed('pose {\nname: "box_12"\nposition {\nx: 1\n')
    dec.last_input-=2
    assert dec.flush_idle() is None


def test_stream_auto_topic_discovery():
    assert usable_pose_topic(['/world/x/pose/info','/world/x/dynamic_pose/info'])=='/world/x/dynamic_pose/info'
    assert usable_pose_topic(['/world/x/pose/info'])=='/world/x/pose/info'
    assert usable_pose_topic(['/world/x/pose/info','/world/y/pose/info'])==''
    assert usable_pose_topic([], '/world/y/pose/info')=='/world/y/pose/info'


def test_prefer_top_level_boxes_over_links():
    def e(n):return {'name':n,'x':0,'y':0,'z':0,'yaw':0}
    value=[e('robot::link'+str(i)) for i in range(500)]+[e('box_12'),e('pallet_main')]
    out=prefer_top_level(value)
    assert [x['name'] for x in out]==['box_12','pallet_main']


def test_pose_v_bridge_empty_frames_do_not_block_native_gazebo_stream():
    # No ROS available in testing environment: exercise sidecar method using inert mocks.
    class Logger:
        def __init__(self):self.warnings=[]
        def warning(self,msg):self.warnings.append(msg)
    logger=Logger()
    o=ReadOnlyObserver.__new__(ReadOnlyObserver)
    o.lock=threading.Lock();o.state=empty_sample()
    o.args=SimpleNamespace(entity_pattern='box|hdr50')
    o.box_sizes=SimpleNamespace(apply=lambda xs:xs)
    o.ros_pose_last=0.0;o.gazebo_named_warning=False
    o.node=SimpleNamespace(get_logger=lambda:logger)
    msg=SimpleNamespace(transforms=[SimpleNamespace(child_frame_id='',transform=SimpleNamespace(
        translation=SimpleNamespace(x=1.,y=2.,z=3.),
        rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))])
    o.on_gazebo_tf(msg)
    assert o.ros_pose_last==0.0
    assert o.state['gazebo']['observed_epoch_s'] is None
    assert logger.warnings


def test_srdf_reference_is_never_claimed_as_observation(tmp_path):
    srdf=tmp_path/HOME_SRDF;srdf.parent.mkdir(parents=True)
    srdf.write_text('<robot name="hdr_robot"><group_state name="home" group="robot">'+
       ''.join(f'<joint name="j{i}" value="{i/10}"/>' for i in range(1,7))+
       '</group_state></robot>')
    urdf=tmp_path/URDF;urdf.parent.mkdir(parents=True,exist_ok=True)
    urdf.write_text('<robot name="hdr_robot"><link name="world"/><link name="base_link"/>'+
       '<joint name="world_joint" type="fixed"><parent link="world"/><child link="base_link"/>'+
       '<origin xyz="0 0 .4" rpy="0 0 0"/></joint></robot>')
    result=robot_for_repo(tmp_path)
    assert len(result['reference_joints'])==6
    assert result['reference_joints'][0]['position']==.1
    assert result['joints'][0]['name']=='world_joint'
    assert 'observed_epoch_s' not in result
