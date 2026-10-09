import os
import time
from pac_box_geometry import SpawnedBoxSizes, parse_spawned_box_sdf

SDF=b'''<sdf version="1.8"><model name="box_01"><link name="box_link"><visual name="visual">
<geometry><box><size>0.41 0.31 0.28</size></box></geometry>
</visual></link></model></sdf>'''


def test_spawned_box_parsing_dimensions_not_log_guesses():
    assert parse_spawned_box_sdf(SDF)==[.41,.31,.28]
    for invalid in (b'<sdf/>',b'<sdf><model><link><visual><geometry><box><size>3 -1 1</size></box></geometry></visual></link></model></sdf>'):
        try:parse_spawned_box_sdf(invalid)
        except ValueError:pass
        else:assert False,'unverified box accepted'


def test_box_sizes_only_from_new_files_not_stale_log(tmp_path):
    folder=tmp_path/'logs/v44_generator_cycle/SCENARIO_20001010_010100/boxes'
    folder.mkdir(parents=True)
    file=folder/'box_01.sdf';file.write_bytes(SDF)
    os.utime(file,(time.time()-3600,time.time()-3600))
    boxes=SpawnedBoxSizes(tmp_path,start_epoch=time.time()-10)
    boxes.refresh()
    assert not boxes.known
    file.touch()
    boxes.refresh()
    assert boxes.apply([{'name':'box_01','x':0,'y':0,'z':0,'yaw':0}])[0]['size']==[.41,.31,.28]
    assert boxes.apply([{'name':'box_01::box_link','x':0,'y':0,'z':0,'yaw':0}])[0].get('size') is None


def test_box_size_never_invented_without_file(tmp_path):
    boxes=SpawnedBoxSizes(tmp_path)
    poses=[{'name':'box_02','x':1,'y':2,'z':3,'yaw':0}]
    assert boxes.apply(poses)==poses and 'size' not in poses[0]
