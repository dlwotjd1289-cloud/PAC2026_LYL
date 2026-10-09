# PAC2026 대시보드 V5.3 — 최종 알고리즘 통합 준비

## 범위와 검증 상태

이 버전은 기존 Gazebo, MoveIt, 적재 알고리즘을 변경하지 않는 **읽기 전용 사이드카**입니다. 특정 검증 스크립트가 Gazebo를 실행한 뒤 종료해도, 대시보드 자체가 Gazebo를 시작하거나 강제로 재시작하지 않습니다. 실제 ROS 2 Humble + Gazebo Fortress 동작은 사용자 PC에서 확인해야 합니다. ROS/Gazebo 없이 수행한 자동 테스트는 통합 환경 검증이 아닙니다.

### 핵심 규칙

- **실제 위치**: 선택한 Gazebo 월드의 `/world/<world>/{dynamic_}pose/info`에서 수집합니다.
- **로봇 관절**: 선택한 ROS 2 `sensor_msgs/msg/JointState`에서 수집합니다. Gazebo 세계와 관절 데이터의 연결성이 완전히 입증되었다고 가정하지 않습니다.
- **박스 크기**: 동일 `run_id`의 카탈로그 또는 `log_dir/boxes/*.sdf`에서 확인된 경우에만 3D 박스로 그립니다. 크기를 모르면 위치 마커입니다.
- **구역 분류**: 등록된 구역 범위에 들어오는 관측 좌표만 사용하여 후보 위치를 결정합니다. 중심점만 확인한 경우 `center_only`로 표시합니다.
- **공정 상태**: 실행기의 `events.jsonl`에 기록된 완료 주장을 별도로 표시합니다. **실행기 완료 이벤트 ≠ Gazebo 위치 관측 ≠ 물리적 안전성 인증**입니다. 불일치 시 경고합니다.
- **실패 로그**: 활성 실행의 등록된 `log_dir`만 읽습니다. 다른 실행의 기록과 합치지 않습니다.
- **Claude**: 읽기 전용 상태와 로그 근거만 설명합니다. 로봇·컨베이어를 제어할 수 없습니다. API 키가 없어도 기본 진단은 작동합니다.

## 설치 및 실행

```bash
cd ~/AHEAD/pac2026_integrated
mkdir -p tools/pac_gazebo_3d_v5_3
unzip -q "$(xdg-user-dir DOWNLOAD)/PAC2026_GAZEBO_3D_DASHBOARD_V5_3_INTEGRATION.zip" -d tools/pac_gazebo_3d_v5_3

# 기존 시뮬레이터가 실행 중이어도 되고, 나중에 시작해도 됩니다.
# 알고리즘/ROS 실행 터미널과 동일한 통신 환경변수에서 실행하세요.
PAC_ENABLE_CLAUDE=1 bash tools/pac_gazebo_3d_v5_3/start_dashboard_integrated.sh "$PWD"
```

- 대시보드: **http://127.0.0.1:4183**
- 전용 관측 파일: `logs/pac_dashboard_integrated_v53/telemetry.json`
- 연결 확인:

```bash
python3 tools/pac_gazebo_3d_v5_3/check_integrated_v53.py --repo "$PWD"
```

`PAC_ENABLE_CLAUDE=1`을 생략하면 API 키 없이 읽기 전용 오프라인 답변만 사용합니다. API 키는 실행 시 숨김 입력 가능하며 파일에 저장하지 않습니다.

서버·관측기만 이 세션에서 종료할 수 있고, **이미 실행된 Gazebo, MoveIt, 팀 실행기는 종료하지 않습니다.** 카메라 브리지 감시를 끄려면 `PAC_DASHBOARD_CAMERA_BRIDGE=0`을 지정하세요.

## 최종 통합 알고리즘에서 한 번 등록할 파일

새로운 실행이 시작될 때 알고리즘이 다음 파일을 **원자적으로 기록**하면 됩니다.

`logs/pac_dashboard_integration/active_run.json`

```json
{
  "schema_version": "1.0",
  "run_id": "S0001_20261010_090000",
  "world": "ahead_workcell_v4_2_physical_scale",
  "world_sdf": "ros2_ws/src/pac_simulation/worlds/ahead_workcell_v4_6_two_cam.sdf",
  "log_dir": "logs/v44_generator_cycle/S0001_20261010_090000",
  "box_catalog": "logs/v44_generator_cycle/S0001_20261010_090000/boxes.json",
  "events_file": "logs/v44_generator_cycle/S0001_20261010_090000/events.jsonl",
  "topics": {
    "joint_states": "/joint_states",
    "controller": "/joint_trajectory_controller/state",
    "scale_camera": "/pac/scale_camera/image",
    "cctv_camera": "/pac/top_camera/image",
    "gazebo_pose": "/world/ahead_workcell_v4_2_physical_scale/dynamic_pose/info"
  },
  "zones": {
    "pallet": {"x": [-0.55, 0.55], "y": [0.65, 1.75], "z": [0.0, 2.0]},
    "buffer": {"x": [-1.50, -0.60], "y": [0.10, 0.80], "z": [0.0, 2.0]}
  }
}
```

**위 값은 V4.6 구조를 설명하는 예시입니다. 실행 전에 정확한 SDF, 좌표계, 팔레트 교체 상황을 확인해 갱신하세요.** 특히 NG, PICK, 계량, 컨베이어 구역은 각 시나리오의 실제 경계를 확인한 후 등록해야 합니다. 등록하지 않은 구역은 `0`으로 세는 것이 아니라 **분류 정보 미설정**입니다. 동일 공간이 두 구역에 겹치면 해당 박스는 `ambiguous`로 표시됩니다.

- `world`는 SDF `<world name="...">`와 정확히 일치해야 합니다.
- `gazebo_pose`는 실제 게시 중인 `pose/info` 또는 `dynamic_pose/info`를 선택합니다. 없는 토픽은 연결하지 않습니다.
- `world_sdf`는 저장소 `ros2_ws/` 또는 `logs/` 아래의 실재 SDF여야 하며, 다른 월드 SDF를 등록하면 배경 표시를 차단합니다.
- `log_dir`, `box_catalog`, `events_file`은 저장소 `logs/` 아래에 두어야 합니다.
- `box_catalog`는 `{"box_01":{"size_m":[0.41,0.31,0.28]},...}` 형태이며 모델명은 Gazebo와 같아야 합니다.
- 실행기에서 로봇을 구동하는 API를 여기에 넣지 않습니다. 데이터 파일만 갱신합니다.

### Python 실행기 연동 예시

```python
# 통합 실행기 시작 시; 필요한 값은 실제 실행 설정에서 읽으세요.
# tools/ 경로의 모듈은 자동으로 Python 경로에 추가되지 않습니다.
import sys
from pathlib import Path
sys.path.insert(0, str(Path(repo_root) / 'tools/pac_gazebo_3d_v5_3'))
from pac_run_contract_writer import register_active_run, report_event

register_active_run(
    repo_root,
    run_id=run_id,
    world=world_name,
    world_sdf=world_file_relative_to_repo,
    log_dir=run_log_dir_relative_to_repo,
    box_catalog=box_catalog_relative_to_repo,
    events_file=events_relative_to_repo,
    topics=actual_ros_and_gazebo_topics,
    zones=actual_cell_zones,
)

# 관측 및 로봇 실행기에서 해당 단계 완료가 확인된 뒤에만 기록
report_event(repo_root, 'box_05', 'buffer_stored', success=True)
```

`report_event`는 파일에 기록하는 도우미일 뿐, Gazebo/MoveIt의 실제 성공 여부를 확인하지 않습니다. 최종 실행기가 결과 판단 후 호출해야 합니다. `events.jsonl`의 각 행은 다음 구조입니다.

```json
{"run_id":"S0001_20261010_090000","world":"ahead_workcell_v4_2_physical_scale","box_id":"box_05","event":"buffer_stored","success":true,"timestamp_epoch_s":1791580000.0,"reason":""}
```

가능한 이벤트: `on_conveyor`, `at_pick`, `pick_completed`, `pallet_placed`, `buffer_stored`, `ng_diverted`, `held`, `failed`. NG 원인 기록은 `reason`을 사용합니다. 실패 이벤트도 자동 로봇 정지를 의미하지는 않습니다. 보고 주체와 관측 주체가 독립되지 않았다면 두 기록이 일치해도 안전성이 증명되지 않습니다.

## 복수 월드 및 별도 검증 코드

manifest가 없다면 Gazebo `dynamic_pose/info` 토픽을 자동 찾습니다. **다른 월드가 동시에 2개 이상 있으면 임의로 선택하지 않습니다.** 이때는 manifest를 등록하거나 `PAC_GAZEBO_WORLD=<정확한월드명>`으로 대시보드를 시작하세요. 연결이 끊겼다면 새 토픽을 탐색하고, 이전 값은 최신 상태처럼 사용하지 않습니다.

동일한 `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`, `RMW_IMPLEMENTATION`, `IGN_IP`, `IGN_PARTITION`/`GZ_PARTITION`이 시뮬레이터/관측기에서 호환되는지 먼저 확인해야 합니다. **환경이 분리된 서로 다른 프로세스끼리 자동으로 통신 환경을 합치는 기능은 아닙니다.** 시뮬레이터 실행기가 종료된 이후에는 실제 위치 관측을 계속할 수 없습니다.

## 신뢰성의 한계

- `TRACKED POSES`: 필터를 통과한 엔티티 위치 개수. 전체 박스 수가 아닙니다.
- `ZONE OBSERVED`: 최신 Gazebo 좌표를 분류한 수. 정상 적재 개수가 아닙니다.
- `EXECUTOR REPORTED`: 해당 실행기가 남긴 이벤트 수. 물리적 확인 완료 수가 아닙니다.
- `DISCREPANCY`: 이벤트 목적지와 관측 위치가 불일치한 항목. 좌표계·박스 크기·위치 편차·중간 상태를 추가 조사해야 합니다.
- `LAST DATA AGE`: 가장 최근 상태 도착 후 경과시간. 네트워크 E2E 지연·실시간성 보장이 아닙니다.
- `CAMERA`: 파일 존재가 아니라 최신 ROS 영상으로 만든 JPEG 수신 여부입니다.

## 검증

```bash
cd ~/AHEAD/pac2026_integrated/tools/pac_gazebo_3d_v5_3
python3 -m pytest -q
```

자동 테스트와 로컬 HTTP 테스트는 수행했지만, **팀 최종 알고리즘을 실행한 Gazebo에 직접 연결한 종단 간 시험은 아직 하지 않았습니다.** 추후 팀 통합 직후 `check_integrated_v53.py`로 토픽·로그·박스 동기화부터 확인하세요.
