# PAC2026 LYL — 혼합 팔레타이징 알고리즘

PAC 2026 Mission 1 (AHEAD 혼합 박스 적재). **주문 데이터 생성 → 박스 인식 → 적재 결정 → 적재 완료 → 평가**를 명령 하나로 실행합니다.

```bash
pip install numpy pyyaml pytest       # Python 3.10+, ROS 불필요
./run_pipeline.sh                     # 6개 주문 x 40박스 → runs/<시각>/summary.md
pip install aiohttp && ./run_dashboard.sh   # 마지막 주문의 실패 분석·3D 대시보드 → http://127.0.0.1:4183
```

## 전체 흐름

```
 0 데이터 생성   tools/ahead_dataset_generator      주문 = 도착 순서·SKU·무게·규격 (6개 유형)
        │
 1 박스 인식     pipeline/depth_perception.py        저울 무게 + 라벨 판독 + 깊이 카메라
        │          └ pac_perception (동한)           규격 측정 · 윗면 눌림 · 모서리 찌그러짐 · SKU 추정
 2 상태 검증     pac_runtime/state_validator.py      정상 / 규격 불일치 / 파손 → 검사대 / 인식 실패
 3 감독          pac_runtime/supervisor.py           팔레트 교체 · 누락 박스 · 버퍼
 4 상위 결정     pac_highlevel (look-ahead N=5)      적재 / 버퍼 / 버퍼에서 꺼냄 / 팔레트 마감 / 재적재
 5 위치 선택     pac_candidates + LayerPlacer (태현) 후보 생성 → 지지·하중·CoG 검사 → 층 쌓기 순위
 6 로봇 검사     pac_robot_check                     HDR50-22 도달 · 충돌 · 사이클 시간
 7 실행 (가상)   pac_runtime/executor.py             놓기 오차 · 흡착 실패 · 사후 검증 L0~L4
 8 상태 갱신     pac_runtime/state_manager.py        측정 상태 반영 → 다음 박스
        │
 + 평가          tools/algorithm_integration (재성)  공식 5항목: 공간 · 시간 · 하중 · 안정성 · 실행 가능성
 + 대시보드      tools/pac_failure_analysis_dashboard 실패 로그 분석 · Claude 질의 · 3D 적재 결과 (읽기 전용)
```

1~8단계는 ROS 노드(`pac_runtime runtime_node`)가 쓰는 것과 같은 `RuntimeCore` 하나가 결정합니다. 이 저장소의 실행기는 그 주변의 공장(컨베이어, 카메라 영상, 로봇 동작)을 가상으로 만들어 돌립니다.

### 0. 데이터 생성 (재성)

`tools/ahead_dataset_generator` v1.4.0. 6개 유형(`normal`, `size_mixed`, `weight_mixed`, `late_large`, `late_heavy`, `repeated_sku`)의 주문을 만들고, 팔레트 규격은 `config/default.yaml`(1.10 × 1.10 m, 데크 위 1.5 m)을 따릅니다. 실행기가 생성기를 호출하며, 따로 쓸 때는 `tools/ahead_dataset_generator/README.md`를 보세요.

가상 셀은 주문에 현장 이상을 더합니다(`tools/virtual_data/runtime_cell.py`): 박스 2 %가 SKU와 다른 규격(한 변 ×0.85/×1.15), 2 %는 도착하지 않음, 일부 박스는 생성기 강도 모델에 따라 파손 상태입니다.

### 1. 박스 인식 (동한님 깊이 알고리즘)

`pipeline/depth_perception.py`가 태현님 가상 인식(`PerceptionSim`, 확률로 결과를 정하는 방식)을 대신합니다. 박스마다 다음 순서로 처리합니다.

1. `pipeline/depth_camera.py`가 스토퍼에 닿은 박스의 **위에서 본 깊이 영상**을 그립니다. 다음 박스가 뒤에 줄 서 있을 수 있고, 파손 박스는 윗면 눌림(25 mm) 또는 모서리 찌그러짐(60 mm)이 형상으로 그려집니다. 깊이 잡음 1 mm, 결측 화소 0.2 %.
2. 동한님 `measure_box`가 영상만 보고 규격·자세·눌림·기울기·모서리 형상·확신도를 계산합니다. 줄 선 다음 박스는 컨베이어 방향 간격으로 분리합니다. **정답 크기나 파손 여부를 인식 결과로 넘기지 않습니다.**
3. 라벨을 못 읽으면 `resolve_sku`가 측정 규격과 무게로 SKU를 추정합니다.
4. `build_raw_observation` → `RawObservation` → 2단계 `StateValidator`.

미리보기 카메라(다음 최대 5개 박스, look-ahead 입력)도 같은 깊이 측정을 씁니다. 실행이 끝나면 숨겨 둔 정답과 비교한 인식 성능(규격 오차, 파손 검출/오검출, SKU 추정 정확도)을 보고서에 남깁니다.

### 5. 위치 선택 + 평가 (태현 위치 선택, 재성 사후 평가)

통합 묶음(`integration/jaesung-taehyeon-algorithm`, `6f12962`) 그대로입니다. 위치는 태현님 후보 생성과 `LayerPlacer`가 고르고, 재성님 비선형 점수(`C = E − φS(S) − φL(L)`)는 **선택을 바꾸지 않고** 주문이 끝난 뒤 선택된 동작마다 계산합니다. 자세한 식과 한계는 [docs/02_stacking_algorithm.md](docs/02_stacking_algorithm.md).

설정은 `config/jaesung_taehyeon/`(통합 프로필): N=5 행동 탐색, 결정 제한 7초, 다음 5개 박스 모두 실측 무게·규격 사용, 박스별 허용 하중.

## 실행

```bash
python3 -m pipeline.run --output runs/demo                     # 생성 + 전체 주문
python3 -m pipeline.run --output runs/demo --ids S0001 --boxes 80
python3 -m pipeline.run --output runs/again --dataset runs/demo/dataset   # 같은 데이터로 재실행
python3 -m pipeline.run --help
```

설정은 `config/pipeline.yaml`(생성 개수, 가상 셀 이상 비율, 카메라·ROI, 파손 형상, 주문당 제한 시간).

### 출력

```
runs/demo/
  dataset/              생성기 출력 (test_data/, ground_truth/, manifest.json)
  summary.md            주문별 표: 적재·검사대·미처리·팔레트 수·적재율·결정 시간·인식 성능
  summary.json          같은 내용 + 실행 설정
  orders/S0001.json     주문 하나의 전체 기록
```

`orders/<id>.json`의 주요 키:

| 키 | 내용 |
|---|---|
| `result.pallet_list[].layout` | 팔레트별 최종 적재 배치 (박스 id, 규격, 무게, 위치·yaw). 대시보드·3D 뷰어 입력 |
| `result.events` | 시간순 이벤트 (ARRIVE, PLACE, BUFFER, NG, CLOSE, MISSING …) |
| `result.inspection` | 검사대로 보낸 박스와 사유 |
| `decisions` | 결정마다 걸린 시간·동작·사유, 최대/평균, 7초 초과 수 |
| `perception`, `perception_records` | 인식 요약과 박스별 측정 기록 |
| `evaluation` | 선택된 동작별 재성 5항목 평가 |

## 첫 실행 결과

6개 주문 × 40박스: 5개 주문 완료, 1개(S0002)는 결정 하나가 7.7초로 제한을 넘어 중단. 파손 박스 7/7 검출, 오검출 0, 규격 오차 평균 1.5~1.7 mm, 5항목 평가 292건 오류 0. 자세한 표와 해석은 [docs/03_verification_20261010.md](docs/03_verification_20261010.md). 이후 7초 초과 시 계산된 명령으로 계속하도록 바꾼 뒤 S0002도 완료(39개 적재 + 파손 1개 검사대, 팔레트 2개).

## 대시보드 (실패 분석 · Claude · 3D)

`tools/pac_failure_analysis_dashboard`는 팀 저장소 PR #12(V5.3, `f0a171a`)의 원본 그대로입니다. 알고리즘을 제어하지 않고 `logs/` 아래 파일만 읽는 사이드카이며, 파이프라인이 주문마다 그 계약 형식으로 결과를 써 줍니다(`pipeline/dashboard_export.py`).

```
logs/pipeline/<주문>_<UTC시각>/
  pipeline_cycle.log     전체 타임라인 (PASS / WARN / FAIL / ERROR)
  executor_alerts.log    경고·실패만 (대시보드는 로그 끝부분만 읽으므로 분리)
  boxes.json             박스 실제 규격
  events.jsonl           on_conveyor · pallet_placed · buffer_stored · ng_diverted · failed
  world.sdf              팔레트와 최종 적재 배치 (3D 화면의 정적 형상)
logs/pac_dashboard_integration/active_run.json   마지막으로 끝난 주문을 가리킴
```

```bash
./run_dashboard.sh                         # API 키 없이: 로그 근거의 규칙 기반 진단
PAC_ENABLE_CLAUDE=1 ./run_dashboard.sh     # Claude API 질의 (키는 $ANTHROPIC_API_KEY 또는 숨김 입력, 저장 안 함)
python3 tools/pac_failure_analysis_dashboard/pac_diagnostic_console.py ask \
  --url http://127.0.0.1:4183 '마지막 적재가 왜 실패했어?'
```

예: S0002를 실행한 뒤 물으면 `[planner] ERROR order stopped: DECISION_BUDGET_EXCEEDED after 7.76s; unprocessed=18`을 근거로 계획 단계 실패라고 답합니다. 파일 출력을 끄려면 `pipeline.run --no-dashboard`.

- 가상 실행이라 Gazebo 박스 위치·로봇 관절·카메라는 없습니다. 화면의 "TRACKED POSES" 등 실시간 항목은 비어 있고, 3D 박스는 **관측이 아니라 계획·가상 실행된 최종 배치**입니다.
- Gazebo V4.6 작업셀과 함께 쓸 때는 대시보드의 `start_dashboard_integrated.sh`와 `INTEGRATION_GUIDE_KO.md`를 따르세요 (ROS 2 Humble 필요).
- Claude API 모드는 이번 정리에서 실제 키로 호출해 보지 않았습니다 (대시보드 자체 시험은 모의 응답 기준).

## 테스트

```bash
python3 -m pytest -q
```

생성기, 깊이 인식, 상태 검증, 후보 생성·하중 검사, look-ahead, 로봇 검사, 런타임, 5항목 평가, 생성부터 적재까지 한 번에 도는 파이프라인 시험, 대시보드 파일 계약 시험(`tests/pipeline`)을 포함합니다. 대시보드 자체 시험은 따로 돌립니다: `cd tools/pac_failure_analysis_dashboard && python3 -m pytest -q` (aiohttp 필요).

## 폴더

| 위치 | 내용 | 담당 |
|---|---|---|
| `pipeline/` | 전체 실행기, 가상 깊이 카메라, 깊이 인식 연결 | 통합 |
| `tools/ahead_dataset_generator/` | 주문 데이터 생성기 | 재성 |
| `ros2_ws/src/pac_perception/` | 깊이 측정, SKU 추정 | 동한 |
| `ros2_ws/src/pac_runtime/` | 1~8단계 코어, ROS 노드, launch | 태현 |
| `ros2_ws/src/pac_candidates/` | 후보 생성, 하드 검사 (지지·하중·CoG) | 태현 |
| `ros2_ws/src/pac_highlevel/` | 상위 결정, look-ahead, 재성 통합 점수 | 태현 / 재성 |
| `ros2_ws/src/pac_robot_check/` | HDR50-22 실행 가능성 | 태현 |
| `ros2_ws/src/pac_common/` | 공통 자료형·설정 계약 | 공통 |
| `tools/virtual_data/` | 생성기 주문 → 가상 셀 | 태현 |
| `tools/algorithm_integration/` | 5항목 사후 평가 | 재성 |
| `tools/pac_failure_analysis_dashboard/` | 실패 분석·Claude·3D 대시보드 V5.3 | 재성 |
| `config/` | `default.yaml`(공통), `jaesung_taehyeon/`(통합 프로필), `taehyeon/`, `donghan/perception_depth.yaml`, `pipeline.yaml` | |
| `docs/` | 인식·적재 알고리즘 설명 | |

ROS 2 Humble에서는 `ros2_ws`를 colcon으로 빌드한 뒤 `ros2 launch pac_runtime merged_algorithm.launch.py repo:=<이 저장소> order_file:=<주문.json>`으로 같은 코어를 노드로 띄울 수 있습니다. 이번 정리에서 ROS 실행은 확인하지 않았습니다.

## 알려진 한계

- **가상 환경 결과입니다.** 깊이 영상은 합성이고, 로봇은 기구학 모델, 놓기 결과는 확률 모델입니다. 실물 운반 안전성, 실제 카메라 성능을 증명하지 않습니다.
- **결정이 7초를 넘어도 멈추지 않습니다** (`planning.use_late_command: true`, 2026-10-10). 그때까지 계산된 최선의 명령을 그대로 실행하고 `decision_budget_exceeded_used_best`로 셉니다. 실시간 보장이 아니라 늦은 결정을 허용하는 것이며, 7초를 넘긴 시간은 `decisions`에 그대로 남습니다. `false`로 두면 예전처럼 주문을 멈춥니다.
- **동한님 ROI 설정과 스토퍼 위치가 3 mm 겹칩니다.** `perception_depth.yaml`의 ROI는 x 0.232에서 끝나는데 스토퍼 면은 0.235라, 스토퍼에 닿은 박스는 항상 "시야 경계에 걸림"으로 판정되어 파손 판정이 생략됩니다. 가상 카메라에는 스토퍼가 없으므로 `config/pipeline.yaml`에서 ROI를 0.26까지 넓혔습니다. 실제 셀 값은 동한님 확인이 필요합니다 (`tests/pipeline` 에 재현 시험 있음).
- 컨베이어 좌표계 원점, 파손 임계값(눌림 12 mm 등)은 동한님 문서의 가정값 그대로입니다.
- Gazebo V4.6 작업셀·MoveIt 실행기, 동한님 학습 랭커(`pac_planning`)는 이 저장소에 포함하지 않았습니다.
