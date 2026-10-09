> 원문: 팀 저장소 `docs/donghan/perception_depth.md` (커밋 `7f5a63d`). 브라우저 시뮬레이터(`viewer/donghan/…`)와 재인식 정책 문서는 원 팀 저장소에만 있습니다. 이 저장소에서 깊이 계산부를 런타임에 연결한 방식은 [README](../README.md#1-박스-인식-동한님-깊이-알고리즘)를 보세요.

# 1단계 인식: Top-view 깊이 측정과 SKU 추정 (동한)

`docs/flow/01_perception.md`의 미구현 항목 "깊이(RGB-D) 처리"를 채우는 계산부입니다. 합성 깊이 이미지와 브라우저 시뮬레이터로만 검증했습니다. Gazebo 카메라는 아직 RGB이므로 실제 깊이 카메라 연결은 남아 있습니다.

## 흐름에서의 위치

```
무게 (AutoScaleCycle.measured_kg) ─┐
ID·라벨 (판독 결과 또는 None) ─────┼─► build_raw_observation ─► RawObservation ─► StateValidator (2단계)
Top-view RGB-D ─ measure_box ──────┤
라벨 미판독 시 ─ resolve_sku ──────┘
```

인식은 관측만 만듭니다. 이상 판정과 경로는 2단계가 정합니다. 출력은 `pac_runtime.perception.RawObservation` 그대로입니다.

## 파일 (`ros2_ws/src/pac_perception/pac_perception/`, 새 파일만 추가)

| 파일 | 내용 |
|---|---|
| `depth_measurement.py` | 깊이 이미지 → `conveyor` frame 점군 → 크기(긴 변·짧은 변·높이), 위치·yaw, 파손 단서(윗면 눌림, 기울어짐, 모서리 찌그러짐: 면적 비율 + 모서리 빈 거리), 확신도. 대기 중인 다음 박스를 컨베이어 방향으로 분리(`target_x`, 앞뒤 간격), ROI·이미지 경계에서 잘린 박스는 `in_full_view=False`로 파손 판단을 생략 |
| `sku_resolver.py` | 라벨 미판독 시 크기·무게로 SKU 추정. 허용오차는 `ValidatorConfig`와 같은 의미. 후보가 여럿이거나 없으면 라벨 없음 |
| `raw_observation.py` | 무게 + 깊이 측정 (+ 라벨) → `RawObservation` |
| `perception_config.py` | `config/donghan/perception_depth.yaml` 로더 |
| `signals.py` | `RawObservation`에 없는 원신호 (`PerceptionSignals`, 재인식에서 사용: [reinspection_policy.md](reinspection_policy.md)) |

`pac_perception/__init__.py`는 비어 있는 그대로 두었습니다. 모듈 경로로 import 합니다(`from pac_perception.depth_measurement import measure_box`).

## 가정

- `conveyor` frame: 저장소에 원점 정의가 없어 **픽업 구역 중심 (-1.08, 1.20)의 롤러 윗면 z 0.895, 축은 world와 같음**으로 가정했습니다. 팀 확정이 필요합니다.
- PICK ROI `[-0.50, 0.232, -0.375, 0.375]`: 스토퍼 면(x -0.845)에 기댄 가장 긴 박스(0.66 m)까지, 스토퍼 윗면과 레일은 제외합니다.
- 임계값(눌림 12 mm, 모서리 빈 거리 20 mm, 유효 깊이 85 % 등)은 시험용 가정치입니다.

## 시뮬레이터 (`viewer/donghan/perception_sim.html`)

브라우저로 바로 엽니다(three.js CDN). 재성 님 V4.4 작업셀 배치(컨베이어, 인라인 저울, PICK 스토퍼, 기둥 CCTV 자세·화각, 팔레트, 2칸 버퍼 테이블, 도착 흔들림)를 그대로 쓰고, 위 계산부와 `StateValidator`, `pac_highlevel.rules.RulePolicy`를 JS로 옮겨 연속 투입, 정지 계량, 분류·적재 연산 중 정지, 버퍼, 팔레트 교체를 돌립니다.

- 초입 기둥 카메라(**위치 가정**, Gazebo 파일 미공개): 붙어서 들어온 박스 감지 → 뒤 박스 정지(갭 생성), 측면 라벨 2차 판독, Base-view 대체, 초입/PICK 크기 비교.
- "초입 카메라 유무 비교" 버튼: 같은 시드로 두 번 실행. 예: `demo_original8 ×6`, 붙어서 투입 30 % → 판정 오류 0 vs 8, 함께 계량 0 vs 19.
- 계산부만 node로: `node viewer/donghan/check_core.js demo_original8 3 6 0.3 5`
- 로봇 팔 형상, 팔레트 배치 규칙(2 cm 높이맵), Hold/NG 위치, 존 축적 컨베이어는 시각화용 단순화입니다.

## 테스트

`tests/donghan/test_depth_measurement.py`, `test_sku_resolver.py`, `test_stage1_to_stage2_contract.py`(출력이 `StateValidator`를 그대로 통과), 보조 `perception_testkit.py`.
