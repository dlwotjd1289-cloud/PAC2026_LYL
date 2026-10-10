# Gazebo V4.6 작업셀에서 실행

가상 셀(`pipeline/`)과 같은 알고리즘을 재성님 V4.6 Gazebo 작업셀에서 돌립니다. 작업셀 코드는 팀 저장소 `jaesung/v46-final-3d-20261010`(`eba65a2`)의 로컬 사본(`PAC_WORKCELL_REPO`, 기본 `../pac2026_integrated`)을 씁니다.

```bash
bash workcell/run_gazebo_demo.sh <DATASET> S0003 12     # 12박스, 0 = 주문 전체
PAC_ENABLE_CLAUDE=1 bash workcell/run_gazebo_demo.sh ...  # 대시보드 Claude 답변
```

Gazebo V4.6 → MoveIt2 + RViz → 브리지 → 실행기 → V5.3 대시보드(http://127.0.0.1:4183) 순서로 뜹니다. PyBullet 뷰어는 쓰지 않습니다.

## 구성

| 단계 | 사용 코드 |
|---|---|
| 주문 | 생성기 데이터셋 + `virtual_data.runtime_cell` (규격 불일치·누락·파손 박스, 가상 셀과 같은 시드) |
| 컨베이어 | `workcell/lyl_cell_runner.py`: 계량→스토퍼 20초, 박스 간격 4초(주행 시간 기준), 계량~스토퍼 5개면 투입 정지. 롤러 구동이 하나라 계량·스토퍼 대기 때 라인 전체가 멈추고 간격은 유지 |
| 인식 | 계량 구간 탑뷰 `rgbd_camera`의 깊이 영상 → 동한님 `measure_box`(규격·눌림·모서리) + 저울 무게 → `build_raw_observation` |
| 판단 | `RuntimeCore` (통합 프로필 + `config/workcell_v46.yaml`: 버퍼 4칸·칸 크기 제한, 재적재 끔) |
| 경로·실행 | V4.6 `run_moveit_pick_place_v44.sh` (MoveIt 경로, CCTV 위치 확인, 집기 재시도·재배치·재계획) |
| NG | 검증 단계 `INSPECTION`·`REJECT_NG` 박스를 V4.6 NG 구역 바닥으로 (`--place-on ng`), 자리 순서는 동한님 Hold/NG(3열×2행, 다음 층) |
| 팔레트 교체 | V4.6 `pallet_swap_v44.py` |
| 대시보드 | 실행 폴더 `logs/lyl_cell/<run>/`의 사이클 로그·`boxes.json`·`events.jsonl` + 실행 등록. 3D는 V4.6 월드 SDF + Gazebo 실시간 위치 |

## 작업셀 변경 (V4.6 사본, 기존 동작은 그대로)

| 파일 | 변경 |
|---|---|
| `worlds/ahead_workcell_v4_6_two_cam.sdf`, `scripts/make_world_v46.py` | 계량 탑뷰 카메라 `camera` → `rgbd_camera` (컬러 토픽 이름 유지, `/pac/scale_camera/depth_image` 추가). 버퍼 선반 모듈 1개 추가(`buffer_rack_ext`, 기존 선반의 상류 0.90 m, 칸 3·4) |
| `scripts/moveit_pick_place_v44.py` | `--place-on ng`, 속도 환경변수 `PAC_FREE_SPEED` / `PAC_CART_SPEED` (기본값 그대로) |
| `scripts/make_box_sdf_v44.py` | `--dent` (파손 박스: 윗면 가운데가 들어간 형상) |
| `scripts/run_generator_cycle_v44.sh` | `PLANNER_BRIDGE` 지정 시 그 플래너 사용 (기존 사이클에 위치 선택만 바꿔 끼울 때, `workcell/lyl_planner_bridge.py`) |

## 좌표

알고리즘 팔레트 좌표계 원점은 로봇 쪽 가까운 모서리(world −0.55, 0.65), 축은 월드와 같습니다. 이때 로봇 베이스는 팔레트 중심 기준 (0, −1.20) m로 로봇 검사 모델의 가정 (0, −1.15)과 맞습니다. 기존 V4.6 브리지는 반대 모서리를 원점으로 축을 뒤집어 쓰므로 그 좌표계를 그대로 넣으면 로봇 위치가 거울상이 됩니다.

깊이 측정 좌표계는 계량대 중심의 롤러 윗면(−5.985, 1.20, 0.895)입니다. 계량대 틀이 롤러보다 35 mm 높아 박스 판정 높이를 30 mm → 60 mm로 올렸습니다(가장 낮은 박스 0.09 m).

## 검증 상태

- 깊이 카메라: 계량대 위 0.41×0.31×0.28 m 박스를 Gazebo 깊이 영상으로 측정 → 0.410×0.310×0.280 m, 위치·회전 일치, 확신도 1.0.
- 월드 생성 스크립트 결과와 SDF가 바이트 단위로 같음(공백 제외).
- 버퍼 칸 4(로봇에서 수평 2.21 m)와 NG 구역 바닥의 도달 가능성, MoveIt 속도 0.5는 Gazebo 실행으로 확인해야 함. 코드 주석상 0.4에서 긴 PTP 이동이 경로 허용오차로 중단된 기록이 있음.
