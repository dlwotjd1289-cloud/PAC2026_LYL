# PAC2026 팀 공통 개발 기준 v0.2 준수표

| 기준 | v1.2 대응 |
|---|---|
| Ubuntu 22.04 / Python 3.10 | Python 3.10 문법 호환 테스트 포함 |
| 설정 YAML | `config/default.yaml`, `robot.yaml`, `local.example.yaml` |
| 테스트 데이터 JSON | `test_data/*.json`, 생성 dataset의 `test_data/` |
| 로그 JSONL | `logs/dataset_generation.jsonl` |
| 내부 SI 단위 | m, kg, s, rad |
| 오른손/공통 frame | 생성 Box Pose에 `frame_id=conveyor`, pallet pose 계약은 `pallet` |
| pac_common canonical model | `Size3D/Pose3D/BoxState/PalletState/InventoryState/SystemState` 단일 정의 |
| JSON raw dict 직접 계약 금지 | serialization adapter에서 dataclass로 복원 |
| orientation | `allowed_yaws_rad=(0, pi/2)` |
| Single Writer | Generator는 runtime `SystemState`를 변경하지 않고 fixture만 생성 |
| ACTUAL/PLANNED/SIMULATED 구분 | 생성 fixture에 `state_kind=SIMULATED` 명시 |
| Scenario-level split | train/val/test 분할 + overlap validator |
| Reproducibility | seed + core file SHA-256 + pytest 재현성 테스트 |
| pytest | Contract/Reproducibility/Schema/Python3.10 테스트 |
| Robot feasibility 분리 | `config/robot.yaml`에 `feasibility_owner: pac_robot`; Generator 미판정 |

## 의도적으로 Generator 책임 밖에 둔 항목

- Candidate / Hard Constraint / Score / Look-ahead 결과
- Robot payload / IK / collision / approach / retreat 판정
- Execution / State Manager 상태 갱신
- CCTV noise, missing detection, pose error의 실제 분포

이는 공통 계약 위반이 아니라 모듈 책임 분리다.
