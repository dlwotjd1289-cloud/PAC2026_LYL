# Common test fixtures

이 폴더의 001~004는 Dataset Generator가 소유할 수 있는 공통 fixture다.

- `scenario_001_basic`
- `scenario_002_random_order`
- `scenario_003_heavy_box_late`
- `scenario_004_same_sku_sequence`

각 planner-safe JSON은 정확한 미래 arrival order를 포함하지 않으며, 대응하는 simulator/evaluator 전용 정답은 `ground_truth/` 아래에 분리한다.

공통 기준서의 005~012 (`out_of_bound`, `low_support`, `robot_infeasible`, `size_corrected`, `actual_pose_error`, `timeout`, `stale_plan`, `tracking_lost`)는 Validator/Robot/State Manager/Perception 통합 동작을 시험하는 fixture이므로 Dataset Generator가 실패 결과를 선결정하지 않는다. 해당 모듈 통합 테스트에서 이 Generator의 기본 fixture를 조합해 관리한다.
