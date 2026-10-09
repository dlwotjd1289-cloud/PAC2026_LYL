# Changelog

## v1.4.0

- 시나리오 seed를 `base + index * 1009`에서 SHA-256 해시 파생으로 변경.
  base seed가 1009의 배수만큼 다른 두 실행이 같은 시나리오를 재생산하던 문제 수정.
  (같은 seed라도 v1.3 이하와 생성 결과가 다름. 각 시나리오 seed는 그대로 기록되어 재현 가능.)
- 같은 데이터셋 안에서 SKU 구성(multiset)이 같은 시나리오를 재샘플링해 제거.
  순서만 다른 시나리오가 train/test에 동시에 들어가는 누출 방지.
  공간이 부족해 못 피한 수는 `coverage_report.json`의 `duplicate_inventory_scenarios`에 기록.
- `repeated_sku`: 최소 2개 block, 연속 block에 같은 SKU 금지, block 크기를 박스 수에 맞게 축소.
  박스 수가 적을 때 시나리오 전체가 SKU 하나로 채워지던 문제 수정.
- `late_large`/`late_heavy` 재번호 중복 코드 제거(출력 동일).
- 크기와 무관한 무게(stress) 비율을 약 7% → 15%로 상향(`weight_profile_distribution`).
  큰 박스가 대체로 무겁지만 예외도 나오도록 함. corr(부피, 무게) 0.66 → 약 0.5 (normal family).
- `boxes_per_scenario < 1`이면 설정 검증에서 거부(빈 시나리오 생성 방지).

## v1.2.0

- Ubuntu 22.04 / Python 3.10 기준으로 계약 정리.
- 모든 내부 길이 단위를 mm에서 m로 변경.
- orientation 저장을 degree에서 rad로 변경.
- `pac_common` canonical dataclass 추가 및 Generator가 이를 import하도록 변경.
- JSON 설정을 YAML로 변경.
- canonical test fixture를 JSON/JSONL로 변경하고 CSV는 analysis 전용으로 축소.
- 모든 Pose에 `frame_id` 포함.
- Planner-safe scenario와 exact future ground truth 물리 분리.
- pytest Contract / Reproducibility / Schema / Python 3.10 syntax 테스트 추가.
- Python 3.13 `__pycache__` 제거.
- Robot feasibility는 Generator 밖에 유지.
