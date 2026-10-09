# Storage Schema v1

## 원칙

- 알고리즘 모듈 간 런타임 계약: `pac_common` dataclass
- YAML/JSON/JSONL: 설정 및 저장/테스트 fixture
- CSV: 분석 전용
- 내부 단위: `m`, `kg`, `s`, `rad`

## Planner-safe scenario (`test_data/Sxxxx.json`)

주요 필드:

- `schema_version`
- `scenario_id`
- `scenario_family`
- `state_kind = SIMULATED`
- `seed`
- `initial_system_state`
- `constraints`

`initial_system_state`는 adapter를 통해 `SystemState`로 복원 가능해야 한다.
정확한 미래 arrival order는 포함하지 않는다.

## Ground Truth (`ground_truth/Sxxxx.json`)

Simulator/Evaluator 전용이다.

- `arrival_events[].arrival_index`
- `arrival_events[].weight_profile_component`
- `arrival_events[].box_state`

`box_state`는 `BoxState` 직렬화 형태다.

## Simulation Observation (`simulation_observations/Sxxxx.jsonl`)

현재는 identity observation만 제공한다. 각 line의 `observation`은 `BoxState` 직렬화 형태다. 실제 CCTV 오차 모델은 향후 Observation Generator에서 주입한다.
