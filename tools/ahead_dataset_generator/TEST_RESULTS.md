# Test Results — v1.2.0

검증 기준: 빌드 컨테이너에서 Python source를 Python 3.10 grammar로 파싱하고, pytest/생성/Validator를 실행했다.

## pytest

```text
4 passed
```

검증 항목:

- Python 3.10 grammar parse
- YAML config -> Generator
- JSON -> `pac_common.SystemState` contract
- Ground Truth JSON -> `pac_common.BoxState` contract
- `frame_id` / SI unit 계약
- Planner-safe fixture에 future order/`true_*` leakage 없음
- 동일 seed core file hash 일치

## Sample dataset

```text
12 scenarios
288 boxes
13 SKU types
6 scenario families x 2
parcel-domain violations: 0
contract validator: PASS
```

## Fixed benchmark

```text
600 scenarios
14,400 boxes
normal: 100
size_mixed: 100
weight_mixed: 100
late_large: 100
late_heavy: 100
repeated_sku: 100
parcel-domain violations: 0
contract validator: PASS
```

## 주의

빌드 환경 자체의 `python3`는 3.13이었기 때문에 런타임을 3.10에서 직접 실행한 것은 아니다. 대신 모든 Python source를 `ast.parse(..., feature_version=(3, 10))`로 검사하여 Python 3.10 문법 호환을 확인했고, 코드에는 3.10 이후 전용 문법/API를 사용하지 않았다. 실제 목표 환경은 Ubuntu 22.04 + Python 3.10이다.
