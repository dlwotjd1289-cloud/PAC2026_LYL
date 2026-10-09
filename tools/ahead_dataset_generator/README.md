# AHEAD Dataset Generator

PAC 2026 HD현대로보틱스 미션 1 Mixed Palletizing을 위한 재현 가능한 합성 데이터/시나리오 생성기입니다.

- Target environment: Ubuntu 22.04 LTS / Python 3.10
- Contract: PAC 2026 팀 공통 개발 기준 v0.2 준수
- Internal units: SI (m, kg, s, rad)
- Configuration: YAML
- Runtime contract: `pac_common` dataclasses
- Canonical test fixtures: JSON
- Tests: pytest

자세한 사용법은 아래 문서와 스크립트를 참고하세요.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
./run_tests.sh
./run_sample.sh
```
