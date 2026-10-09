#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEAM_ROOT="$(cd "$ROOT/../.." && pwd)"

export PYTHONPATH="$TEAM_ROOT/ros2_ws/src/pac_common:$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"

python3 "$ROOT/scripts/generate_dataset.py" \
  --config "$ROOT/config/default.yaml" \
  --output "$ROOT/generated/sample_dataset" \
  --mode sample \
  --sample-per-family 2

python3 "$ROOT/scripts/validate_dataset.py" \
  "$ROOT/generated/sample_dataset"
