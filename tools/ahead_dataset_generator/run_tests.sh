#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEAM_ROOT="$(cd "$ROOT/../.." && pwd)"

export PYTHONPATH="$TEAM_ROOT/ros2_ws/src/pac_common:$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"

cd "$ROOT"
python3 -m pytest -q
