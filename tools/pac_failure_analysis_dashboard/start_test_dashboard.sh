#!/usr/bin/env bash
# Read-only, isolated TEST server. Doesn't launch or stop Gazebo/ROS/MoveIt/observer.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="${1:-$HOME/AHEAD/pac2026_integrated}"
REPO="$(cd "$REPO" && pwd)"
RUN="${2:-}"
if [[ -z "$RUN" ]]; then
  shopt -s nullglob
  candidates=("$REPO"/logs/pac_dashboard_test_scenarios/TEST_*/)
  if ((${#candidates[@]} == 0)); then
    echo "테스트 기록 없음. 먼저 scenario_fail_ik.py --mode sample 또는 --mode moveit-ik 실행" >&2
    exit 2
  fi
  IFS=$'\n' read -r RUN < <(ls -dt "${candidates[@]}")
fi
RUN="$(cd "$RUN" && pwd)"
case "$RUN" in "$REPO"/logs/pac_dashboard_test_scenarios/TEST_*) ;; *)
 echo "테스트 폴더가 아닙니다: $RUN" >&2; exit 2;; esac
[[ -f "$RUN/test_manifest.json" ]] || { echo "test_manifest.json 파일이 없습니다" >&2; exit 2; }
PORT="${PAC_TEST_DASHBOARD_PORT:-4180}"
TELEMETRY="$REPO/logs/pac_dashboard_3d_v5/telemetry.json"
# Reuses V5 observation file if the separate V5 observer is already running.
# Does not generate telemetry and never claims simulated log as real sensor data.
unset OPENAI_API_KEY ANTHROPIC_API_KEY  # TEST dashboard always stays offline.
unset PAC_ENABLE_CLAUDE PAC_ENABLE_API  # No API calls in TEST mode.
exec python3 -u "$HERE/pac_diagnostic_console.py" serve \
  --repo "$REPO" --run-dir "$RUN" --port "$PORT" --live-state-file "$TELEMETRY" \
  --provider claude
