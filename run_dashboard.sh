#!/usr/bin/env bash
# Failure-analysis dashboard on the latest pipeline run (no ROS / Gazebo needed).
#   ./run_dashboard.sh                         # offline answers, http://127.0.0.1:4183
#   PAC_ENABLE_CLAUDE=1 ./run_dashboard.sh     # + Claude API (key: $ANTHROPIC_API_KEY or hidden prompt)
# The run shown is logs/pac_dashboard_integration/active_run.json, written by
# `python3 -m pipeline.run` for the last order it finished.
# With the Gazebo cell running, use tools/pac_failure_analysis_dashboard/start_dashboard_integrated.sh instead.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HERE="$ROOT/tools/pac_failure_analysis_dashboard"
PORT="${PAC_DASHBOARD_PORT:-4183}"
if [[ ! -f "$ROOT/logs/pac_dashboard_integration/active_run.json" ]]; then
  echo "No active run yet: run ./run_pipeline.sh first." >&2
  exit 1
fi
ARGS=(serve --repo "$ROOT" --port "$PORT" --provider claude
      --live-state-file "$ROOT/logs/pac_dashboard_integrated_v53/telemetry.json")
if [[ "${PAC_ENABLE_CLAUDE:-0}" != 1 ]]; then
  unset ANTHROPIC_API_KEY OPENAI_API_KEY || true
elif [[ -z "${ANTHROPIC_API_KEY:-}" ]]; then
  ARGS+=(--ask-key)
fi
mkdir -p "$ROOT/logs/pac_dashboard_integrated_v53"
exec python3 -u "$HERE/pac_diagnostic_console.py" "${ARGS[@]}"
