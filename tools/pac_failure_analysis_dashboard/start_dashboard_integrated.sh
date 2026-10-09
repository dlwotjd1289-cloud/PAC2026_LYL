#!/usr/bin/env bash
# Integration-ready, read-only dashboard. Gazebo/MoveIt are NEVER launched/killed.
set -euo pipefail
ROOT="${1:-$HOME/AHEAD/pac2026_integrated}"
ROOT="$(cd "$ROOT" && pwd)"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
OUT="$ROOT/logs/pac_dashboard_integrated_v53"
mkdir -p "$OUT"
STATE="$OUT/telemetry.json"
PORT="${PAC_DASHBOARD_PORT:-4183}"
ARGS=(--repo "$ROOT" --state-file "$STATE")
SRV=(--repo "$ROOT" --live-state-file "$STATE" --port "$PORT" --provider claude)
WATCH=(--repo "$ROOT" --log-file "$OUT/camera_bridge.log")
if [[ -n "${PAC_BINDING_FILE:-}" ]]; then
  ARGS+=(--binding-file "$PAC_BINDING_FILE")
  SRV+=(--binding-file "$PAC_BINDING_FILE")
  WATCH+=(--binding-file "$PAC_BINDING_FILE")
fi
if [[ -n "${PAC_GAZEBO_WORLD:-}" ]]; then
  ARGS+=(--world "$PAC_GAZEBO_WORLD")
fi
if [[ -n "${PAC_GAZEBO_POSE_TOPIC:-}" ]]; then
  ARGS+=(--gazebo-pose-topic "$PAC_GAZEBO_POSE_TOPIC")
fi
if [[ "${PAC_ENABLE_CLAUDE:-0}" != 1 ]]; then
  unset ANTHROPIC_API_KEY OPENAI_API_KEY || true
elif [[ -z "${ANTHROPIC_API_KEY:-}" ]]; then
  SRV+=(--ask-key)
fi

observer_pid=''; watcher_pid=''
cleanup(){
  trap - EXIT INT TERM
  for pid in "$observer_pid" "$watcher_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM
python3 -u "$HERE/pac_ros_observer.py" run "${ARGS[@]}" \
  > "$OUT/observer.log" 2>&1 &
observer_pid=$!
if [[ "${PAC_DASHBOARD_CAMERA_BRIDGE:-1}" == 1 ]]; then
  python3 -u "$HERE/pac_camera_bridge_watch.py" "${WATCH[@]}" \
    > "$OUT/camera_watch.log" 2>&1 &
  watcher_pid=$!
fi
printf 'PAC2026 통합 연동 준비형 V5.3 (읽기 전용)\n'
printf '접속 http://127.0.0.1:%s\n' "$PORT"
printf '관측기 PID: %s / 카메라 감독 PID: %s\n' "$observer_pid" "${watcher_pid:-disabled}"
printf 'Gazebo/MoveIt은 이 스크립트가 실행·종료하지 않습니다.\n'
printf '실행 manifest: %s\n' "${PAC_BINDING_FILE:-$ROOT/logs/pac_dashboard_integration/active_run.json}"
printf '로그: %s\n' "$OUT"
python3 -u "$HERE/pac_diagnostic_console.py" serve "${SRV[@]}"
