#!/usr/bin/env bash
# V5.2 Claude: Start ONLY its own read-only observer / optional Gazebo-to-ROS camera
# image bridge / local web server. No robot, MoveIt or Gazebo commands.
set -eo pipefail
REPO="${1:-$HOME/AHEAD/pac2026_integrated}"
REPO="$(cd "$REPO" && pwd)"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$REPO/ros2_ws/install/setup.bash"
export IGN_IP="${IGN_IP:-127.0.0.1}"
OUT="$REPO/logs/pac_dashboard_3d_v5"
mkdir -p "$OUT"
STATE="$OUT/telemetry.json"
PORT="${PAC_DASHBOARD_PORT:-4182}"
OBSERVER_PID=""; BRIDGE_PID=""
cleanup(){
  trap - EXIT INT TERM
  for pid in "$OBSERVER_PID" "$BRIDGE_PID"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM
# Independent camera publishers may already exist (e.g. during the team's
# generator cycle). Do not launch a competing bridge in that case.
if [[ "${PAC_DASHBOARD_CAMERA_BRIDGE:-1}" == 1 ]]; then
  GZ_TOPICS="$(timeout 6 ign topic -l 2>/dev/null || true)"
  ROS_TOPICS="$(timeout 6 ros2 topic list -t 2>/dev/null || true)"
  ARGS=()
  for cam in scale_camera top_camera; do
    topic="/pac/$cam/image"
    if grep -Fxq "$topic" <<< "$GZ_TOPICS"; then
      PUBS=0
      if grep -Fq "$topic [sensor_msgs/msg/Image]" <<< "$ROS_TOPICS"; then
        TOPIC_INFO="$(timeout 4 ros2 topic info "$topic" 2>/dev/null || true)"
        PUBS="$(sed -n 's/^[[:space:]]*Publisher count:[[:space:]]*\([0-9][0-9]*\).*$/\1/p' <<< "$TOPIC_INFO" | head -1)"
        PUBS="${PUBS:-0}"
      fi
      if ((PUBS > 0)); then
        echo "[카메라] 기존 ROS 이미지 발행자 $PUBS개 확인: $topic (중복 브리지 방지)"
      else
        ARGS+=("$topic@sensor_msgs/msg/Image[ignition.msgs.Image")
      fi
    else
      echo "[카메라] Gazebo 이미지 토픽 미발견: $topic (이 카메라 브리지 생략)"
    fi
  done
  if ((${#ARGS[@]})); then
    if ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
      echo "[카메라] Gazebo → ROS 이미지 브리지 시작: ${#ARGS[@]}개"
      ros2 run ros_gz_bridge parameter_bridge "${ARGS[@]}" > "$OUT/camera_bridge.log" 2>&1 &
      BRIDGE_PID=$!
    else
      echo "[카메라] ros_gz_bridge 설치 상태 확인 필요 · 브리지 미시작"
    fi
  fi
else
  echo '[카메라] PAC_DASHBOARD_CAMERA_BRIDGE=0: 자동 읽기 전용 브리지 사용 안 함'
fi
python3 -u "$HERE/pac_ros_observer.py" run --repo "$REPO" --state-file "$STATE" \
  --write-interval 0.25 --gazebo-interval 0.2 > "$OUT/observer.log" 2>&1 &
OBSERVER_PID=$!
# V5.2 Claude: opt in explicitly. API credentials never appear in arguments.
# Without API opt-in, any accidental credentials are ignored; offline Q&A still works.
KEY_ARGS=(--provider claude)
if [[ "${PAC_ENABLE_CLAUDE:-0}" != 1 ]]; then
  unset ANTHROPIC_API_KEY OPENAI_API_KEY || true
elif [[ -z "${ANTHROPIC_API_KEY:-}" ]]; then
  KEY_ARGS+=(--ask-key)
fi
printf '\nPAC2026 V5.2 + Claude (Gazebo read-only)\n관측기 PID: %s\n로그: %s\n대시보드: http://127.0.0.1:%s\n' "$OBSERVER_PID" "$OUT" "$PORT"
if [[ -n "$BRIDGE_PID" ]]; then printf '이 세션에서 실행한 카메라 브리지 PID: %s\n' "$BRIDGE_PID"; fi
printf '여기에서 Ctrl+C를 누르면 이 세션의 관측기/브리지만 종료합니다. Gazebo/MoveIt은 유지됩니다.\n'
python3 -u "$HERE/pac_diagnostic_console.py" serve --repo "$REPO" \
  --live-state-file "$STATE" --port "$PORT" "${KEY_ARGS[@]}"
