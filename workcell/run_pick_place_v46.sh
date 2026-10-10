#!/usr/bin/env bash
# V4.6 helper lifecycle matches the workcell launcher; execution uses the LYL adapter.
set -eo pipefail
LYL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "${PAC_WORKCELL_REPO:-$LYL_ROOT/../pac2026_integrated}" && pwd)"
export PAC_WORKCELL_REPO="$ROOT"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
BRIDGE_BIN="$(ros2 pkg prefix ros_gz_bridge)/lib/ros_gz_bridge/parameter_bridge"

PIDS=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${PIDS[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'echo "[moveit pick] stopping..."; exit 130' INT TERM

BOX="v43_scale_box_5kg"
ARGS=("$@")
for ((i = 0; i < ${#ARGS[@]}; i++)); do
  if [[ "${ARGS[$i]}" == "--box" ]]; then
    if ((i + 1 >= ${#ARGS[@]})); then echo "--box requires a model name" >&2; exit 2; fi
    BOX="${ARGS[$((i + 1))]}"
  fi
done
mkdir -p "$ROOT/logs"
setsid bash "$ROOT/scripts/run_suction_gripper.sh" &
PIDS+=($!)
setsid "$BRIDGE_BIN" "/model/$BOX/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose" \
  > "$ROOT/logs/v44_moveit_pose_bridge.log" 2>&1 &
PIDS+=($!)
if [[ " $* " == *" --pose-source camera "* && "${PERCEPTION_EXTERNAL:-0}" != 1 ]]; then
  echo ">>> [MoveIt 준비] PICK CCTV 영상 bridge·박스 인식 노드 시작 중..."
  setsid "$BRIDGE_BIN" '/pac/cctv_pick/image@sensor_msgs/msg/Image[ignition.msgs.Image' \
    '/pac/cctv_pallet/image@sensor_msgs/msg/Image[ignition.msgs.Image' \
    > "$ROOT/logs/v44_cctv_bridge.log" 2>&1 &
  PIDS+=($!)
  setsid python3 -u "$ROOT/scripts/box_perception_v44.py" > "$ROOT/logs/v44_perception.log" 2>&1 &
  PIDS+=($!)
fi
# No fixed startup sleep: original.main waits for action/service availability,
# box/suction state, and the matching suction-target ACK before moving.
python3 -u "$LYL_ROOT/workcell/lyl_moveit_executor.py" "$@"
