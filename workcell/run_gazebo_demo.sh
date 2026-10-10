#!/usr/bin/env bash
# One order of the PAC2026 LYL algorithm on the Gazebo V4.6 cell:
#   Gazebo V4.6 world -> MoveIt2 + RViz -> camera / sensor bridges -> runner -> V5.3 API dashboard.
# No PyBullet viewer. Stop: bash $PAC_WORKCELL_REPO/scripts/stop_review_v44.sh (+ Ctrl+C here).
#   bash workcell/run_gazebo_demo.sh DATASET SCENARIO [COUNT]
#   PAC_ENABLE_CLAUDE=1  dashboard answers with the Claude API (key prompt, never stored)
set -eo pipefail
LYL="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WC="${PAC_WORKCELL_REPO:-$(cd "$LYL/../pac2026_integrated" && pwd)}"
DATASET="${1:?usage: run_gazebo_demo.sh DATASET SCENARIO [COUNT]}"; SCEN="${2:?scenario}"; COUNT="${3:-0}"
PORT="${PAC_DASHBOARD_PORT:-4183}"
source /opt/ros/humble/setup.bash
source "$WC/ros2_ws/install/setup.bash"
set -u
export PAC_WORKCELL_REPO="$WC" IGN_IP="${IGN_IP-127.0.0.1}"
export PAC_LAYOUT=v46 PAC_SCALE_SHIFT_M=-2.185 PAC_NO_WRIST=1 PAC_BUFFER_X=-1.05 PAC_BUFFER_Y=0.44 PAC_BUFFER_DEPTH=0.66 \
       PAC_BUFFER_EXT_DX=0.90
if [[ "${USE_NVIDIA:-1}" == 1 && -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]]; then
  export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
         __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
fi
LOG="$WC/logs/lyl_demo/$(date +%Y%m%d_%H%M%S)"; mkdir -p "$LOG"
WORLD=ahead_workcell_v4_2_physical_scale
BR="$(ros2 pkg prefix ros_gz_bridge)/lib/ros_gz_bridge/parameter_bridge"

echo ">>> [1/6] 이전 시뮬레이션 정리"
bash "$WC/scripts/stop_review_v44.sh" >/dev/null 2>&1 || true

echo ">>> [2/6] Gazebo V4.6 작업셀 (깊이 카메라, 버퍼 4칸)"
setsid nohup ros2 launch pac_bringup hdr50_workcell_v4_6.launch.py > "$LOG/gazebo.log" 2>&1 < /dev/null &
for _ in $(seq 60); do ign topic -l 2>/dev/null | grep -qx /pac/gripper/state && break; sleep 1; done
for _ in $(seq 40); do timeout 5 ros2 control list_controllers 2>/dev/null | grep -q 'trajectory.*active' && break; sleep 1; done
timeout 5 ros2 control list_controllers 2>/dev/null | grep -q 'trajectory.*active' \
  || { echo "ERROR: robot controllers not active ($LOG/gazebo.log)" >&2; exit 1; }

echo ">>> [3/6] MoveIt2 + RViz"
setsid nohup ros2 launch pac_bringup hdr50_moveit_v44.launch.py > "$LOG/moveit.log" 2>&1 < /dev/null &
for _ in $(seq 60); do grep -q 'Ready to take commands' "$LOG/moveit.log" && break; sleep 1; done
grep -q 'Ready to take commands' "$LOG/moveit.log" || { echo "ERROR: MoveIt not ready ($LOG/moveit.log)" >&2; exit 1; }
# work cell collision scene in MoveIt before the first box, arm to READY (above PICK)
if ! (cd "$WC/scripts" && python3 -u moveit_prepare_v46.py) 2>&1 | tee "$LOG/moveit_prepare.log"; then
  echo "ERROR: MoveIt environment/READY preparation failed ($LOG/moveit_prepare.log)" >&2
  exit 1
fi

echo ">>> [4/6] 센서·카메라 브리지, CCTV 박스 인식 노드"
setsid "$BR" '/pac/scale/wrench@geometry_msgs/msg/WrenchStamped[ignition.msgs.Wrench' \
  '/pac/conveyor/roller_cmd_vel@std_msgs/msg/Float64]ignition.msgs.Double' \
  '/pac/scale_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image' \
  '/pac/scale_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo' \
  "/world/$WORLD/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V" \
  '/pac/top_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image' \
  '/pac/scale_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image' > "$LOG/bridge.log" 2>&1 &
setsid python3 -u "$WC/scripts/box_perception_v44.py" > "$LOG/perception.log" 2>&1 &

echo ">>> [5/6] 적재 실행기 시작 (주문 $SCEN)"
MANIFEST="$WC/logs/pac_dashboard_integration/active_run.json"
before="$(stat -c %y "$MANIFEST" 2>/dev/null || echo missing)"
python3 -u "$LYL/workcell/lyl_cell_runner.py" --dataset "$DATASET" --scenario "$SCEN" --count "$COUNT" \
  2>&1 | tee "$LOG/runner.log" &
RUNNER=$!
for _ in $(seq 60); do
  [[ "$(stat -c %y "$MANIFEST" 2>/dev/null || echo missing)" != "$before" ]] && break
  kill -0 "$RUNNER" 2>/dev/null || break
  sleep 1
done
if [[ "$(stat -c %y "$MANIFEST" 2>/dev/null || echo missing)" == "$before" ]]; then
  echo "ERROR: 현재 실행 등록을 확인하지 못했습니다. 이전 대시보드를 새 실행으로 열지 않습니다 ($LOG/runner.log)." >&2
  # Preserve a running robot task; report failure without silently abandoning it.
  if wait "$RUNNER"
  then exit 1
  else exit "$?"
  fi
fi

echo ">>> [6/6] API 연동 대시보드 V5.3 (http://127.0.0.1:$PORT)"
for p in $(pgrep -f 'pac_diagnostic_console.py serve|pac_ros_observer.py run|pac_camera_bridge_watch.py'); do kill "$p" 2>/dev/null || true; done
sleep 1
setsid nohup bash "$LYL/tools/pac_failure_analysis_dashboard/start_dashboard_integrated.sh" "$WC" \
  > "$LOG/dashboard.log" 2>&1 < /dev/null &
DASHBOARD_PID=$!
dashboard_status=1
for _ in $(seq 30); do
  if curl --max-time 2 -sf "http://127.0.0.1:$PORT/api/state" >/dev/null; then
    dashboard_status=0
    break
  fi
  kill -0 "$DASHBOARD_PID" 2>/dev/null || break
  sleep 1
done
if [[ "$dashboard_status" == 0 ]]; then
  [[ "${NO_BROWSER:-0}" == 1 ]] || (xdg-open "http://127.0.0.1:$PORT" >/dev/null 2>&1 &)
else
  echo "ERROR: 대시보드 시작 실패 ($LOG/dashboard.log). 적재 실행 결과를 기다립니다." >&2
fi
if wait "$RUNNER"; then
  if [[ "${dashboard_status:-0}" != 0 ]]; then
    echo "ERROR: 적재 실행은 종료됐지만 대시보드 시작에 실패했습니다 ($LOG/dashboard.log)." >&2
    exit "$dashboard_status"
  fi
  echo "RUN DONE: Gazebo / RViz / dashboard stay up. Logs $LOG"
else
  runner_status=$?
  echo "ERROR: runner failed (exit $runner_status). Gazebo / RViz / dashboard stay up. Logs $LOG" >&2
  exit "$runner_status"
fi
