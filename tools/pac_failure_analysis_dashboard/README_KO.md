# PAC2026 Gazebo 3D 통합 연동 준비형 V5.3

**최신 설치·통합 안내: [INTEGRATION_GUIDE_KO.md](INTEGRATION_GUIDE_KO.md)**

이 버전은 Gazebo/MoveIt 재실행 자동 재탐색, 활성 실행별 로그, 구역별 박스 관측과 실행기 이벤트의 분리, Claude API 읽기 전용 답변을 지원합니다. 기존 V5.2.1은 덮어쓰지 않습니다. 실행 포트는 `4183`이며, 시작 스크립트는 `start_dashboard_integrated.sh`입니다. **최종 알고리즘/Gazebo 실환경 통합 검증은 아직 수행하지 않았습니다.**

---

## 이하: 이전 V5 관련 안내(기능 배경 참고용)

> **V5.2 안내:** 이 문서는 이전 V5 버전의 사용 설명을 보존한 것입니다. 현재 Claude API 연결 및 실행은 `CLAUDE_SETUP_KO.md`와 `start_dashboard_claude.sh`를 우선하세요.

# V5.1 TEST 전용: 먼저 README_TEST_SCENARIO_KO.md를 읽으세요

# PAC2026 Gazebo 3D Dashboard V5 — 채팅·모델·카메라·지표 검증

## 핵심 변화

1. **질문 입력창을 화면 하단에 고정**했습니다. ChatGPT처럼 질문/답변 버블과 이전 대화를 표시합니다. `Enter` 전송, `Shift+Enter` 줄바꿈, 새 대화, 추천 질문이 가능합니다.
2. **API 키 없이도 질문 가능**합니다. 실패 로그·로봇 관절·Gazebo 박스 위치·카메라·수신 경과·선택 실행 기록을 **제한된 로컬 규칙 기반**으로 답하고 동일한 고정 JSON 구조를 반환합니다. 자유로운 대화형 AI는 구현하지 않았습니다. 입력 문장이 로봇 제어 명령이어도 실행되지 않습니다.
3. 3D 렌더러가 **원본 HDR50-22 STL과 URDF**를 Three.js에서 직접 표시하도록 개선했습니다. STL 메시당 1,250개 삼각형만 읽는 기존 Plotly 방식보다 부드러운 형상·조명 표현을 목표로 합니다. 실시간 관절값은 ROS 2에서, 박스 위치는 Gazebo에서 읽습니다.
4. **청회색 배경**으로 밝기를 높이고 3D 주화면을 유지했습니다.
5. 카메라 2대를 위한 **Gazebo → ROS 2 `ros_gz_bridge` 읽기 전용 영상 중계**를 실행 스크립트가 시작합니다. Gazebo 영상 토픽이 실제로 있고, 사용 중인 ROS 영상 발행자가 없을 때만 실행합니다. 실행한 브리지의 PID만 세션 종료 시 정리합니다.
6. `CAMERA JPEG`는 토픽 존재가 아니라 **최신 JPEG 파일**을 확인해 카운트합니다. `DATA LATENCY`는 오해를 막기 위해 `LAST DATA AGE`로 바꿨고, `GAZEBO OBJECTS`는 **TRACKED POSES**로 바꿨습니다. 자세한 신뢰성 검토는 `METRIC_AUDIT_KO.md`를 읽으세요.

## 현재 확인된 한계

- 개발 컨테이너에서는 WebGL이 제한되고 외부 CDN이 차단되어 **Three.js가 네트워크로 로드되는 환경의 최종 시각 품질을 검증하지 못했습니다**. 실제 Ubuntu Firefox에서 추가 검증이 필요합니다. Three.js / OrbitControls / STLLoader는 기존 PyBullet 웹 뷰어와 같은 CDN 라이브러리를 로드합니다. **인터넷 또는 CDN에 접속하지 못하면 번들 Plotly/Canvas 3D로 자동 전환**되며 이때는 로봇 외형이 거칠거나 중심선으로 보일 수 있습니다. 3D 화면에 `Three.js` 대신 `Canvas 간략형`이 보이면 해당 상황입니다.
- 카메라 2대 영상의 실제 수신은 **사용자 Gazebo PC에서 아직 검증되지 않았습니다.** 브리지 설치, ROS 2 Image 수신, `cv2` JPEG 변환이 모두 성공해야 화면에 나옵니다. 0/2면 영상을 성공적으로 받지 못한 것이며 가짜 영상을 보여주지 않습니다.
- 화면을 30/60FPS 렌더링할 수 있어도 ROS 관절과 Gazebo 위치는 관측기 수신 빈도로 갱신됩니다. 중간의 로봇 동작을 만들어내지 않습니다.
- 이 프로젝트에는 제어 요청 엔드포인트가 없습니다. Gazebo/MoveIt/로봇 명령·중지·초기화에 관여하지 않습니다.
- 구 버전 V4와는 **별도 폴더·별도 포트**를 사용합니다. 기존 알고리즘/메인 브랜치는 수정하지 않습니다.

## 실행 (Gazebo V4.6와 MoveIt이 켜진 상태)

이전 V4 대시보드 세션만 해당 터미널에서 `Ctrl+C`로 종료하세요. **Gazebo, MoveIt, RViz는 종료하지 마세요.**

```bash
cd ~/AHEAD/pac2026_integrated
mkdir -p tools/pac_gazebo_3d_v5
unzip -q "$(xdg-user-dir DOWNLOAD)/PAC2026_GAZEBO_3D_DASHBOARD_V5.zip" \
  -d tools/pac_gazebo_3d_v5
bash tools/pac_gazebo_3d_v5/start_dashboard_v5.sh "$PWD"
```

접속: **http://127.0.0.1:4178**

터미널 진단:

```bash
cd ~/AHEAD/pac2026_integrated
python3 tools/pac_gazebo_3d_v5/check_v5.py
```

API 없이 터미널 질문하기:

```bash
python3 tools/pac_gazebo_3d_v5/pac_diagnostic_console.py ask \
  --url http://127.0.0.1:4178 '마지막 적재가 왜 실패했어?'

python3 tools/pac_gazebo_3d_v5/pac_diagnostic_console.py chat \
  --url http://127.0.0.1:4178
```

원본 JSON을 보려면 `--json`을 추가하세요.

## 카메라가 0/2인 경우 점검

실행 스크립트는 **센서 토픽을 읽어 ROS 토픽으로 중계할 뿐**, 로봇을 제어하지 않습니다.

```bash
cd ~/AHEAD/pac2026_integrated
ign topic -l | grep -E '/pac/(scale_camera|top_camera)/image'
ros2 topic list -t | grep -E '/pac/(scale_camera|top_camera)/image'
tail -30 logs/pac_dashboard_3d_v5/camera_bridge.log
tail -30 logs/pac_dashboard_3d_v5/observer.log
python3 tools/pac_gazebo_3d_v5/check_v5.py
```

예상 Gazebo 토픽: `/pac/scale_camera/image`, `/pac/top_camera/image`.

브리지를 다른 프로세스가 이미 실행하는 경우 기존 ROS 발행자를 사용합니다. 자동 브리지를 끄려면 `PAC_DASHBOARD_CAMERA_BRIDGE=0`을 지정해서 실행할 수 있습니다. 카메라 영상 처리 부하가 높은 경우에도 이 옵션을 활용하세요.

## 성능 및 화면 관련 참고

- `Three.js` 모드의 로봇 모델은 기존 HDR50-22 원본 STL을 로컬 HTTP로 읽습니다. 원본 STL 파일 자체는 ZIP에 포함하지 않습니다.
- Plotly/Canvas 대체 모드에서 보이는 중심선은 상세 외형이 아닙니다. 어떤 모드로 동작하는지는 화면 좌측 장면 상태 메시지에서 확인합니다.
- WebGL 모드에서는 원본 STL의 위치별 법선을 평균하여 조명을 부드럽게 처리합니다. **이건 시각화 보정**으로 실제 물체 위치나 로봇 제어값에는 영향을 주지 않습니다.
- 3D 배경은 로컬 V4.6 SDF의 정적 visual, 관절은 `/joint_states`, 동적 박스는 Gazebo Pose입니다. 시뮬레이터 GUI의 전체 프레임을 화면복제하는 기능은 아닙니다.

## 검증

```bash
cd ~/AHEAD/pac2026_integrated/tools/pac_gazebo_3d_v5
python3 -m pytest -q
```

개발 컨테이너에서 모든 자동 테스트 및 **가상 데이터로 수행한 브라우저 폴백 화면 테스트**를 완료했습니다. 테스트 성공은 사용자 PC의 GPU WebGL, Gazebo 카메라 브리지, 실제 모델 렌더링 성공을 증명하지 않습니다.

안전·신뢰성 문서: `METRIC_AUDIT_KO.md`.
