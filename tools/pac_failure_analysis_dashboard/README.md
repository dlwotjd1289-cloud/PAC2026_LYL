# PAC 2026 — Gazebo 실패 분석 · Claude API 연동 대시보드 (V5.3)

**팀 프로젝트용 독립 실행(Read-only) 진단 대시보드**. Gazebo Fortress + ROS 2 Humble + MoveIt 2 환경에서 로봇 관절, Gazebo 박스 위치, 고정 카메라 및 실행 로그를 관찰합니다. Claude Messages API를 이용해 자연어 질문에 답할 수 있으며 고정 JSON 구조와 근거 ID를 서버에서 검증합니다.

> **상태:** 코드 및 모의 입력 자동 테스트 121개 통과(게시 전 로컬 실행 기준). 실제 팀 통합 실행기와의 종단 간 연결은 미검증. 이 프로그램은 로봇 안전계통을 대체하지 않으며 로봇에 동작 명령을 보내지 않습니다.

## 기능

- Gazebo 실행 시작/종료·월드 전환을 감지하고, 지정된 월드의 위치 데이터를 읽습니다.
- ROS 2 `/joint_states`에 따라 HDR50-22 모델을 관찰하고, 정적 작업셀을 SDF에서 읽습니다.
- 팔레트·컨베이어·버퍼·NG 구역을 **좌표 기반으로 분류**합니다. 실제 적재 완료는 실행기의 이벤트와 별도로 대조합니다.
- 최신 카메라 JPEG가 있는 경우에만 영상 수신으로 표시하며 미수신을 정상으로 위장하지 않습니다.
- 현재 실행의 `log_dir`에서 실패 근거를 수집하고 Claude로 분석할 수 있습니다. Claude 키가 없으면 제한적인 로컬 질문 응답을 제공합니다.
- 같은 시뮬레이션의 실제 위치·완료 기록 불일치, 오래된 텔레메트리, 다중 월드 혼동을 경고합니다.

## 설치·실행 (저장소 루트에서)

```bash
cd ~/AHEAD/pac2026_integrated
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

# 선택: Plotly fallback을 오프라인에서 사용하고 싶으면 한 번 다운로드
bash tools/pac_failure_analysis_dashboard/fetch_plotly_vendor.sh

# API 키 없이 진단
bash tools/pac_failure_analysis_dashboard/start_dashboard_integrated.sh "$PWD"
# Claude API를 활성화하려면 (키는 터미널에서 숨김 입력)
PAC_ENABLE_CLAUDE=1 bash tools/pac_failure_analysis_dashboard/start_dashboard_integrated.sh "$PWD"
```

웹 주소: **http://127.0.0.1:4183** (`PAC_DASHBOARD_PORT`로 변경 가능).

Claude API 키는 `ANTHROPIC_API_KEY` 환경변수 또는 서버 시작 시 숨김 입력으로 전달합니다. **키, 실행 로그, 실제 텔레메트리 파일을 Git에 커밋하지 마세요.** `.gitignore`에 기본 차단 규칙이 포함되어 있습니다.

Plotly 3.3.1은 소스 저장소 용량을 고려해 미포함했습니다. 파일이 없으면 공식 CDN으로 리다이렉트하며, 인터넷이 제한된 환경은 `fetch_plotly_vendor.sh`로 미리 설치하십시오. Three.js와 OrbitControls/STLLoader 역시 웹에서 외부 CDN을 사용합니다. 사용 라이브러리 저작권: `THIRD_PARTY_NOTICES.txt`.

## 최종 알고리즘과 연결

실행기는 `register_active_run()`으로 `logs/pac_dashboard_integration/active_run.json`을 생성하고, `report_event()`로 배치/버퍼/NG 결과를 기록합니다. 자세한 JSON 예시와 통합 절차는 [INTEGRATION_GUIDE_KO.md](INTEGRATION_GUIDE_KO.md)를 확인하세요.

> 박스의 위치가 해당 구역에 들어왔다는 사실만으로 적재 완료나 NG 판정을 확정하지 않습니다. 로봇 실행기의 이벤트·관측 좌표·허용 기준을 따로 확인합니다.

## 검사

```bash
cd tools/pac_failure_analysis_dashboard
python3 -m pytest -q
python3 check_integrated_v53.py --repo ~/AHEAD/pac2026_integrated
```

기존 Gazebo·MoveIt 프로세스는 이 패키지가 시작하거나 종료하지 않습니다. **본 브랜치에서 `main`과 팀원 알고리즘 코드는 수정하지 않았습니다.**
