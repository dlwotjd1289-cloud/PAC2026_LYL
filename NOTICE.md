# 출처

이 저장소는 PAC 2026 Mission 1 팀 저장소(`dlwotjd1289-cloud/dlwotjd1289-cloud`)에서
우리 알고리즘을 실행하는 데 필요한 부분만 옮겨 정리한 것입니다.

| 부분 | 담당 | 원본 |
|---|---|---|
| 데이터 생성기 `tools/ahead_dataset_generator` | 재성 | 팀 `main` (v1.4.0, `730ec44`) |
| 깊이 인식 `ros2_ws/src/pac_perception` | 동한 (관측 생성기: 재성) | 팀 `main` (`7f5a63d`) |
| 런타임·후보·상위결정·로봇검사 `pac_runtime`, `pac_candidates`, `pac_highlevel`, `pac_robot_check`, `tools/virtual_data` | 태현 | 통합 묶음 `integration/jaesung-taehyeon-algorithm` (`6f12962`) |
| 비선형 평가 `tools/algorithm_integration`, `pac_highlevel/merged_placement.py` | 재성 | 같은 통합 묶음 |
| 공통 계약 `pac_common`, `config/default.yaml` | 공통 | 같은 통합 묶음 |
| 실패 분석·Claude·3D 대시보드 `tools/pac_failure_analysis_dashboard` (V5.3) | 재성 | 팀 PR #12 `feature/gazebo-claude-failure-analysis-dashboard-v53` (`f0a171a`). `start_dashboard_integrated.sh`의 ROS 환경 로딩만 수정(`set -u` 버그) |
| 파이프라인 연결 `pipeline/`, `config/pipeline.yaml`, `run_dashboard.sh` | 이 저장소에서 새로 작성 | — |
| Gazebo 연결 `workcell/`, `config/workcell_v46.yaml` | 이 저장소에서 새로 작성. V4.6 작업셀 수정은 `workcell/v46_patch/` | V4.6 최종 `jaesung/v46-final-3d-20261010` (`eba65a2`) 기준 |

원본 알고리즘 코드 변경: `pac_runtime/loop.py` 주입 지점 2개, `pac_runtime` 7초 초과 시 계산된 명령 사용(설정),
`pac_highlevel` 버퍼 칸 크기 제한(설정, 기본 끔). 통합 프로필 설정값 변경과 이유는
[docs/05_worklog_20261010.md](docs/05_worklog_20261010.md)에 있습니다.

동한님 구현은 `donghan7298-code/pac2026-pattern-evaluation` 프로젝트와 팀 공통 v0.2 계약을
바탕으로 재구성한 것입니다.

라이선스는 팀이 정하지 않았으므로 별도 오픈소스 라이선스를 부여하지 않았습니다.
