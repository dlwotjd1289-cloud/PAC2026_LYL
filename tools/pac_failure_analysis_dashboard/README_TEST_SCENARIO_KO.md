# PAC2026 V5.1 — 안전한 MoveIt 실패 진단 시나리오

## 핵심 목표

```
의도적으로 도달 불가능한 IK 목표 설정
  → MoveIt /compute_ik 서비스 결과 확인 (궤적 명령 없음)
  → 테스트 전용 로그 작성
  → 대시보드 TEST ONLY 실패 알람 표시
  → 하단 자연어 질문에 근거 포함 JSON 응답
```

기존 `Gazebo`, `MoveIt`, `RViz2`와 적재 알고리즘, 작업 중인 로그를 **수정하거나 종료하지 않습니다.**
이번 실제 연동 시험은 **MoveIt IK 계산 거절**을 검증합니다. 실제 박스의 픽앤플레이스 실패, 물리적 충돌, 컨트롤러 실행 실패를 재현하는 것이 아닙니다. 

`--mode sample`은 **합성 실패 로그**를 쓰는 별도 스모크 테스트입니다. ROS/MoveIt 호출이 전혀 없습니다. UI에 `TEST ONLY`가 표시됩니다.

## 준비

이미 실행 중인 V5 포트 `4178`은 유지해도 됩니다. V5의 관측기 `telemetry.json`을 읽고, 실패 로그만 **테스트 폴더**를 선택하는 별도 포트 `4180`을 사용합니다.

```bash
cd ~/AHEAD/pac2026_integrated
mkdir -p tools/pac_gazebo_3d_v5_1_test
unzip -q "$(xdg-user-dir DOWNLOAD)/PAC2026_GAZEBO_3D_DASHBOARD_V5_1_TEST.zip" \
  -d tools/pac_gazebo_3d_v5_1_test
```

## A. 먼저 가상 로그로 UI 전체 흐름 점검

**터미널 1** (V5/Gazebo 없이도 가능):

```bash
cd ~/AHEAD/pac2026_integrated
python3 tools/pac_gazebo_3d_v5_1_test/scenario_fail_ik.py \
  --repo "$PWD" --mode sample
```

`mode: sample`, `TEST_IK_SAMPLE_...`과 `robot_command_sent: false`가 보이면 정상입니다. 이 테스트에는 **실제 MoveIt 응답이 없습니다.**

**터미널 2**:

```bash
cd ~/AHEAD/pac2026_integrated
bash tools/pac_gazebo_3d_v5_1_test/start_test_dashboard.sh "$PWD"
```

웹: **http://127.0.0.1:4180**. 우측에 테스트 알람과 하단 질문창이 표시됩니다.

**터미널 3**:

```bash
python3 ~/AHEAD/pac2026_integrated/tools/pac_gazebo_3d_v5_1_test/check_test_dashboard.py
```

5개 `PASS`가 확인돼야 합니다. 카메라·로봇 실측은 V5 관측기가 별도로 실행 중일 때만 추가로 표시됩니다.

## B. 실제 MoveIt 서비스의 도달 불가능 IK 요청

아래 시험은 **실제 ROS 2 + MoveIt이 실행 중인 사용자 PC에서만** 가능합니다. 기존 Gazebo 작업셀과 MoveIt 2를 정상 실행하되, 별도로 박스 투입·로봇 움직임을 시작할 필요는 없습니다.

```bash
cd ~/AHEAD/pac2026_integrated
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export IGN_IP=127.0.0.1

ros2 service list | grep -x /compute_ik
```

`/compute_ik`가 없으면 MoveIt 프로세스가 실제로 준비됐는지 확인합니다. 서비스를 찾지 못하면 실패 로그를 지어내지 않습니다.

**터미널 1**에서 실행:

```bash
python3 tools/pac_gazebo_3d_v5_1_test/scenario_fail_ik.py \
  --repo "$PWD" --mode moveit-ik
```

이 스크립트는 `hdr_manipulator`의 `suction_tcp`에 대해 `world` 좌표 `(25,25,25) m`로 `GetPositionIK`를 **한 번** 요청합니다. 로봇 이동/궤적 실행/시뮬레이터 상태 변경 명령은 보내지 않습니다. `error_code != 1` 응답을 받으면 받은 **실제 오류 코드 그대로**를 테스트 기록에 남깁니다. `error_code=1`이거나 API 호출에 실패하면 실패 로그를 만들지 않고 중단합니다.

`run_dir` 출력 예시:

```text
.../logs/pac_dashboard_test_scenarios/TEST_IK_MOVEIT_IK_20261010_XXXXXX_abcdef
```

테스트 서버가 `sample`을 먼저 선택했다면 해당 서버 터미널에서만 `Ctrl+C`로 종료하고 다시 실행하세요. 가장 최근 테스트 폴더가 자동 선택됩니다. 확실하게 선택하려면 두 번째 인수로 출력된 `run_dir`를 지정합니다.

```bash
bash tools/pac_gazebo_3d_v5_1_test/start_test_dashboard.sh "$PWD" "<출력된 run_dir 전체 경로>"
```

**터미널 3**에서 실제 ROS 응답인지까지 검사:

```bash
python3 tools/pac_gazebo_3d_v5_1_test/check_test_dashboard.py --require-real
```

확인할 출력:

- `TEST ONLY` 알람과 `MOVEIT IK FAIL` 원문
- `failure_stage = moveit_ik`
- 자연어 질문 `이 테스트의 마지막 실패 원인이 뭐야?`에 **근거 ID가 붙은 정형 답변**
- `control.command_executed = false`
- `--require-real` 검사 5개 PASS

**중요:** `error_code=-31 (NO_IK_SOLUTION)`은 일반적인 예상값일 뿐, 기종·설정에 따른 실제 결과는 받은 코드로 판정해야 합니다.

## 출력 위치와 실제 로그 격리

```
logs/pac_dashboard_test_scenarios/TEST_IK_.../
  box_TEST_moveit.log        # 테스트 표시된 원문
  test_manifest.json         # mode, /compute_ik, 목표, 결과, 실행 금지 기록
```

기존 `logs/v44_generator_cycle/...` 파일은 수정하지 않습니다. **V5 기존 4178번 화면은 테스트 폴더를 자동으로 선택하지 않습니다.** 테스트가 끝나면 `4180` 화면의 터미널만 종료하면 됩니다.

## 오류 기준과 성공 기준

- `--mode sample` 성공: **대시보드/UI/오프라인 답변 파이프라인 작동** 검증 (ROS 연동 아님).
- `--mode moveit-ik` 성공: **실제 MoveIt IK 계산의 실패 응답 → 로그 → 대시보드 알람 → 진단 답변** 검증 (실제 로봇 이동·픽앤플레이스 아님).
- 실제 Gazebo 픽앤플레이스 오류 자동 복구 검증은 별도의 후속 단계가 필요합니다.
- `[camera] coarse pose error ...`를 오류 알람으로 오인하지 않도록 추가 분류 예외를 적용했습니다.

## Claude API 연동 예정 사항

이 V5.1 테스트는 **API 키 없이 실행**합니다. 현재 대시보드 원본의 외부 LLM 코드는 OpenAI용입니다. Claude API 연동은 별도 구현이 필요합니다.

예정 구조: `상태/로그 수집(동일) → Claude API 연결 어댑터 → 정해진 JSON 응답 → 서버의 스키마·근거 ID 재검증 → 대시보드 표시`.

Claude 공식 문서의 `output_config.format`은 JSON Schema 형식의 구조화된 출력을 지원합니다. 단, 현 V5의 출력 스키마에 있는 `maxLength`, `maxItems` 등은 Claude의 제공자 측 지원 스키마 제한에 맞춰 변환해야 하며, **서버 내부 검증 규칙은 더 엄격하게 유지**해야 합니다. (https://platform.claude.com/docs/ko/build-with-claude/structured-outputs)

이 번들에서는 Claude API 키 입력·외부 호출을 하지 않습니다.
