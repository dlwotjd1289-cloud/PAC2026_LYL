> **V5.3에서는 최신 `start_dashboard_integrated.sh` (포트 4183)를 사용하세요.** Claude API 키는 `PAC_ENABLE_CLAUDE=1`로 활성화합니다. 아래는 이전 V5.2 절차 기록입니다.

# PAC2026 V5.2 Claude API 연결 가이드 (2026-10-10)

## 무엇이 바뀌었나

기존 PAC V5.1 대시보드의 OpenAI 전용 LLM 요청부를 Anthropic Claude Messages API로 추가했다. 3D·ROS·Gazebo 관측, 오류 로그 수집, 카메라 표시, 오프라인 질문 답변은 그대로 유지한다. 실제 로봇을 조종하는 기능은 없다.

- 기본 API 모델: `claude-haiku-5-5` (2026-10-10 당시 유효한 Anthropic 모델 ID)
- 선택 모델: `claude-sonnet-5-5`
- 통신: `POST https://api.anthropic.com/v1/messages`, `anthropic-version: 2023-06-01`, `x-api-key`
- 고정 응답: `output_config.format.type=json_schema`, `MODEL_SCHEMA`에 맞는 결과
- Anthropic 요청에서는 미지원 `maxLength`·`maxItems` 제약을 제외한다. **서버의 원본 모델 검증에는 그대로 유지**한다.
- `source=claude`로 표시하며 `confirmed_facts`는 실제 컨텍스트에 존재하는 근거 ID만 채택한다.
- API에 입력으로 전송되는 내용은 사용자 질문, 선택된 실행 기록 및 읽기 전용 Gazebo/ROS 관측 요약, 현재 대화의 제한된 문맥이다. 로봇 명령은 호출하지 않는다.

## 먼저 키 발급

1. https://platform.claude.com/ (Claude Console) 로그인
2. https://platform.claude.com/settings/keys 에서 API 키 생성
3. 크레딧·사용량 한도 확인
4. 키는 채팅, Git, 스크린샷, 공유 로그, `.env` 공개 저장소에 넣지 않는다.

Claude 채팅 앱 구독료와 개발자 API 사용 요금은 별개이다.

## 설치

프로젝트 위치가 `~/AHEAD/pac2026_integrated`이고 ZIP을 다운로드 폴더에 받았다면:

```bash
cd ~/AHEAD/pac2026_integrated
mkdir -p tools/pac_gazebo_3d_v5_2_claude
unzip -q "$(xdg-user-dir DOWNLOAD)/PAC2026_GAZEBO_3D_DASHBOARD_V5_2_CLAUDE.zip" \
  -d tools/pac_gazebo_3d_v5_2_claude
```

V5/V5.1과 별도 폴더·별도 포트(4181)에서 실행하므로 이전 코드를 덮어쓰지 않는다. Gazebo와 MoveIt을 종료하거나 재시작하지 않는다.

## API 없이 오프라인으로 먼저 켜기

Gazebo/MoveIt은 이미 켜져 있어야 실측 정보가 표시된다.

```bash
cd ~/AHEAD/pac2026_integrated
bash tools/pac_gazebo_3d_v5_2_claude/start_dashboard_claude.sh "$PWD"
```

브라우저: http://127.0.0.1:4181

`PAC_ENABLE_CLAUDE=1`을 지정하지 않으면 환경에 키가 있어도 **Claude API를 호출하지 않는다.**

## API 키 활성화 (키를 입력할 때 화면에 나타나지 않는 방식, 권장)

V5.2를 오프라인으로 실행 중인 터미널에서는 `Ctrl+C`로 V5.2만 종료한다. Gazebo·MoveIt은 건드리지 않는다.

```bash
cd ~/AHEAD/pac2026_integrated
PAC_ENABLE_CLAUDE=1 bash tools/pac_gazebo_3d_v5_2_claude/start_dashboard_claude.sh "$PWD"
```

터미널에 `Claude API 키 (입력 숨김):`이 나타나면 키를 붙여넣고 Enter를 누른다. 입력 문자열은 화면에 표시되지 않고 디스크에 기록되지 않는다. 브라우저 http://127.0.0.1:4181 접속 후 하단 입력창에서 질문한다. 오프라인 사용으로 돌아가려면 V5.2만 종료하고 위 오프라인 명령으로 다시 시작한다.

### 환경변수로 입력하는 방식 (선택)

같은 셸에서 `read -s`로 받고, API를 실행하면 된다.

```bash
read -r -s -p 'Claude API key: ' ANTHROPIC_API_KEY; echo
export ANTHROPIC_API_KEY
export PAC_ENABLE_CLAUDE=1
bash tools/pac_gazebo_3d_v5_2_claude/start_dashboard_claude.sh "$PWD"
unset ANTHROPIC_API_KEY PAC_ENABLE_CLAUDE
```

가능하면 키 보관 대신 프로그램의 **숨김 입력**을 사용한다. 환경변수는 해당 프로세스의 자식에게 전달될 수 있다.

## AI 모델 변경

기본은 `claude-haiku-5-5`. 더 깊은 분석이 필요하면 시작 전에:

```bash
export ANTHROPIC_MODEL=claude-sonnet-5-5
PAC_ENABLE_CLAUDE=1 bash tools/pac_gazebo_3d_v5_2_claude/start_dashboard_claude.sh "$PWD"
unset ANTHROPIC_MODEL
```

## 키 자체가 정상인지 시험 (선택; 1회 소량 과금)

```bash
python3 tools/pac_gazebo_3d_v5_2_claude/test_claude_key.py
```

이 테스트는 키를 숨김 입력받아 Claude에 **짧은 테스트 문장만** 전송한다. 로봇·작업셀 로그는 보내지 않는다. 별도 API 요금이 발생할 수 있다.

## 테스트 질문

브라우저에서 입력:

- `현재 선택된 실행에서 마지막 실패 기록을 근거 ID와 함께 요약해줘.`
- `실패 원인이 확정인지 추정인지 분리하고 추가로 확인할 항목을 알려줘.`
- `Gazebo의 관측 상태와 MoveIt 로그가 다른 정보를 말한다면 구분해서 설명해줘.`
- `박스를 팔레트에 올려줘.` → 명령 실행 금지, 읽기 전용 답변만.

터미널 질문도 가능:

```bash
python3 tools/pac_gazebo_3d_v5_2_claude/pac_diagnostic_console.py ask \
  --url http://127.0.0.1:4181 '마지막 실패는 무엇이야?'
```

질문은 모든 서버 요청에서 같은 `REPLY_SCHEMA`로 검증된다. 성공했어도 AI 진단은 안전 판정이나 실제 로봇 완료 판정이 아니다.

## 문제 해결

- **키가 없는데 API가 안 됨**: 정상이다. 기본은 로컬 질문만 지원한다.
- **401**: 키 오류 또는 다른 Workspace의 키. 키 재발급/권한 확인.
- **402**: API 크레딧·결제 설정 확인.
- **403**: 프로젝트/Workspace 접근 권한 확인. 여러 Workspace에 걸친 키라면 `ANTHROPIC_WORKSPACE_ID`를 환경변수로 지정할 수 있다.
- **404**: 모델 ID 사용 가능 여부 확인. 기본은 `claude-haiku-5-5`.
- **429**: 속도 제한 또는 사용량 한도 확인.
- **invalid_response**: 메시지가 불완전하거나 출력 스키마·근거 ID 검증에 실패. 화면에는 채택되지 않는다.
- **카메라/3D 안 보임**: API 문제가 아니라 ROS/Gazebo 관측/브리지 문제. `logs/pac_dashboard_3d_v5/observer.log`, `camera_bridge.log` 확인.
- **대시보드 끊김**: API 요청은 비동기이며 로봇 관측과 별도로 동작. 관측 주기/브라우저 렌더링은 이전 V5 설정을 유지한다.

## 검증 및 제한

- 기존 테스트 + Claude용 모의 API 추가 자동 테스트: `pytest -q`로 실행 가능.
- **실제 Claude 연결 성공은 API 키가 없어서 이 개발 환경에서는 아직 미검증이다.** 사용자의 터미널에서 `test_claude_key.py`를 실행하여 확인해야 한다.
- API 키가 포함된 화면, HTTP 헤더, 전체 터미널 로그는 채팅에 공유하지 않는다. 오류 코드와 마스킹된 상황만 보내도 된다.
- API를 활성화하면 질문과 관측/로그 데이터 일부가 Anthropic 서버로 전송된다. 외부 전송이 허용되는 데이터만 사용한다.

## 공식 문서

- https://platform.claude.com/docs/en/api/overview
- https://platform.claude.com/docs/en/build-with-claude/structured-outputs
- https://platform.claude.com/docs/en/models/haiku-5-5/overview
- https://platform.claude.com/docs/en/models/sonnet-5-5/overview
