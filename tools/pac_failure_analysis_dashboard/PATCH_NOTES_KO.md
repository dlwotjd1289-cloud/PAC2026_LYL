# Claude API 45초 응답 제한 수정 V5.2.1

기존 V5.2 코드를 확인하고 제한시간 및 예외 분류만 변경했습니다. 로봇·Gazebo·MoveIt 실행 코드는 수정하지 않았습니다.

## 수정 사항

1. `pac_diagnostic_console.py`: Claude aiohttp 총 요청시간 45초 → **120초**.
2. `web/dashboard.html`: 브라우저 질문 요청 중단 55초 → **135초**. 서버보다 길게 설정.
3. 터미널 `ask/chat` HTTP 요청 대기 60초 → **150초**.
4. 오류 종류 분리: 실제 시간 초과는 경과 초 단위와 제한시간을 표시, 네트워크·SSL 오류는 예외 클래스 이름만 노출. 키·질문·서버 응답 본문은 로그로 출력하지 않음.
5. 실행 포트 기본값 4182(V5.2의 4181과 분리).

## 왜 첫 질문이 느릴 수 있나?

Claude `output_config.format.type=json_schema`는 최초 구조화 출력 스키마 컴파일 단계에서 추가 지연이 발생할 수 있습니다. 최초 사용 후 스키마는 캐시될 수 있으나 항상 캐시 적중을 보장하지는 않습니다. 사용자가 실제로 45초를 기다렸는지와 네트워크 오류인지는 이전 프로그램의 문구만으로 구분할 수 없습니다.

## 설치

V5.2를 실행 중이라면 **V5.2 대시보드만** Ctrl+C로 종료하고, Gazebo/MoveIt은 유지하세요. 새로운 터미널에서:

```bash
cd ~/AHEAD/pac2026_integrated
mkdir -p tools/pac_gazebo_3d_v5_2_1
unzip -q "$(xdg-user-dir DOWNLOAD)/PAC2026_GAZEBO_3D_DASHBOARD_V5_2_1_TIMEOUT_FIX.zip" -d tools/pac_gazebo_3d_v5_2_1
PAC_ENABLE_CLAUDE=1 bash tools/pac_gazebo_3d_v5_2_1/start_dashboard_claude.sh "$PWD"
```

키 입력 후 http://127.0.0.1:4182 를 열고 먼저 짧은 질문을 하세요: `현재 ROS 관절 수와 Gazebo 엔티티 수를 요약해줘.`

성공한다면 다음으로 `마지막 실패를 확인된 사실과 추정 원인으로 구분해줘.`를 질문하세요. API 호출 시 비용이 발생할 수 있습니다.

## 확인할 사항

- 10초 안에 `네트워크/SSL/연결 오류`가 나오면 120초 문제가 아니라 연결·SSL·프록시 문제입니다.
- 약 120초 후 `시간 초과`가 나오면 Claude 응답시간·콜드 스키마 컴파일·요청 크기 등을 검토해야 합니다.
- `HTTP 400`은 구조화 출력 스키마 또는 모델에 대한 거절일 수 있습니다.
- `HTTP 401`은 키 인증 오류입니다.

읽기 전용 어댑터이며 로봇 명령을 실행하지 않습니다. 테스트는 로컬 모의 HTTP 서버만 사용합니다. 실제 Anthropic 서버에서의 성공은 사용자 API 키로 확인해야 합니다.
