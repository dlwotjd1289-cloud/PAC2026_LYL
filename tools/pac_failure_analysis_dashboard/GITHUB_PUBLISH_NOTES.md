# GitHub 코드 배포 범위

- **새 기능 브랜치에만 게시**: 팀의 main, 기존 알고리즘, ROS/MoveIt/Gazebo 설정에 변경 없음.
- V5.3의 독립 대시보드/관측기/Claude API 클라이언트/실행 계약 모듈, 문서, 자동 테스트 업로드.
- 실행 중 생성된 `logs/`, 카메라 프레임, `active_run.json`, 환경변수/API 키/캐시/테스트 임시 파일 제외.
- Plotly 3.3.1 대형 빌드(4.8MB)는 소스 저장소에서 제외하고 공식 CDN 리다이렉트 또는 선택적 로컬 설치 사용.
- `active_run.example.json`은 예시 값이며 실제 실행 데이터가 아닙니다.
- 통합 실행기에서 `active_run.json`과 완료 이벤트를 실제로 발행하기 전까지는 박스별 확정 처리 상태가 모두 표시되지 않을 수 있습니다.
- API 모델 비용, 원격 로그 전송, 인증된 안전장치와의 분리는 `CLAUDE_SETUP_KO.md`와 `INTEGRATION_GUIDE_KO.md` 참고.
