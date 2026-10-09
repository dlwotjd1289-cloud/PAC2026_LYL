# 팀 monorepo 통합 방법

권장 최종 배치:

```text
pac2026/
├── config/
│   ├── default.yaml
│   ├── robot.yaml
│   └── local.example.yaml
├── test_data/
├── ros2_ws/src/
│   ├── pac_common/
│   ├── pac_perception/
│   ├── pac_planning/
│   ├── pac_robot/
│   └── pac_bringup/
└── tools/
    └── ahead_dataset_generator/
```

이 ZIP의 `src/pac_common`은 단독 검증용 기준 구현이다. 팀 monorepo에 이미 `pac_common`이 생기면 그 패키지를 단일 원본으로 사용하고, Generator의 중복 사본은 제거한다.

Generator 출력의 `test_data` JSON은 storage fixture이며, 런타임에서는 adapter를 통해 `pac_common.SystemState`로 변환한다. JSON raw dict를 Planner에 직접 전달하지 않는다.
