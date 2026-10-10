#!/usr/bin/env bash
# Source this file in the foreground before starting background dashboards.
# The key lives only in the process environment; do not run this helper through tee.

pac_dashboard_api_env() {
  # Disable shell tracing before accessing a secret, even if the caller uses bash -x.
  local lyl_restore_xtrace=0
  case $- in *x*) set +x; lyl_restore_xtrace=1 ;; esac
  local lyl_key='' lyl_tty_fd

  if [[ "${PAC_ENABLE_CLAUDE:-}" == 0 ]]; then
    printf '%s\n' '대시보드: 오프라인 진단 모드(API 사용 안 함).' >&2
  elif [[ -n "${ANTHROPIC_API_KEY:-}" ]]; then
    export ANTHROPIC_API_KEY PAC_ENABLE_CLAUDE=1
    printf '%s\n' '대시보드: 기존 Claude API 키 사용(키 검증·API 호출은 아직 하지 않음).' >&2
  elif { exec {lyl_tty_fd}<>/dev/tty; } 2>/dev/null; then
    printf '%s' 'Claude API 키 입력(숨김, Enter: 오프라인으로 계속): ' >&"$lyl_tty_fd"
    IFS= read -r -s -u "$lyl_tty_fd" lyl_key || lyl_key=''
    printf '\n' >&"$lyl_tty_fd"
    exec {lyl_tty_fd}>&-
    if [[ -n "$lyl_key" ]]; then
      export ANTHROPIC_API_KEY="$lyl_key" PAC_ENABLE_CLAUDE=1
      printf '%s\n' '대시보드: Claude API 키 입력 완료(키 검증·API 호출은 아직 하지 않음).' >&2
    else
      export PAC_ENABLE_CLAUDE=0
      printf '%s\n' '대시보드: 키 입력을 건너뛰어 오프라인 진단으로 계속합니다.' >&2
    fi
  else
    export PAC_ENABLE_CLAUDE=0
    printf '%s\n' '대시보드: 입력 터미널과 API 키가 없어 오프라인 진단으로 계속합니다.' >&2
  fi
  # Downstream launchers use PAC_ENABLE_CLAUDE and ANTHROPIC_API_KEY unchanged.
  unset lyl_key
  if (( lyl_restore_xtrace )); then set -x; fi
  return 0
}

pac_dashboard_api_env
