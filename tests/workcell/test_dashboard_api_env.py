"""Shell environment handoff only; never contacts an API or starts a dashboard."""
import os
from pathlib import Path
import subprocess


HELPER = Path(__file__).resolve().parents[2] / "workcell/dashboard_api_env.sh"


def run_helper(extra=None, traced=False):
    env = os.environ.copy()
    for name in ("ANTHROPIC_API_KEY", "PAC_ENABLE_CLAUDE"):
        env.pop(name, None)
    env.update(extra or {})
    return subprocess.run(
        ["bash", "-euxc" if traced else "-euc",
         'source "$1"; printf "mode=%s\\n" "$PAC_ENABLE_CLAUDE"',
         "bash", str(HELPER)],
        env=env, capture_output=True, text=True, start_new_session=True, timeout=5,
    )


def test_noninteractive_without_key_continues_offline():
    result = run_helper({"PAC_ENABLE_CLAUDE": "1"})
    assert result.returncode == 0
    assert result.stdout == "mode=0\n"
    assert "오프라인" in result.stderr


def test_existing_key_is_reused_without_prompt_or_trace_leak():
    sentinel = "test-secret-never-log-123"
    result = run_helper({"ANTHROPIC_API_KEY": sentinel}, traced=True)
    assert result.returncode == 0
    assert result.stdout == "mode=1\n"
    assert sentinel not in result.stdout + result.stderr
    assert "기존 Claude API 키 사용" in result.stderr


def test_explicit_offline_is_respected():
    result = run_helper({"PAC_ENABLE_CLAUDE": "0", "ANTHROPIC_API_KEY": "unused-test-key"})
    assert result.returncode == 0
    assert result.stdout == "mode=0\n"
