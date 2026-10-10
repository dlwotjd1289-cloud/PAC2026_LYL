"""Dashboard launcher should preserve a dashboard it did not start."""
from pathlib import Path


def test_launcher_does_not_terminate_unrelated_dashboard_processes():
    script = (Path(__file__).resolve().parents[2] / 'workcell/run_gazebo_demo.sh').read_text()
    section = script.split('echo ">>> [6/6] API 연동 대시보드 V5.3', 1)[1].split(
        'if wait "$RUNNER"; then', 1)[0]
    assert 'pgrep -f' not in section
    assert 'do kill' not in section
    assert 'api/state' in section
    assert 'DASHBOARD_PID=""' in section
    assert 'setsid nohup bash' in section
