"""실제 셸 준비 확인 분기를 실행하되 ROS와 외부 HTTP는 모의 처리한다."""
from pathlib import Path
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[2] / 'workcell/run_gazebo_demo.sh'


class LaunchReadinessTests(unittest.TestCase):
    def run_dashboard_gate(self, curl_status):
        script = SCRIPT.read_text()
        fragment = script[script.index('dashboard_status=1\n'):]
        with tempfile.TemporaryDirectory() as tmp:
            trace = Path(tmp) / 'curl_args'
            prefix = '''set -euo pipefail
PORT=4199
LOG=test
NO_BROWSER=1
DASHBOARD_PID=99999999
curl() { printf '%s\\n' "$@" > "$TRACE"; return "$CURL_STATUS"; }
sleep() { :; }
(exit 0) &
RUNNER=$!
'''
            import os
            result = subprocess.run(['bash', '-c', prefix + fragment],
                                    env=dict(os.environ, TRACE=str(trace), CURL_STATUS=str(curl_status)),
                                    capture_output=True, text=True, timeout=5)
            return result, trace.read_text()

    def test_ready_uses_configured_port(self):
        result, args = self.run_dashboard_gate(0)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('http://127.0.0.1:4199/api/state', args)
        self.assertIn('RUN DONE:', result.stdout)

    def test_dashboard_failure_is_not_reported_as_complete(self):
        result, _ = self.run_dashboard_gate(7)
        self.assertEqual(result.returncode, 1)
        self.assertIn('대시보드 시작 실패', result.stderr)
        self.assertNotIn('RUN DONE:', result.stdout)

    def test_missing_current_manifest_preserves_runner_failure(self):
        script = SCRIPT.read_text()
        start = script.index('RUNNER=$!\n') + len('RUNNER=$!\n')
        fragment = script[start:script.index('echo ">>> [6/6]')]
        for status, expected in ((7, 7), (0, 1)):
            with self.subTest(status=status), tempfile.TemporaryDirectory() as tmp:
                prefix = f'''set -euo pipefail
LOG=test
MANIFEST='{tmp}/missing.json'
before=missing
sleep() {{ :; }}
(exit {status}) &
RUNNER=$!
'''
                result = subprocess.run(['bash', '-c', prefix + fragment],
                                        capture_output=True, text=True, timeout=5)
                self.assertEqual(result.returncode, expected)
                self.assertIn('현재 실행 등록을 확인하지 못했습니다', result.stderr)


if __name__ == '__main__':
    unittest.main()
