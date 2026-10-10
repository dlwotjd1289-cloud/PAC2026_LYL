"""자식 프로세스 종료 전 로그 전달 및 무출력 대기를 확인한다."""
import os
from pathlib import Path
import sys
import tempfile
import unittest

from workcell.process_stream import run_logged


class ProcessStreamTests(unittest.TestCase):
    def test_output_reaches_parent_before_child_can_complete(self):
        with tempfile.TemporaryDirectory() as tmp:
            gate, log = Path(tmp)/'gate', Path(tmp)/'moveit.log'
            lines = []
            def on_line(line):
                lines.append(line)
                gate.touch()
            code = ('import pathlib,sys,time; print("stage", flush=True); '
                    'p=pathlib.Path(sys.argv[1]); end=time.monotonic()+3\n'
                    'while not p.exists() and time.monotonic()<end: time.sleep(.01)\n'
                    'raise SystemExit(0 if p.exists() else 9)')
            rc = run_logged([sys.executable, '-u', '-c', code, str(gate)],
                            log_path=log, env=os.environ.copy(), cwd=tmp,
                            on_line=on_line, on_wait=lambda _: None, heartbeat_s=.05)
            self.assertEqual(rc, 0)
            self.assertEqual(lines, ['stage'])
            self.assertEqual(log.read_text(), 'stage\n')

    def test_silence_reports_wait_and_failure_exit_is_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            waits = []
            rc = run_logged([sys.executable, '-c', 'import time; time.sleep(.2); raise SystemExit(7)'],
                            log_path=Path(tmp)/'moveit.log', env=os.environ.copy(), cwd=tmp,
                            on_line=lambda _: None, on_wait=waits.append, heartbeat_s=.03)
            self.assertEqual(rc, 7)
            self.assertTrue(waits)
