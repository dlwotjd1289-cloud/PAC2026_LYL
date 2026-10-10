"""실행 로그를 즉시 전달하고 출력이 없는 동안 진행 중임을 알린다."""
from __future__ import annotations

import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time


def run_logged(command, *, log_path, env, cwd, on_line, on_wait, heartbeat_s=5.0):
    messages = queue.Queue()
    started = time.monotonic()
    with Path(log_path).open('w', encoding='utf-8') as log:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   text=True, encoding='utf-8', errors='replace', bufsize=1,
                                   start_new_session=True, env=env, cwd=cwd)

        def read_output():
            try:
                for line in process.stdout:
                    messages.put(line)
            finally:
                messages.put(None)

        reader = threading.Thread(target=read_output, daemon=True)
        reader.start()
        try:
            while True:
                try:
                    line = messages.get(timeout=heartbeat_s)
                except queue.Empty:
                    on_wait(time.monotonic() - started)
                    continue
                if line is None:
                    break
                log.write(line)
                log.flush()
                on_line(line.rstrip('\n'))
            return process.wait()
        finally:
            if process.poll() is None:
                # Only the subprocess group created by this function is stopped.
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            reader.join(timeout=1)
            process.stdout.close()
