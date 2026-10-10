"""No ROS/Gazebo: failed physical actions must not be committed as success."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import unittest
import tempfile
import subprocess


ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('lyl_cell_runner_under_test', ROOT / 'workcell/lyl_cell_runner.py')
runner_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner_module)


def bare_runner(tmp_path):
    r = runner_module.Runner.__new__(runner_module.Runner)
    r.stop = False
    r.run = SimpleNamespace(log=Mock(), event=Mock(), dir=tmp_path)
    r.core = SimpleNamespace(on_result=Mock(), sm=SimpleNamespace(t=1.0),
                             supervisor=SimpleNamespace(pallet_change=Mock()))
    r.pallet_no = 1
    r.placed_now = [('box_01', 'SKU1', (0.2, 0.2, 0.2))]
    r.state_file = tmp_path / 'pallet_1_state.json'
    return r


def check_ng_commits_only_after_success(tmp_path, success):
    r = bare_runner(tmp_path)
    box = SimpleNamespace(name='box_02')
    r.box_of = Mock(return_value=box)
    r.to_ng = Mock(return_value=success)
    cmd = SimpleNamespace(action='REJECT_NG', box_id='B2', reason='damaged')
    assert r.execute(cmd) is success
    assert getattr(box, 'done', False) is success
    assert r.core.on_result.call_count == int(success)
    assert r.stop is (not success)


def check_pallet_swap_failure_keeps_current_pallet(tmp_path, monkeypatch, returncode):
    r = bare_runner(tmp_path)
    old_placed = list(r.placed_now)
    old_state_file = r.state_file
    monkeypatch.setattr(runner_module.subprocess, 'run', Mock(return_value=SimpleNamespace(
        returncode=returncode, stdout='swap output', stderr='swap failure' if returncode else '')))
    cmd = SimpleNamespace(action='PALLET_CLOSE', box_id=None, reason='dead')
    assert r.execute(cmd) is (returncode == 0)
    if returncode:
        assert r.pallet_no == 1
        assert r.placed_now == old_placed
        assert r.state_file == old_state_file
        r.core.on_result.assert_not_called()
        r.core.supervisor.pallet_change.assert_not_called()
        assert r.stop
    else:
        assert r.pallet_no == 2
        assert r.placed_now == []
        r.core.on_result.assert_called_once_with(cmd)
        r.core.supervisor.pallet_change.assert_called_once_with(1.0)


def check_moveit_nonzero_exit_cannot_pass_on_log_marker(tmp_path, monkeypatch):
    r = bare_runner(tmp_path)
    r.buffer = {}
    r.cfg = {'robot': {'free_speed': 0.18, 'cart_speed': 0.20, 'gripper': 'test'}}
    b = SimpleNamespace(name='box_02', obs=SimpleNamespace(weight_kg=1.0),
        field=SimpleNamespace(truth=SimpleNamespace(size=SimpleNamespace(x=.2, y=.2, z=.2))))
    def failed_process(*args, **kwargs):
        kwargs['stdout'].write('MOVEIT PICK&PLACE PASS\nsubsequent failure\n')
        return SimpleNamespace(returncode=1)
    monkeypatch.setattr(runner_module.subprocess, 'run', failed_process)
    ok, measured, text = r.moveit(b, 'test', [])
    assert not ok
    assert measured is None
    assert 'MOVEIT PICK&PLACE PASS' in text


def check_prepare_shell_gate_preserves_exit_status(tmp_path, returncode):
    # Exercise the production gate exactly, replacing only the external executable.
    script = (ROOT / 'workcell/run_gazebo_demo.sh').read_text()
    gate = script.split('# work cell collision scene', 1)[1].split('echo ">>> [4/6]', 1)[0]
    gate = gate[gate.index('if ! '):]
    wc = tmp_path / 'cell'
    (wc / 'scripts').mkdir(parents=True)
    (wc / 'scripts/moveit_prepare_v46.py').write_text(f'raise SystemExit({returncode})\n')
    log = tmp_path / 'logs'
    log.mkdir()
    result = subprocess.run(['bash', '-c', 'set -eo pipefail\nWC="$1"\nLOG="$2"\n' + gate + '\necho AFTER_PREPARE\n',
                             'test', str(wc), str(log)], capture_output=True, text=True)
    assert result.returncode == (0 if returncode == 0 else 1)
    assert ('AFTER_PREPARE' in result.stdout) is (returncode == 0)
    assert (log / 'moveit_prepare.log').exists()


class ExecutionFailureTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.tmp_path = Path(self.temp.name)

    def setattr(self, target, name, value):
        patcher = patch.object(target, name, value)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_ng_failure(self):
        check_ng_commits_only_after_success(self.tmp_path, False)

    def test_ng_success(self):
        check_ng_commits_only_after_success(self.tmp_path, True)

    def test_swap_failure(self):
        check_pallet_swap_failure_keeps_current_pallet(self.tmp_path, self, 3)

    def test_swap_success(self):
        check_pallet_swap_failure_keeps_current_pallet(self.tmp_path, self, 0)

    def test_moveit_exit_status(self):
        check_moveit_nonzero_exit_cannot_pass_on_log_marker(self.tmp_path, self)

    def test_prepare_failure(self):
        check_prepare_shell_gate_preserves_exit_status(self.tmp_path, 9)

    def test_prepare_success(self):
        check_prepare_shell_gate_preserves_exit_status(self.tmp_path, 0)


    def test_runner_outcomes_propagate_and_stop_conveyor(self):
        for outcome, expected in [('complete', 0), ('failed', 1), ('exception', 1), ('interrupt', 130)]:
            with self.subTest(outcome=outcome):
                r = bare_runner(self.tmp_path)
                r.args = SimpleNamespace(scenario='S0001')
                r.boxes, r.decisions = [], []
                r.run.id = 'test_run'
                r.core.closed, r.core.sm.placed = [], []
                r.core.inspection, r.core.counts = [], {}
                r.cell = SimpleNamespace(roller=Mock())
                r.conveyor_loop = Mock()
                def decide():
                    if outcome == 'failed':
                        r.stop = True
                    elif outcome == 'exception':
                        raise RuntimeError('test failure')
                    elif outcome == 'interrupt':
                        raise KeyboardInterrupt()
                r.decision_loop = decide
                assert r.main() == expected
                r.cell.roller.assert_called_with(0.0)
                summary = runner_module.json.loads((self.tmp_path / 'summary.json').read_text())
                assert summary['exit_code'] == expected
                assert summary['status'] == ('interrupted' if outcome == 'interrupt' else
                                              'complete' if outcome == 'complete' else 'failed')

    def test_shell_wait_preserves_runner_failure(self):
        script = (ROOT / 'workcell/run_gazebo_demo.sh').read_text()
        tail = script[script.index('if wait "$RUNNER"; then'):]
        for status in (0, 9):
            with self.subTest(status=status):
                result = subprocess.run(['bash', '-c',
                    f'set -eo pipefail\nLOG=test\n(exit {status}) &\nRUNNER=$!\n' + tail],
                    capture_output=True, text=True)
                assert result.returncode == status
                assert ('RUN DONE:' in result.stdout) is (status == 0)


if __name__ == '__main__':
    unittest.main()
