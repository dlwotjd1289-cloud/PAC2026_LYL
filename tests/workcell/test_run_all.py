"""Fast command-flow tests: no generator, ROS, Gazebo or external API calls."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[2] / 'run_all.sh'


class RunAllTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / 'repo with spaces'
        (self.root / 'workcell').mkdir(parents=True)
        (self.root / 'bin').mkdir()
        shutil.copy2(SCRIPT, self.root / 'run_all.sh')
        (self.root / 'workcell/dashboard_api_env.sh').write_text('export API_HELPER_LOADED=yes\n')
        (self.root / 'workcell/run_gazebo_demo.sh').write_text(
            'test "$API_HELPER_LOADED" = yes || exit 8\n'
            'printf "gazebo:%s:%s:%s\\n" "$1" "$2" "$3" >> "$FLOW_LOG"\n'
            'exit "${GAZEBO_STATUS:-0}"\n')
        stub = self.root / 'bin/python3'
        stub.write_text('#!/usr/bin/env bash\ncat >/dev/null\n'
                        'echo generate >> "$FLOW_LOG"\n'
                        '[[ "${GEN_STATUS:-0}" == 0 ]] || exit "$GEN_STATUS"\n'
                        'mkdir -p "$3"\n')
        stub.chmod(0o755)
        self.log = self.root / 'flow.log'
        self.env = dict(os.environ, PATH=str(stub.parent)+os.pathsep+os.environ['PATH'], FLOW_LOG=str(self.log))

    def run_script(self, *args):
        return subprocess.run(['bash', str(self.root / 'run_all.sh'), *map(str, args)],
                              cwd=self.root, env=self.env, capture_output=True, text=True)

    def test_default_generates_then_starts_full_s0002(self):
        result = self.run_script('--output', self.root / 'out')
        self.assertEqual(result.returncode, 0, result.stderr)
        lines = self.log.read_text().splitlines()
        self.assertEqual(lines[0], 'generate')
        self.assertEqual(lines[1], f'gazebo:{self.root}/out/dataset:S0002:0')

    def test_existing_dataset_skips_generation_and_forwards_failure(self):
        dataset = self.root / 'existing data'
        dataset.mkdir()
        self.env['GAZEBO_STATUS'] = '9'
        result = self.run_script('--dataset', dataset, '--scenario', 'S0003', '--count', '2')
        self.assertEqual(result.returncode, 9)
        self.assertEqual(self.log.read_text().strip(), f'gazebo:{dataset}:S0003:2')

    def test_generator_failure_never_starts_gazebo(self):
        self.env['GEN_STATUS'] = '7'
        result = self.run_script()
        self.assertEqual(result.returncode, 7)
        self.assertEqual(self.log.read_text().strip(), 'generate')

    def test_invalid_count_stops_before_any_execution(self):
        self.assertEqual(self.run_script('--count', '-1').returncode, 2)
        self.assertFalse(self.log.exists())


if __name__ == '__main__':
    unittest.main()
