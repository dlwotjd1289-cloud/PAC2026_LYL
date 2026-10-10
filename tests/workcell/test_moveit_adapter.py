"""Adapter regression checks with a fake executor; no ROS or robot motion."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
ADAPTER = ROOT / "workcell" / "lyl_moveit_executor.py"


class MoveItAdapterTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.cell = Path(self.tmp.name)
        scripts = self.cell / "scripts"
        scripts.mkdir()
        (scripts / "vertical_descent_v46.py").write_text("EXEC_EDGE_M = 0.0015\nPRESS_EDGE_M = 0.002\n")
        (scripts / "moveit_pick_place_v44.py").write_text(
            "import json, os, sys\nimport vertical_descent_v46 as guard\n"
            "PRESS_M = 0.006\nCART_SPEED = 0.20\n"
            "def main():\n"
            "    print(json.dumps({'press': PRESS_M, 'speed': CART_SPEED, "
            "'edge': guard.EXEC_EDGE_M, 'press_edge': guard.PRESS_EDGE_M, 'args': sys.argv[1:]}))\n"
            "    return 7\n"
        )

    def run_adapter(self, value=None):
        env = dict(os.environ, PAC_WORKCELL_REPO=str(self.cell))
        env.pop("PAC_PRESS_M", None)
        if value is not None:
            env["PAC_PRESS_M"] = value
        return subprocess.run([sys.executable, str(ADAPTER), "--box", "box_01"],
                              env=env, text=True, capture_output=True)

    def test_default_only_changes_press_and_preserves_args_exit_and_guards(self):
        result = self.run_adapter()
        self.assertEqual(result.returncode, 7, result.stderr)
        data = json.loads(result.stdout.splitlines()[-1])
        self.assertEqual(data, {"press": 0.001, "speed": 0.20, "edge": 0.0015,
                                "press_edge": 0.002, "args": ["--box", "box_01"]})

    def test_valid_bounds(self):
        for value in ("0", "0.002"):
            with self.subTest(value=value):
                result = self.run_adapter(value)
                self.assertEqual(result.returncode, 7, result.stderr)
                self.assertEqual(json.loads(result.stdout.splitlines()[-1])["press"], float(value))

    def test_invalid_press_fails_before_loading_original_executor(self):
        (self.cell / "scripts" / "moveit_pick_place_v44.py").write_text("raise RuntimeError('must not import')\n")
        for value in ("-0.001", "0.0021", "nan", "inf", "bad"):
            with self.subTest(value=value):
                result = self.run_adapter(value)
                self.assertEqual(result.returncode, 2)
                self.assertIn("Invalid PAC_PRESS_M", result.stderr)
                self.assertNotIn("must not import", result.stderr)

    def test_missing_original_executor_reports_failure(self):
        (self.cell / "scripts" / "moveit_pick_place_v44.py").unlink()
        result = self.run_adapter()
        self.assertEqual(result.returncode, 2)
        self.assertIn("Original V4.6 executor missing", result.stderr)


if __name__ == "__main__":
    unittest.main()
