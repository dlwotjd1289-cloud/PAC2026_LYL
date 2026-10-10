import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

SOURCE = Path(__file__).resolve().parents[1] / 'workcell/collect_pick_diagnostics.py'
spec = importlib.util.spec_from_file_location('collect_pick_diagnostics', SOURCE)
diag = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diag)


class CollectDiagnosticsTests(unittest.TestCase):
    def test_missing_is_incomplete(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = diag.collect(tmp)
            self.assertEqual(result['status'], 'incomplete')
            self.assertEqual(len(result['errors']), 2)

    def test_only_numeric_evidence_and_motion_log(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            logs = root / 'logs/lyl_cell/run01'
            logs.mkdir(parents=True)
            (logs / 'box_01_moveit_pallet.log').write_text(
                'dashboard question PRIVATE\n[vertical] STOP: edge exceeded\n'
                '[vertical] api_key=SECRET\nMOVEIT PICK&PLACE FAIL: press\n')
            traces = root / 'logs/vertical_v46'
            traces.mkdir()
            sample = {'stamp': 1.2, 'desired_tcp': [0, 0, 1.0], 'actual_tcp': [0, 0, 1.003],
                      'actual': {'xy_m': .0002, 'orientation_deg': .2, 'edge_bound_m': .0021}}
            (traces / 'vertical_01_press_cup_on_box.json').write_text(json.dumps({
                'step': 'press cup on box', 'radius_m': .5, 'completed': True,
                'guard_failure': 'PRIVATE_TOKEN', 'unrelated': 'PRIVATE',
                'planned': [{'tcp': [0, 0, 1.0]}], 'samples': [sample]}))
            result = diag.collect(root)
            self.assertEqual(result['status'], 'ok')
            self.assertAlmostEqual(result['press']['last_sample']['actual_z_error_m'], .003)
            self.assertEqual(result['press']['peak_actual_edge_sample']['actual']['edge_bound_m'], .0021)
            self.assertNotIn('PRIVATE', json.dumps(result))
            self.assertNotIn('SECRET', json.dumps(result))

    def test_corrupt_trace_is_friendly(self):
        with tempfile.TemporaryDirectory() as tmp:
            traces = Path(tmp) / 'logs/vertical_v46'
            traces.mkdir(parents=True)
            (traces / 'vertical_01_press.json').write_text('{')
            result = diag.collect(tmp)
            self.assertEqual(result['status'], 'incomplete')
            self.assertTrue(any('JSON' in message for message in result['errors']))

    def test_nonfinite_metrics_not_emitted(self):
        self.assertEqual(diag.metrics({'xy_m': float('nan'), 'edge_bound_m': True}),
                         {'xy_m': None, 'orientation_deg': None, 'edge_bound_m': None})


if __name__ == '__main__':
    unittest.main()
