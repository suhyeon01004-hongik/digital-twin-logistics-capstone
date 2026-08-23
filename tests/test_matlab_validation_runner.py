from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPOSITORY_ROOT / "scripts" / "run_matlab_validation.py"
SPEC = importlib.util.spec_from_file_location("run_matlab_validation_test", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class MatlabValidationRunnerTests(unittest.TestCase):
    def test_all_expands_to_two_independent_suites(self):
        self.assertEqual(MODULE.selected_suites("all"), ["manual", "e2e"])

    def test_single_suite_is_preserved(self):
        self.assertEqual(MODULE.selected_suites("manual"), ["manual"])

    def test_batch_expression_escapes_matlab_string_literals(self):
        expression = MODULE.build_batch_expression(
            Path("/tmp/team's/milemate"),
            "run_parcel_manual_regression_suite",
            "quick",
        )
        self.assertIn("cd('/tmp/team''s/milemate')", expression)
        self.assertIn("run_parcel_manual_regression_suite('quick')", expression)


if __name__ == "__main__":
    unittest.main()
