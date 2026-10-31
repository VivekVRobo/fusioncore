"""Regression tests for the benchmark regression gate's input contract."""

import contextlib
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import check_benchmark_regression as checker  # noqa: E402


class BenchmarkRegressionTests(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tempdir.cleanup)
        self.root = Path(self.tempdir.name)
        self.baseline_path = self.write_json(
            "baseline.json",
            {
                "regression_threshold_pct": 10.0,
                "sequences": {
                    "seq-a": {
                        checker.BASELINE_KEY: 1.0,
                        "verified": True,
                    }
                },
            },
        )

    def write_json(self, name, value):
        path = self.root / name
        path.write_text(json.dumps(value), encoding="utf-8")
        return path

    def write_metrics(self, name, value):
        return self.write_json(name, value)

    def invoke(self, *args):
        stdout, stderr = io.StringIO(), io.StringIO()
        with patch.object(sys, "argv", ["check_benchmark_regression.py", *map(str, args)]):
            with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
                code = checker.main()
        return code, stdout.getvalue(), stderr.getvalue()

    def valid_metrics(self, value=1.0):
        return {
            "sequence": "seq-a",
            "filters": {"FusionCore": {checker.METRIC: value}},
        }

    def test_load_metrics_accepts_valid_payload(self):
        path = self.write_metrics("metrics.json", self.valid_metrics(1.25))
        self.assertEqual(checker.load_metrics(path), ("seq-a", 1.25))

    def test_load_metrics_rejects_missing_metric(self):
        path = self.write_metrics("missing.json", {"sequence": "seq-a", "filters": {}})
        with self.assertRaisesRegex(ValueError, "FusionCore/ate_rmse_xy"):
            checker.load_metrics(path)

    def test_load_metrics_rejects_non_finite_metric(self):
        path = self.write_metrics("nan.json", self.valid_metrics(float("nan")))
        with self.assertRaisesRegex(ValueError, "finite, non-negative"):
            checker.load_metrics(path)

    def test_load_metrics_reports_malformed_json(self):
        path = self.root / "broken.json"
        path.write_text("{not-json", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "cannot read valid metrics JSON"):
            checker.load_metrics(path)

    def test_invalid_metrics_fail_closed_without_pass_message(self):
        path = self.write_metrics("missing-metric.json", {"sequence": "seq-a", "filters": {}})
        code, stdout, stderr = self.invoke(path, "--baseline", self.baseline_path)
        self.assertEqual(code, 2)
        self.assertNotIn("PASS:", stdout)
        self.assertIn("invalid", stderr.lower())

    def test_zero_baseline_is_rejected_instead_of_dividing_by_zero(self):
        baseline = self.write_json(
            "zero-baseline.json",
            {"sequences": {"seq-a": {checker.BASELINE_KEY: 0.0}}},
        )
        metrics = self.write_metrics("valid.json", self.valid_metrics(1.0))
        code, stdout, stderr = self.invoke(metrics, "--baseline", baseline)
        self.assertEqual(code, 2)
        self.assertNotIn("PASS:", stdout)
        self.assertIn("finite, positive", stderr)

    def test_valid_metric_passes(self):
        metrics = self.write_metrics("valid.json", self.valid_metrics(1.05))
        code, stdout, _ = self.invoke(metrics, "--baseline", self.baseline_path)
        self.assertEqual(code, 0)
        self.assertIn("PASS:", stdout)

    def test_regression_returns_nonzero(self):
        metrics = self.write_metrics("regressed.json", self.valid_metrics(1.2))
        code, stdout, _ = self.invoke(metrics, "--baseline", self.baseline_path)
        self.assertEqual(code, 1)
        self.assertIn("REGRESSION", stdout)
        self.assertNotIn("PASS:", stdout)

    def test_negative_threshold_is_rejected(self):
        metrics = self.write_metrics("valid.json", self.valid_metrics())
        code, stdout, stderr = self.invoke(
            metrics, "--baseline", self.baseline_path, "--threshold", "-1"
        )
        self.assertEqual(code, 2)
        self.assertNotIn("PASS:", stdout)
        self.assertIn("non-negative", stderr)


if __name__ == "__main__":
    unittest.main()
