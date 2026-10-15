"""Calibration plan bounds, paired releases and adaptive settling; no CAN."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class GravityCalibrationPlanTests(unittest.TestCase):
    def test_bounds_speed_and_adaptive_hold(self):
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / "calibration_plan_test"
            build = subprocess.run(
                ["c++", "-std=c++17", "-Wall", "-Wextra", "-Wpedantic",
                 "-I", str(ROOT / "src"),
                 str(ROOT / "tests/gravity_calibration_plan_test.cpp"), "-o", str(binary)],
                capture_output=True, text=True,
            )
            self.assertEqual(build.returncode, 0, build.stderr)
            result = subprocess.run([str(binary)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(len(result.stdout.splitlines()), 32)


if __name__ == "__main__":
    unittest.main()
