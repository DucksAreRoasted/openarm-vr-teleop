"""plan_next_deadline does not catch up missed cycles. No realtime thread."""

from __future__ import annotations

from pathlib import Path
import subprocess
import tempfile
import unittest

from interface_probe import MISSING_API, declares

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "tests" / "periodic_deadline_test.cpp"


class PeriodicDeadlineTests(unittest.TestCase):
    def test_late_deadline_is_one_period_after_now(self):
        if not declares("src/periodic_timer_thread.hpp", "plan_next_deadline"):
            self.skipTest(MISSING_API)
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / "periodic_deadline_test"
            compiled = subprocess.run(
                [
                    "c++",
                    "-std=c++17",
                    "-Wall",
                    "-Wextra",
                    "-I",
                    str(ROOT / "src"),
                    str(SOURCE),
                    "-pthread",
                    "-o",
                    str(binary),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            completed = subprocess.run(
                [str(binary)],
                check=False,
                capture_output=True,
                text=True,
            )
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)


if __name__ == "__main__":
    unittest.main()
