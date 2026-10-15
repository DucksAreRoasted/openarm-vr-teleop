"""CAN feedback freshness contract. Does not open SocketCAN.

The age / missing-reply / error_code rule is enforced by replay.
PR #4's evaluate_can_feedback is asserted when that header is present.
"""

from __future__ import annotations

from pathlib import Path
import subprocess
import tempfile
import unittest

import re

from interface_probe import MISSING_API, declares
from safety_contract import CAN_ERROR_CODE_MAX_OK, CAN_FEEDBACK_MAX_AGE_MS

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "tests" / "can_feedback_contract_test.cpp"
API_SOURCE = ROOT / "tests" / "can_feedback_api_test.cpp"
CAN_ROOT = ROOT / "third_party" / "openarm_can"
REPLAY = ROOT / "src" / "controller" / "replay_arm.hpp"
CONSTANTS = ROOT / "src" / "openarm_constants.hpp"


def _compile(binary: Path) -> None:
    subprocess.run(
        [
            "c++",
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-I",
            str(ROOT / "tests"),
            "-I",
            str(CAN_ROOT / "include"),
            str(SOURCE),
            str(CAN_ROOT / "src" / "openarm" / "damiao_motor" / "dm_motor.cpp"),
            "-o",
            str(binary),
        ],
        check=True,
        capture_output=True,
        text=True,
    )


class CanFeedbackContractTests(unittest.TestCase):
    def test_missing_stale_and_fault_feedback_are_invalid(self):
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / "can_feedback_contract_test"
            _compile(binary)
            completed = subprocess.run(
                [str(binary)],
                check=False,
                capture_output=True,
                text=True,
            )
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)

    def test_replay_threshold_matches_contract(self):
        text = REPLAY.read_text(encoding="utf-8")
        age = int(CAN_FEEDBACK_MAX_AGE_MS)
        self.assertIn(
            f"age_ms > {age} || m.get_error_code() > {CAN_ERROR_CODE_MAX_OK}",
            text,
        )
        self.assertIn("last_state_received().time_since_epoch().count() == 0", text)

    def test_evaluate_can_feedback_and_null_arm(self):
        if not declares("src/openarm_port/can_feedback.hpp", "evaluate_can_feedback"):
            self.skipTest(MISSING_API)
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / "can_feedback_api_test"
            compiled = subprocess.run(
                [
                    "c++",
                    "-std=c++17",
                    "-Wall",
                    "-Wextra",
                    "-I",
                    str(ROOT / "src"),
                    "-I",
                    str(CAN_ROOT / "include"),
                    str(API_SOURCE),
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

    def test_production_thresholds_match_contract(self):
        if not declares("src/openarm_constants.hpp", "kCanFeedbackMaxAgeMs", invocation=False):
            self.skipTest(MISSING_API)
        text = CONSTANTS.read_text(encoding="utf-8")
        age = re.search(r"\bkCanFeedbackMaxAgeMs\s*=\s*([0-9]+(?:\.[0-9]+)?)", text)
        code = re.search(r"\bkCanFeedbackMaxErrorCode\s*=\s*([0-9]+)", text)
        self.assertIsNotNone(age)
        self.assertIsNotNone(code)
        self.assertAlmostEqual(float(age.group(1)), CAN_FEEDBACK_MAX_AGE_MS)
        self.assertEqual(int(code.group(1)), CAN_ERROR_CODE_MAX_OK)


if __name__ == "__main__":
    unittest.main()
