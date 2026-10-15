"""SafetyFilter position / non-finite contract. No CAN and no simulator."""

from __future__ import annotations

from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

from interface_probe import MISSING_API, declares
from safety_contract import ELBOW_JOINT_INDEX, ELBOW_URDF_UPPER_RAD

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "tests" / "safety_filter_contract_test.cpp"
API_SOURCE = ROOT / "tests" / "safety_filter_api_test.cpp"


def _compile(binary: Path) -> None:
    subprocess.run(
        [
            "c++",
            "-std=c++17",
            "-Wall",
            "-Wextra",
            # robot_state.hpp uses std::runtime_error without including <stdexcept>.
            # Force the header here so this test can compile current main unchanged.
            "-include",
            "stdexcept",
            "-I",
            str(ROOT / "src"),
            "-I",
            str(ROOT / "tests"),
            "-I",
            str(ROOT / "third_party" / "openarm_can" / "include"),
            str(SOURCE),
            str(ROOT / "src" / "safety" / "safety_filter.cpp"),
            "-o",
            str(binary),
        ],
        check=True,
        capture_output=True,
        text=True,
    )


class SafetyFilterContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._directory = tempfile.TemporaryDirectory()
        cls.binary = Path(cls._directory.name) / "safety_filter_contract_test"
        _compile(cls.binary)

    @classmethod
    def tearDownClass(cls):
        cls._directory.cleanup()

    def test_urdf_elbow_matches_contract(self):
        root = ET.parse(ROOT / "assets" / "v1.urdf").getroot()
        for name in ("openarm_left_joint4", "openarm_right_joint4"):
            limit = root.find(f"joint[@name='{name}']/limit")
            self.assertIsNotNone(limit, name)
            self.assertAlmostEqual(float(limit.attrib["upper"]), ELBOW_URDF_UPPER_RAD, places=9)
        self.assertEqual(ELBOW_JOINT_INDEX, 3)

    def test_nonfinite_keeps_previous_position(self):
        completed = subprocess.run(
            [str(self.binary), "nonfinite"],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)

    def test_urdf_clamp_hold_stop_and_size_mismatch(self):
        if not declares("src/safety/safety_filter.hpp", "hold_previous_joint"):
            self.skipTest(MISSING_API)
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / "safety_filter_api_test"
            compiled = subprocess.run(
                [
                    "c++",
                    "-std=c++17",
                    "-Wall",
                    "-Wextra",
                    "-include",
                    "stdexcept",
                    "-I",
                    str(ROOT / "src"),
                    "-I",
                    str(ROOT / "tests"),
                    "-I",
                    str(ROOT / "third_party" / "openarm_can" / "include"),
                    str(API_SOURCE),
                    str(ROOT / "src" / "safety" / "safety_filter.cpp"),
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
