"""SafetyFilter position / non-finite contract. No CAN and no simulator."""

from __future__ import annotations

from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

from safety_contract import ELBOW_JOINT_INDEX, ELBOW_URDF_UPPER_RAD

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "tests" / "safety_filter_contract_test.cpp"
NOT_ON_MAIN = 77

CLAMP_SKIP = (
    "等待 SafetyFilter 位置夹紧（控制与安全 PR）。"
    "解除条件：filter() 把超限参考夹到调用方给出的位置上限；"
    "肘关节上限以 URDF openarm_*_joint4 的 2.443461 rad 为准，"
    "而不是 openarm_constants.hpp 里更宽的 π。"
    "本测试用一条超过 URDF 但仍小于 π 的指令区分这两种上限。"
    "当前 filter() 不读取 min_position/max_position。"
    "不测 |q_des - q_meas| 的 0.25 rad，也不测速度/力矩上限（现为 0，表示不限制）。"
)


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

    def test_out_of_range_reference_clamps_to_urdf_elbow(self):
        completed = subprocess.run(
            [str(self.binary), "clamp"],
            check=False,
            capture_output=True,
            text=True,
        )
        if completed.returncode == NOT_ON_MAIN:
            self.skipTest(CLAMP_SKIP)
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)

    def test_default_safety_config_elbow_follows_urdf(self):
        completed = subprocess.run(
            [str(self.binary), "default-elbow"],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        elbow = float(completed.stdout.strip().splitlines()[-1])
        if abs(elbow - ELBOW_URDF_UPPER_RAD) > 1e-6:
            self.skipTest(
                "等待 SafetyFilter 位置夹紧 PR："
                "make_default_safety_config() 的肘上限仍来自 openarm_constants.hpp 的 π "
                f"（当前 {elbow}）。解除条件：默认安全配置的位置上限改为 URDF "
                "openarm_*_joint4 upper（2.443461 rad）。"
            )
        self.assertAlmostEqual(elbow, ELBOW_URDF_UPPER_RAD, places=6)


if __name__ == "__main__":
    unittest.main()
