"""CAN feedback freshness contract. Does not open SocketCAN.

The age / missing-reply / error_code rule matches mirror teaching and is
asserted here. Teleop startup, zero-g, and MoveIt real_arm do not use it yet.
"""

from __future__ import annotations

from pathlib import Path
import subprocess
import tempfile
import unittest

from safety_contract import CAN_ERROR_CODE_MAX_OK, CAN_FEEDBACK_MAX_AGE_MS

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "tests" / "can_feedback_contract_test.cpp"
CAN_ROOT = ROOT / "third_party" / "openarm_can"
MIRROR = ROOT / "src" / "controller" / "mirror_teaching_arm.hpp"

STARTUP_SKIP = (
    "等待 CAN 新鲜度检查 PR：遥操 Control::SeedHoldMeasured、零重力 "
    "control/openarm_zero_g.cpp 和 MoveIt real_arm.cpp 在反馈无效时仍会进入出力循环。"
    "解除条件：启动时若反馈年龄超过 "
    f"{CAN_FEEDBACK_MAX_AGE_MS:g} ms、error_code > {CAN_ERROR_CODE_MAX_OK}（待硬件确认）"
    "或从未收到回包，拒绝进入出力循环，且不得按 q=0 发带 Kp 的保持或按 G(0) 算重力。"
    "同向示教已有这条规则，见 test_mirror_threshold_matches_contract。"
)

RUNTIME_SKIP = (
    "等待 CAN 新鲜度检查 PR：遥操、零重力和 MoveIt 实机在运行中没有反馈年龄看门狗。"
    "解除条件：反馈年龄超过 "
    f"{CAN_FEEDBACK_MAX_AGE_MS:g} ms 时停止下发，并走现有 disable_all() 失能路径。"
    "error_code > "
    f"{CAN_ERROR_CODE_MAX_OK} 同样视为无效，这一条待硬件确认。"
)


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

    def test_mirror_threshold_matches_contract(self):
        text = MIRROR.read_text(encoding="utf-8")
        age = int(CAN_FEEDBACK_MAX_AGE_MS)
        self.assertIn(
            f"age_ms > {age} || m.get_error_code() > {CAN_ERROR_CODE_MAX_OK}",
            text,
        )
        self.assertIn("last_state_received().time_since_epoch().count() == 0", text)

    def test_startup_refuses_invalid_feedback(self):
        self.skipTest(STARTUP_SKIP)

    def test_runtime_stale_feedback_disables(self):
        self.skipTest(RUNTIME_SKIP)


if __name__ == "__main__":
    unittest.main()
