"""human_mimic clutch behavior against the real controller.

The binary is built by CMake (`human_mimic_clutch_test`) and links
vr_control_core. It does not open CAN. CI fails if the binary is missing;
a local tree without a build skips.
"""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]
BINARY = ROOT / "build" / "human_mimic_clutch_test"
URDF = ROOT / "assets" / "v1.urdf"


class HumanMimicClutchTests(unittest.TestCase):
    def test_timeout_and_ik_failure_reengage_while_grip_held(self):
        if not BINARY.is_file():
            message = (
                "未编译 human_mimic_clutch_test。"
                "在已安装 KDL / kdl_parser / URDF 的环境里执行 "
                "cmake --build build --target human_mimic_clutch_test"
            )
            if os.environ.get("CI"):
                self.fail(message)
            self.skipTest(message)
        completed = subprocess.run(
            [str(BINARY), str(URDF)],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(
            completed.returncode,
            0,
            completed.stdout + completed.stderr,
        )
        self.assertIn("human_mimic_clutch_test: ok", completed.stdout)


if __name__ == "__main__":
    unittest.main()
