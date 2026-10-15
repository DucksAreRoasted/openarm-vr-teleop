"""ROS command checks without connecting to robot hardware."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "script"))
from openarm_app import services


class MoveItServiceTests(unittest.TestCase):
    def test_trigger_response_controls_success(self):
        cases = (
            (0, "std_srvs.srv.Trigger_Response(success=True, message='held')", True),
            (0, "std_srvs.srv.Trigger_Response(success=False, message='rejected')", False),
            (0, "success: true\nmessage: held", True),
            (0, "waiting for service to become available...", False),
            (1, "std_srvs.srv.Trigger_Response(success=True, message='held')", False),
        )
        for code, output, expected in cases:
            with self.subTest(output=output, code=code):
                result = subprocess.CompletedProcess([], code, output, "")
                with patch.object(services.subprocess, "run", return_value=result):
                    ok, detail = services.run_ros2(
                        ["service", "call", "/real_arm/soft_stop", "std_srvs/srv/Trigger"]
                    )
                self.assertEqual(ok, expected)
                self.assertEqual(detail, output)

    @unittest.skipUnless(Path("/opt/ros/humble/setup.bash").is_file(), "ROS Humble required")
    def test_workspace_setup_can_read_unset_optional_variables(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            setup = root / "ros2/install/setup.bash"
            setup.parent.mkdir(parents=True)
            setup.write_text(
                'if [ -n "${OPENARM_TEST_OPTIONAL_SETUP_FLAG}" ]; then :; fi\n'
                "ros2() { printf '%s\\n' 'success: true' 'message: offline test'; }\n"
            )
            env = os.environ.copy()
            env.pop("OPENARM_TEST_OPTIONAL_SETUP_FLAG", None)
            with patch.object(services, "ROOT", root), patch.dict(os.environ, env, clear=True):
                ok, output = services.run_ros2(
                    ["service", "call", "/real_arm/soft_stop", "std_srvs/srv/Trigger"]
                )
            self.assertTrue(ok, output)
            self.assertIn("offline test", output)


if __name__ == "__main__":
    unittest.main()
