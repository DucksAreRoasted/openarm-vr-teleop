"""Optional grippers preserve seven-joint safety and the runtime gravity model."""

import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "script"))
from openarm_app.gravity_identify import latest_two_sided_log
from openarm_app.gravity_knobs import load_params, write_params
from openarm_app.gravity_urdf_seed import install_urdf_gravity_seed

try:
    from PySide6.QtWidgets import QApplication
    QT_AVAILABLE = True
except ImportError:
    QT_AVAILABLE = False


class GravityInstallationTests(unittest.TestCase):
    def test_optional_motor_set_keeps_arm_feedback_strict(self):
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / "installation_test"
            subprocess.run(["c++", "-std=c++17", "-Wall", "-Wextra", "-Wpedantic",
                            "-I", str(ROOT / "src"), "-I", str(ROOT / "third_party/openarm_can/include"),
                            str(ROOT / "tests/gravity_installation_test.cpp"), "-o", str(binary)], check=True)
            subprocess.run([str(binary), directory], check=True)

    def test_save_and_urdf_seed_preserve_installation_and_tuning(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "gravity.yaml"
            shutil.copy2(ROOT / "config/gravity_comp.yaml", path)
            write_params(path, {"left_gripper_present": True, "right_gripper_present": False})
            output = install_urdf_gravity_seed(path, ROOT / "assets/v1.urdf")
            params = load_params(path)
            self.assertEqual((params["left_gripper_present"], params["right_gripper_present"]), (True, False))
            self.assertIn("gripper=removed ee_mass = 0.0000 kg", output)

    def test_runtime_only_removes_right_end_effector_and_preserves_camera(self):
        subprocess.run(["cmake", "--build", str(ROOT / "build"), "--target",
                        "openarm_gravity_model", "-j2"], check=True, stdout=subprocess.DEVNULL)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "gravity.yaml"
            shutil.copy2(ROOT / "config/gravity_comp.yaml", path)

            def evaluate(updates):
                write_params(path, updates)
                q = "0.2,0.5,0.1,0.6,0.1,0.4,0.2"
                result = subprocess.run([str(ROOT / "build/openarm_gravity_model"),
                                         str(ROOT / "assets/v1.urdf"), str(path)],
                                        input=f"left,{q}\nright,{q}\n", text=True,
                                        capture_output=True, check=True)
                return np.array([[float(x) for x in line.split(",")]
                                 for line in result.stdout.splitlines()])

            installed = evaluate({"left_gripper_present": True, "right_gripper_present": True,
                                  "left_wrist_camera_present": True, "right_wrist_camera_present": True})
            removed = evaluate({"right_gripper_present": False})
            np.testing.assert_allclose(removed[0], installed[0])
            self.assertGreater(np.max(np.abs(removed[1] - installed[1])), 0.01)
            reference = evaluate({"right_gripper_present": True, "distal_ee_gain": [0.0] * 7})
            np.testing.assert_allclose(removed[1], reference[1])
            camera_removed = evaluate({"right_gripper_present": False, "right_wrist_camera_present": False,
                                       "distal_ee_gain": load_params(ROOT / "config/gravity_comp.yaml")["distal_ee_gain"]})
            np.testing.assert_allclose(camera_removed[0], removed[0])
            cad_only = evaluate({"wrist_camera_kg": 0.0})
            np.testing.assert_allclose(camera_removed[1], cad_only[1])
            self.assertGreater(np.max(np.abs(removed[1] - camera_removed[1])), 0.01)


class InertialIdentificationConsoleTests(unittest.TestCase):
    def test_latest_two_sided_log_skips_one_sided_logs(self):
        with tempfile.TemporaryDirectory() as directory:
            log_dir = Path(directory)
            (log_dir / "gravity_20260101_000000.csv").write_text("t,approach,tau_g1,ff_gain\n")
            (log_dir / "gravity_20260102_000000.csv").write_text("t,tau_g1,ff_gain\n")
            self.assertEqual(latest_two_sided_log(log_dir).name, "gravity_20260101_000000.csv")
            self.assertIsNone(latest_two_sided_log(log_dir / "missing"))

    @unittest.skipUnless(QT_AVAILABLE, "run with .venv Python for Qt checks")
    def test_button_previews_then_writes_after_confirmation(self):
        from openarm_app.main_window import MainWindow
        from PySide6.QtWidgets import QMessageBox
        app = QApplication.instance() or QApplication([])
        log = Path("/tmp/gravity_test.csv")
        preview = "[left] 配对姿态 9 个\n  当前配置: 0.1\n  本次辨识: 0.05\n其他输出"
        with patch("openarm_app.main_window.local_ip", return_value="127.0.0.1"), \
             patch("openarm_app.main_window.list_usb_cameras", return_value=[]), \
             patch("openarm_app.main_window.latest_two_sided_log", return_value=log), \
             patch("openarm_app.main_window.run_identification",
                   side_effect=[(True, preview), (True, "已写入")]) as run, \
             patch("openarm_app.main_window.QMessageBox.question",
                   return_value=QMessageBox.StandardButton.Yes) as question, \
             patch("openarm_app.main_window.QMessageBox.information") as information, \
             patch("openarm_app.main_window.QMessageBox.warning") as warning:
            window = MainWindow()
            try:
                self.assertFalse(hasattr(window, "left_gripper_present"))
                window._configuration_busy = True
                window.btn_identify.click()
                warning.assert_called_once()
                run.assert_not_called()
                window._configuration_busy = False
                window.btn_identify.click()
                deadline = time.monotonic() + 5
                while not information.called and time.monotonic() < deadline:
                    app.processEvents()
                    time.sleep(0.01)
                self.assertEqual([c.args[2] for c in run.call_args_list], [False, True])
                self.assertIn("本次辨识: 0.05", question.call_args.args[2])
                self.assertNotIn("其他输出", question.call_args.args[2])
                self.assertTrue(window.btn_identify.isEnabled())
            finally:
                window.close()
        app.processEvents()

    @unittest.skipUnless(QT_AVAILABLE, "run with .venv Python for Qt checks")
    def test_auto_calibration_starts_from_current_config_not_template(self):
        from openarm_app.main_window import MainWindow
        from PySide6.QtWidgets import QMessageBox
        app = QApplication.instance() or QApplication([])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "gravity.yaml"
            shutil.copy2(ROOT / "config/gravity_comp.yaml", path)
            with patch("openarm_app.main_window.resolve_gravity_yaml", return_value=path), \
                 patch("openarm_app.main_window.local_ip", return_value="127.0.0.1"), \
                 patch("openarm_app.main_window.list_usb_cameras", return_value=[]), \
                 patch("openarm_app.main_window.QMessageBox.question",
                       return_value=QMessageBox.StandardButton.Yes) as question, \
                 patch("openarm_app.main_window.QMessageBox.warning") as warning, \
                 patch("openarm_app.main_window.threading.Thread") as thread:
                window = MainWindow()
                try:
                    window.radio_zerog.setChecked(True)
                    window.safety.setChecked(True)
                    with patch.object(window, "_ask_sudo", return_value=""):
                        window._auto_calib_gravity()
                    warning.assert_not_called()
                    self.assertIn(str(path), question.call_args.args[2])
                    self.assertNotIn("模板 inf.backup）", question.call_args.args[2])
                    self.assertEqual(thread.call_args.kwargs["args"][-1], path)
                finally:
                    window._configuration_busy = False
                    window.close()
        app.processEvents()
