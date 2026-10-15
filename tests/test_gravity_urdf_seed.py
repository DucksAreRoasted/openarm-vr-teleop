"""URDF gravity reset must be offline, validated before replacing, and gated in the UI."""

import os
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "script"))

from openarm_app.gravity_knobs import load_params  # noqa: E402
from openarm_app.gravity_urdf_seed import install_urdf_gravity_seed  # noqa: E402

try:
    from PySide6.QtWidgets import QApplication, QMessageBox
    from openarm_app.main_window import MainWindow
    QT_AVAILABLE = True
except ImportError:
    QT_AVAILABLE = False


class _GravitySeedFixture:
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.yaml = Path(self.folder.name) / "gravity_comp.yaml"
        shutil.copy2(ROOT / "config" / "gravity_comp.yaml", self.yaml)
        self.original = self.yaml.read_bytes()


class GravityUrdfSeedTests(_GravitySeedFixture, unittest.TestCase):

    def test_install_overwrites_without_backup_and_preserves_accessories(self):
        before = load_params(self.yaml)
        output = install_urdf_gravity_seed(self.yaml, ROOT / "assets" / "v1.urdf")
        after = load_params(self.yaml)
        self.assertEqual([p.name for p in Path(self.folder.name).iterdir()], [self.yaml.name])
        self.assertIn("G_urdf(q)", output)
        self.assertEqual(after["link_mass_scale"], [1.0] * 7)
        for side in ("left", "right"):
            self.assertEqual(after[f"{side}_link_mass_scale"], [1.0] * 7)
            self.assertEqual(after[f"{side}_link_com_delta"], [0.0] * 21)
            self.assertEqual(after[f"{side}_gravity_scale"], [1.0] * 7)
            self.assertEqual(after[f"{side}_gravity_bias"], [0.0] * 7)
            self.assertEqual(after[f"{side}_friction_fc"], before[f"{side}_friction_fc"])
        self.assertEqual(after["urdf_gravity_scale"], 1.0)
        self.assertEqual(after["g_scale"], 0.95)
        self.assertEqual(after["ff_gain"], 0.95)
        for key in ("ee_payload_kg", "wrist_camera_kg", "left_wrist_camera_present", "right_wrist_camera_present", "distal_cam_gain"):
            self.assertEqual(after[key], before[key])

    def test_missing_urdf_leaves_original_untouched(self):
        with self.assertRaises(FileNotFoundError):
            install_urdf_gravity_seed(self.yaml, Path(self.folder.name) / "missing.urdf")
        self.assertEqual(self.yaml.read_bytes(), self.original)
        self.assertEqual(list(Path(self.folder.name).glob("*.bak.urdf.*")), [])


@unittest.skipUnless(QT_AVAILABLE, "run with .venv Python for Qt checks")
class GravityUrdfButtonTests(_GravitySeedFixture, unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_button_writes_staged_seed_and_blocks_running_controller(self):
        with patch("openarm_app.main_window.local_ip", return_value="127.0.0.1"), \
             patch("openarm_app.main_window.list_usb_cameras", return_value=[]):
            window = MainWindow()
        try:
            with patch("openarm_app.main_window.resolve_gravity_yaml", return_value=self.yaml), \
                 patch("openarm_app.main_window.QMessageBox.question", return_value=QMessageBox.StandardButton.Yes), \
                 patch("openarm_app.main_window.QMessageBox.information") as info, \
                 patch("openarm_app.main_window.QMessageBox.warning") as warning:
                window.proc = Mock()
                window.proc.poll.return_value = None
                window._seed_gravity_from_urdf()
                self.assertEqual(self.yaml.read_bytes(), self.original)
                warning.assert_called_once()
                window.proc = None
                window._seed_gravity_from_urdf()
                info.assert_called_once()
            self.assertEqual(load_params(self.yaml)["urdf_gravity_scale"], 1.0)
        finally:
            window.close()
