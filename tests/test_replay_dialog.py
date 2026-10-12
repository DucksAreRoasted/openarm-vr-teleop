"""Replay process-group signalling and exclusive controller ownership."""
import os
import json
from pathlib import Path
import signal
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "script"))
try:
    from PySide6.QtWidgets import QApplication
    from openarm_app.main_window import MainWindow
    from openarm_app.replay_dialog import ReplayDialog
    QT_AVAILABLE = True
except ImportError:
    QT_AVAILABLE = False


@unittest.skipUnless(QT_AVAILABLE, "run with .venv Python for Qt checks")
class ReplayDialogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_never_signal_zero_or_parent_process_group(self):
        dialog = ReplayDialog()
        with patch.object(dialog, "_running", return_value=True), \
             patch.object(dialog.process, "processId", return_value=0), \
             patch("openarm_app.replay_dialog.os.killpg") as kill, \
             patch("openarm_app.replay_dialog.QTimer.singleShot") as retry:
            dialog._signal(signal.SIGINT)
            kill.assert_not_called()
            retry.assert_called_once()
        with patch.object(dialog, "_running", return_value=True), \
             patch.object(dialog.process, "processId", return_value=123), \
             patch("openarm_app.replay_dialog.os.getpgid", return_value=456), \
             patch("openarm_app.replay_dialog.os.killpg") as kill, \
             patch("openarm_app.replay_dialog.QTimer.singleShot") as retry:
            dialog._signal(signal.SIGINT)
            kill.assert_not_called()
            retry.assert_called_once()
        dialog.close()

    def test_console_blocks_replay_during_control_or_startup(self):
        with patch("openarm_app.main_window.local_ip", return_value="127.0.0.1"), \
             patch("openarm_app.main_window.list_usb_cameras", return_value=[]):
            window = MainWindow()
        with patch("openarm_app.main_window.ReplayDialog") as dialog, \
             patch("openarm_app.main_window.QMessageBox.warning") as warning:
            window.proc = Mock()
            window.proc.poll.return_value = None
            window._open_replay()
            dialog.assert_not_called()
            warning.assert_called_once()
            window.proc = None
            warning.reset_mock()
            window.btn_start.setEnabled(False)
            window._open_replay()
            dialog.assert_not_called()
            warning.assert_called_once()
        window.close()

    def test_log_survives_exit_including_unterminated_error(self):
        dialog = ReplayDialog()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "replay.log"
            dialog._log_path = path
            with patch.object(dialog.process, "readAllStandardOutput", side_effect=[
                b"[replay] ALIGNING\n[replay][ERROR] tracking failure", b"",
            ]):
                dialog._read_output()
                dialog._finished(1, "NormalExit")
            text = path.read_text()
            self.assertIn("[replay] ALIGNING\n", text)
            self.assertIn("[replay][ERROR] tracking failure\n", text)
            self.assertIn("EXIT code=1", text)
            self.assertEqual(dialog.status.text(), "[replay][ERROR] tracking failure")
        dialog.close()

    def test_ker_modes_select_hardware_and_wait_for_alignment_before_recording(self):
        with patch("openarm_app.main_window.local_ip", return_value="127.0.0.1"), \
             patch("openarm_app.main_window.list_usb_cameras", return_value=[]):
            window = MainWindow()
        with patch.object(window, "_running", return_value=True):
            for radio, mode, hardware in ((window.radio_ker_sim, "ker_sim", False),
                                          (window.radio_ker_real, "ker_real", True)):
                radio.setChecked(True)
                self.assertEqual(window._mode(), mode)
                self.assertEqual(window._hardware_mode(), hardware)
                window._mirror_ready = False
                self.assertFalse(window._record_available())
                window._mirror_ready = True
                self.assertTrue(window._record_available())
        window.close()

    def test_ker_recording_metadata_selects_source_gripper_units(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            metadata = path / "collection_mode.json"
            metadata.write_text(json.dumps({"mode": "ker_v1", "gripper_units": "meters"}))
            dialog = ReplayDialog(source=directory)
            self.assertEqual(dialog.units.currentIndex(), 1)
            metadata.write_text(json.dumps({"mode": "ker_v1", "gripper_units": "radians"}))
            dialog.source.setText(str(path / "real_teleop_dual_merged.csv"))
            self.assertEqual(dialog.units.currentIndex(), 0)
            dialog.close()


if __name__ == "__main__":
    unittest.main()
