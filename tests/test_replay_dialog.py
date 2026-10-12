"""Replay process-group signalling and exclusive controller ownership."""
import os
import json
from pathlib import Path
import signal
import subprocess
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

    def feed(self, dialog, text):
        payload = text if text.endswith("\n") else text + "\n"
        with patch.object(dialog.process, "readAllStandardOutput", return_value=payload.encode()):
            dialog._read_output()

    def test_progress_shows_index_over_total_and_unknown_phase(self):
        dialog = ReplayDialog()
        dialog.repeat.setValue(4)
        self.feed(dialog, "[replay] ALIGNING: cycle=2 slow return to first frame\n"
                  "[replay] PROGRESS time=1 duration=4 phase=playing error=0.01")
        self.assertEqual(dialog.status.text(), "第 2 / 4 遍 · 回放中  1.0 / 4.0 秒")
        self.assertEqual(dialog.progress.value(), 250)

        dialog.repeat.setValue(0)
        self.feed(dialog, "[replay] PROGRESS time=0 duration=2 phase=aligning error=0")
        self.assertEqual(dialog.status.text(), "第 2 / ∞ 遍 · 接入首帧  0.0 / 2.0 秒")

        self.feed(dialog, "[replay] PROGRESS time=0.5 duration=2 phase=paused error=0 loop=3 loops=10")
        self.assertEqual(dialog.status.text(), "第 3 / 10 遍 · 已暂停  0.5 / 2.0 秒")
        self.assertEqual(dialog._cycle, 3)

        self.feed(dialog, "[replay] PROGRESS time=0 duration=1 phase=playing loop=4 loops=inf")
        self.assertEqual(dialog.status.text(), "第 4 / ∞ 遍 · 回放中  0.0 / 1.0 秒")
        self.feed(dialog, "[replay] PROGRESS time=0 duration=1 phase=playing loop = 1 loops = 0")
        self.assertEqual(dialog.status.text(), "第 1 / ∞ 遍 · 回放中  0.0 / 1.0 秒")
        self.feed(dialog, "[replay] PROGRESS time=0 duration=1 phase=playing loop=2 loops=Inf extra")
        self.assertEqual(dialog.status.text(), "第 2 / ∞ 遍 · 回放中  0.0 / 1.0 秒")

        dialog.repeat.setValue(8)
        self.feed(dialog, "[replay] PROGRESS time=0 duration=1 phase=complete loops=7")
        self.assertEqual(dialog.status.text(), "第 2 / 7 遍 · 已完成，保持末帧  0.0 / 1.0 秒")
        self.feed(dialog, "[replay] PROGRESS time=0 duration=1 phase=playing loop=6")
        self.assertEqual(dialog.status.text(), "第 6 / 8 遍 · 回放中  0.0 / 1.0 秒")
        self.feed(dialog, "[replay] PROGRESS time=0 duration=1 phase=playing loops=abc loop=9")
        self.assertEqual(dialog.status.text(), "第 9 / 8 遍 · 回放中  0.0 / 1.0 秒")

        self.feed(dialog, "[replay] PROGRESS time=0.2 duration=1 phase=returning error=0.4")
        self.assertEqual(dialog.status.text(), "第 9 / 8 遍 · returning  0.2 / 1.0 秒")
        self.feed(dialog, "[replay] PROGRESS time=0 duration=1 phase=mystery")
        self.assertIn("mystery", dialog.status.text())
        dialog.close()


class ImmediateThread:
    def __init__(self, target=None, daemon=None):
        self._target = target

    def start(self):
        self._target()


@unittest.skipUnless(QT_AVAILABLE, "run with .venv Python for Qt checks")
class StopTimeoutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def make_window(self):
        with patch("openarm_app.main_window.local_ip", return_value="127.0.0.1"), \
             patch("openarm_app.main_window.list_usb_cameras", return_value=[]):
            return MainWindow()

    def test_stop_timeout_keeps_process_and_reenables_stop(self):
        window = self.make_window()
        proc = Mock()
        proc.pid = 4242
        proc.poll.return_value = None
        proc.wait.side_effect = [
            subprocess.TimeoutExpired("teleop", 8),
            subprocess.TimeoutExpired("teleop", 3),
        ]
        window.proc = proc
        window._quest_ready = True
        window.status.setText("仿真运行中")
        window.btn_stop.setEnabled(True)

        seen = {}

        class ObserveThread(ImmediateThread):
            def start(self):
                seen["status"] = window.status.text()
                seen["enabled"] = window.btn_stop.isEnabled()
                super().start()

        with patch("openarm_app.main_window.os.killpg") as kill, \
             patch("openarm_app.main_window.threading.Thread", ObserveThread):
            window._stop()
        self.assertEqual(seen, {"status": "正在释放…", "enabled": False})
        self.assertEqual(window.status.text(), "释放超时，进程仍在运行。可再次停止。")
        self.assertTrue(window.btn_stop.isEnabled())
        self.assertIs(window.proc, proc)
        self.assertTrue(window._quest_ready)
        self.assertEqual([call.args for call in kill.call_args_list], [
            (4242, signal.SIGINT), (4242, signal.SIGTERM),
        ])
        window.proc = None
        window.close()

    def test_stop_reaches_idle_when_process_exits(self):
        window = self.make_window()
        proc = Mock()
        proc.pid = 4242
        proc.poll.return_value = None
        proc.wait.side_effect = [subprocess.TimeoutExpired("teleop", 8), 0]
        window.proc = proc
        window._quest_ready = True
        with patch("openarm_app.main_window.os.killpg") as kill, \
             patch("openarm_app.main_window.threading.Thread", ImmediateThread):
            window._stop()
        self.assertEqual(window.status.text(), "已停止")
        self.assertIsNone(window.proc)
        self.assertFalse(window.btn_stop.isEnabled())
        self.assertFalse(window._quest_ready)
        self.assertEqual(kill.call_count, 2)
        window.close()

        window = self.make_window()
        proc = Mock()
        proc.pid = 99
        proc.poll.return_value = None
        proc.wait.return_value = 0
        window.proc = proc
        with patch("openarm_app.main_window.os.killpg", side_effect=ProcessLookupError), \
             patch("openarm_app.main_window.threading.Thread", ImmediateThread):
            window._stop()
        self.assertEqual(window.status.text(), "已停止")
        self.assertIsNone(window.proc)
        window.close()


if __name__ == "__main__":
    unittest.main()
