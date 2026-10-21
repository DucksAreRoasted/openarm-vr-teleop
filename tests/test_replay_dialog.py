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

    def test_quest_profile_reaches_launcher_and_locks_while_running(self):
        with patch.dict(os.environ, {"VR_TELEOP_MODE": ""}), \
             patch("openarm_app.main_window.local_ip", return_value="127.0.0.1"), \
             patch("openarm_app.main_window.list_usb_cameras", return_value=[]):
            window = MainWindow()
        self.assertEqual(window.quest_profile.currentData(), "unitree_style")
        self.assertEqual([window.quest_profile.itemData(i)
                          for i in range(window.quest_profile.count())],
                         ["unitree_style"])
        with tempfile.TemporaryDirectory() as directory:
            window._quest_profile_requested = "pika_style"
            window.log_path = Path(directory) / "control.log"
            with patch("openarm_app.main_window.subprocess.Popen") as spawn:
                window._spawn("sim", "can1", "can0")
                self.assertEqual(spawn.call_args.kwargs["env"]["VR_TELEOP_MODE"], "unitree_style")
            window._close_log()
        window.close()

    def test_recording_metadata_selects_source_gripper_units(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            metadata = path / "collection_mode.json"
            metadata.write_text(json.dumps({"gripper_units": "meters"}))
            dialog = ReplayDialog(source=directory)
            self.assertEqual(dialog.units.currentIndex(), 1)
            metadata.write_text(json.dumps({"gripper_units": "radians"}))
            dialog.source.setText(str(path / "real_teleop_dual_merged.csv"))
            self.assertEqual(dialog.units.currentIndex(), 0)
            dialog.close()

    def feed(self, dialog, text):
        payload = text if text.endswith("\n") else text + "\n"
        with patch.object(dialog.process, "readAllStandardOutput", return_value=payload.encode()):
            dialog._read_output()

    def test_startup_and_connection_show_separate_clocks(self):
        dialog = ReplayDialog()
        self.feed(dialog, "[replay] ALIGNING")
        self.feed(dialog, "[replay] PROGRESS time=0 duration=10 phase=startup preparation_time=2 startup_duration=8 preparation_duration=11")
        self.assertIn("执行起手轨迹  2.0 / 8.0 秒", dialog.status.text())
        self.assertTrue(dialog._aligning)
        self.assertFalse(dialog.start.isEnabled())
        self.feed(dialog, "[replay] PROGRESS time=0 duration=10 phase=connecting preparation_time=9 startup_duration=8 preparation_duration=11")
        self.assertIn("衔接录制首帧  1.0 / 3.0 秒", dialog.status.text())
        dialog.close()

    def test_background_return_preparation_waits_for_manual_play(self):
        dialog = ReplayDialog()
        with patch.object(dialog, "_running", return_value=True):
            self.feed(dialog, "[replay] ALIGNING: preparing return; holding final pose")
            self.feed(dialog, "[replay] PROGRESS time=0 duration=10 phase=preparing")
            self.assertIn("准备返回轨迹，保持末帧", dialog.status.text())
            self.assertFalse(dialog.start.isEnabled())
            self.assertFalse(dialog.prepare.isEnabled())
            self.assertFalse(dialog.progress.isEnabled())
            self.assertTrue(dialog.stop.isEnabled())
            self.feed(dialog, "[replay] ALIGNING: return trajectory ready")
            self.assertFalse(dialog.start.isEnabled())
            self.feed(dialog, "[replay] READY: holding first frame; waiting for play")
            self.assertTrue(dialog.start.isEnabled())
        dialog.close()

    def test_progress_shows_index_over_total_and_unknown_phase(self):
        dialog = ReplayDialog()
        dialog.repeat.setValue(4)
        self.feed(dialog, "[replay] ALIGNING: cycle=2 slow return to first frame\n"
                  "[replay] PROGRESS time=1 duration=4 phase=playing error=0.01")
        self.assertEqual(dialog.status.text(), "第 2 / 4 遍 · 回放中  1.0 / 4.0 秒")
        self.assertEqual(dialog.progress.value(), 250)

        dialog.repeat.setValue(0)
        self.feed(dialog, "[replay] PROGRESS time=0 duration=2 phase=aligning error=0")
        self.assertEqual(dialog.status.text(), "第 2 / ∞ 遍 · 接入录制首帧  0.0 / 2.0 秒")

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

    def test_scrub_sends_seek_on_release_and_preserves_drag_position(self):
        dialog = ReplayDialog()
        with patch.object(dialog, "_running", return_value=True), \
             patch.object(dialog.process, "write") as write:
            self.feed(dialog, "[replay] PLAYING mode=preview\n"
                      "[replay] PROGRESS time=1 duration=10 phase=playing mode=preview")
            write.assert_not_called()
            self.assertTrue(dialog.progress.isEnabled())
            dialog.progress.setSliderDown(True)
            dialog.progress.setSliderPosition(700)
            self.feed(dialog, "[replay] PROGRESS time=2 duration=10 phase=playing mode=preview")
            self.assertEqual(dialog.progress.sliderPosition(), 700)
            write.assert_not_called()
            dialog.progress.setSliderDown(False)
            write.assert_called_once_with(b"seek 7.000000000\n")
            self.feed(dialog, "[replay] PROGRESS time=10 duration=10 phase=complete mode=preview")
            self.assertTrue(dialog.progress.isEnabled())
            self.feed(dialog, "[replay] PROGRESS time=3 duration=10 phase=paused mode=preview")
            self.assertFalse(dialog._complete)
            self.assertTrue(dialog.pause.isEnabled())
            self.assertEqual(dialog.pause.text(), "继续")
            write.reset_mock()
            dialog.mode.setCurrentIndex(1)
            self.assertFalse(dialog.progress.isEnabled())
            dialog.progress.setValue(500)
            write.assert_not_called()
        dialog.close()

    def test_alignment_allows_pause_but_disables_scrubbing(self):
        dialog = ReplayDialog()
        with patch.object(dialog, "_running", return_value=True):
            self.feed(dialog, "[replay] ALIGNING mode=preview transition_seconds=2\n"
                      "[replay] PROGRESS time=0 duration=10 phase=aligning mode=preview")
            self.assertTrue(dialog.pause.isEnabled())
            self.assertFalse(dialog.progress.isEnabled())
            self.feed(dialog, "[replay] PROGRESS time=0 duration=10 phase=paused mode=preview")
            self.assertFalse(dialog.progress.isEnabled())
            self.feed(dialog, "[replay] PLAYING mode=preview\n"
                      "[replay] PROGRESS time=0 duration=10 phase=playing mode=preview")
            self.assertTrue(dialog.progress.isEnabled())
        dialog.close()

    def test_prepare_then_play_and_repeat_in_same_process_for_preview_and_real(self):
        for mode in (0, 1):
            with self.subTest(mode=mode):
                dialog = ReplayDialog()
                dialog.mode.setCurrentIndex(mode)
                dialog.safety.setChecked(True)
                with patch.object(dialog, "_running", return_value=True), \
                     patch.object(dialog.process, "write", side_effect=lambda data: len(data)) as write, \
                     patch.object(dialog.process, "start") as spawn:
                    self.feed(dialog, "[replay] PLAYING\n"
                              "[replay] COMPLETE: holding final pose\n"
                              "[replay] PROGRESS time=10 duration=10 phase=complete")
                    self.assertFalse(dialog.start.isEnabled())
                    self.assertTrue(dialog.prepare.isEnabled())
                    self.assertEqual(dialog.prepare.text(), "返回录制首帧")
                    self.assertFalse(dialog.pause.isEnabled())
                    self.assertTrue(dialog.stop.isEnabled())
                    dialog.prepare.click()
                    dialog._prepare()
                    dialog._start()
                    write.assert_called_once_with(b"prepare\n")
                    spawn.assert_not_called()
                    self.assertFalse(dialog.start.isEnabled())
                    self.assertFalse(dialog.pause.isEnabled())
                    self.assertFalse(dialog.progress.isEnabled())
                    self.assertEqual(dialog.progress.value(), 0)
                    self.feed(dialog, "[replay] PROGRESS time=10 duration=10 phase=complete")
                    self.assertFalse(dialog.start.isEnabled())
                    self.feed(dialog, "[replay] ALIGNING: cycle=1 slow return to first frame")
                    self.assertTrue(dialog._aligning)
                    dialog._start()
                    write.assert_called_once_with(b"prepare\n")
                    self.assertFalse(dialog._complete)
                    self.assertTrue(dialog.pause.isEnabled())
                    self.feed(dialog, "[replay] READY: holding first frame; waiting for play\n"
                              "[replay] PROGRESS time=0 duration=10 phase=ready")
                    self.assertTrue(dialog.start.isEnabled())
                    self.assertFalse(dialog.prepare.isEnabled())
                    self.assertFalse(dialog.pause.isEnabled())
                    self.assertFalse(dialog.progress.isEnabled())
                    dialog.start.click()
                    dialog._start()
                    self.assertEqual(write.call_args_list[-1].args, (b"play\n",))
                    self.assertEqual(write.call_count, 2)
                    self.assertFalse(dialog.start.isEnabled())
                    self.feed(dialog, "[replay] PROGRESS time=0 duration=10 phase=ready")
                    self.assertFalse(dialog.start.isEnabled())
                    self.feed(dialog, "[replay] PLAYING\n"
                              "[replay] PROGRESS time=1 duration=10 phase=playing")
                    self.assertTrue(dialog.pause.isEnabled())
                    self.feed(dialog, "[replay] COMPLETE: holding final pose")
                    self.assertFalse(dialog.start.isEnabled())
                    self.assertTrue(dialog.prepare.isEnabled())
                    dialog._stopping = True
                    dialog._update_controls()
                    self.assertFalse(dialog.start.isEnabled())
                with patch.object(dialog.process, "readAllStandardOutput", return_value=b""):
                    dialog._finished(0, "NormalExit")
                self.assertFalse(dialog.start.isEnabled())
                self.assertTrue(dialog.prepare.isEnabled())
                self.assertEqual(dialog.start.text(), "开始回放")
                dialog.close()

    def test_repeat_editable_at_endpoints_and_sent_before_next_play(self):
        dialog = ReplayDialog()
        with patch.object(dialog, "_running", return_value=True), \
             patch.object(dialog.process, "write", side_effect=lambda data: len(data)) as write:
            self.feed(dialog, "[replay] PLAYING")
            self.assertFalse(dialog.repeat.isEnabled())
            self.feed(dialog, "[replay] COMPLETE: holding final pose")
            self.assertTrue(dialog.repeat.isEnabled())
            self.assertTrue(dialog.forever.isEnabled())
            self.assertFalse(dialog.source.isEnabled())
            dialog.repeat.setValue(3)
            dialog.prepare.click()
            self.assertFalse(dialog.repeat.isEnabled())
            self.feed(dialog, "[replay] ALIGNING\n[replay] READY: holding first frame")
            self.assertTrue(dialog.repeat.isEnabled())
            dialog.start.click()
            self.assertEqual(write.call_args.args, (b"repeat 3\nplay\n",))
            self.assertFalse(dialog.repeat.isEnabled())
            self.feed(dialog, "[replay] PLAYING\n[replay] COMPLETE: holding final pose")
            dialog.forever.setChecked(True)
            self.assertEqual(dialog.repeat.value(), 0)
            dialog.prepare.click()
            self.feed(dialog, "[replay] ALIGNING\n[replay] READY: holding first frame")
            dialog.start.click()
            self.assertEqual(write.call_args.args, (b"repeat 0\nplay\n",))
        dialog.close()

    def test_end_infinite_loop_preserves_pause_and_can_restart_infinite(self):
        dialog = ReplayDialog()
        dialog.repeat.setValue(0)
        dialog._active_repeat = 0
        with patch.object(dialog, "_running", return_value=True), \
             patch.object(dialog.process, "write", side_effect=lambda data: len(data)) as write:
            self.feed(dialog, "[replay] ALIGNING")
            self.assertFalse(dialog.finish_loop.isEnabled())
            self.feed(dialog, "[replay] PLAYING\n[replay] PROGRESS time=1 duration=10 phase=paused")
            self.assertTrue(dialog.finish_loop.isEnabled())
            dialog.finish_loop.click()
            write.assert_called_once_with(b"finish_loop\n")
            self.assertFalse(dialog.finish_loop.isEnabled())
            self.assertTrue(dialog._paused)
            self.assertTrue(dialog.pause.isEnabled())
            self.assertTrue(dialog.stop.isEnabled())
            self.feed(dialog, "[replay] COMPLETE: holding final pose")
            self.assertTrue(dialog.prepare.isEnabled())
            self.assertFalse(dialog.finish_loop.isEnabled())
            dialog.prepare.click()
            self.feed(dialog, "[replay] ALIGNING\n[replay] READY: holding first frame")
            dialog.start.click()
            self.assertEqual(write.call_args.args, (b"repeat 0\nplay\n",))
            self.feed(dialog, "[replay] PLAYING")
            self.assertTrue(dialog.finish_loop.isEnabled())
        dialog.close()

    def test_failed_end_loop_command_can_be_retried(self):
        dialog = ReplayDialog()
        dialog._active_repeat = 0
        with patch.object(dialog, "_running", return_value=True), \
             patch.object(dialog.process, "write", return_value=-1):
            self.feed(dialog, "[replay] PLAYING")
            dialog.finish_loop.click()
            self.assertFalse(dialog._finish_requested)
            self.assertTrue(dialog.finish_loop.isEnabled())
        dialog.close()

    def test_failed_prepare_write_keeps_final_pose_state(self):
        dialog = ReplayDialog()
        with patch.object(dialog, "_running", return_value=True), \
             patch.object(dialog.process, "write", return_value=-1):
            self.feed(dialog, "[replay] COMPLETE: holding final pose")
            dialog.prepare.click()
            self.assertTrue(dialog._complete)
            self.assertIsNone(dialog._pending_action)
            self.assertIn("发送失败", dialog.status.text())
        dialog.close()

    def test_initial_prepare_launches_with_manual_start_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "trajectory.csv"
            source.touch()
            dialog = ReplayDialog(source=str(source))
            self.assertTrue(dialog.prepare.isEnabled())
            self.assertFalse(dialog.start.isEnabled())
            with patch.object(dialog.process, "start") as spawn:
                dialog._start()
                spawn.assert_not_called()
                dialog.prepare.click()
                self.assertIn("--wait-for-start", spawn.call_args.args[1])
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
