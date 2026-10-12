"""Capture gestures and the console's state-gated UDP shortcut interface."""
import json
import os
from pathlib import Path
import socket
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "quest"))
sys.path.insert(0, str(ROOT / "script"))
from capture_shortcuts import CaptureShortcuts

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
try:
    from PySide6.QtWidgets import QApplication, QMessageBox
    from openarm_app.main_window import MainWindow
except ImportError:
    MainWindow = None


class GestureTests(unittest.TestCase):
    def test_short_long_and_held_press(self):
        for hand in ("left", "right"):
            for duration, suffix in ((.2, "short"), (1.2, "long")):
                gestures = CaptureShortcuts()
                gestures.update(hand, {"stickPressed": False}, 0)
                self.assertIsNone(gestures.update(hand, {"stickPressed": True}, .1))
                instant = .1
                while instant < .1 + duration:
                    instant = min(.1 + duration, instant + .1)
                    self.assertIsNone(gestures.update(hand, {"stickPressed": True}, instant))
                self.assertEqual(gestures.update(hand, {"stickPressed": False}, instant+.01), f"{hand}-{suffix}")
                self.assertIsNone(gestures.update(hand, {"stickPressed": False}, instant+.02))

    def test_startup_missing_tracking_and_gap_cancel_press(self):
        for missing in (None, {}, {"stickPressed": True}):
            gestures = CaptureShortcuts()
            gestures.update("left", {"stickPressed": False}, 0)
            gestures.update("left", {"stickPressed": True}, .1)
            gestures.update("left", missing, .5)
            self.assertIsNone(gestures.update("left", {"stickPressed": False}, .6))
        gestures = CaptureShortcuts()
        gestures.update("right", {"stickPressed": True}, 0)
        self.assertIsNone(gestures.update("right", {"stickPressed": False}, .1))


@unittest.skipIf(MainWindow is None, "Qt unavailable")
class ConsoleShortcutTests(unittest.TestCase):
    def setUp(self):
        self.app = QApplication.instance() or QApplication([])
        with patch.dict(os.environ, {"OPENARM_QUEST_CAPTURE_PORT": "0"}), \
             patch("openarm_app.main_window.local_ip", return_value="127.0.0.1"), \
             patch("openarm_app.main_window.list_usb_cameras", return_value=[]):
            self.window = MainWindow()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

    def tearDown(self):
        self.sock.close()
        self.window._poll_timer.stop()
        # Avoid the quit confirmation, which waits forever offscreen.
        self.window.proc = None
        self.window._capture_socket.close()
        self.window.close()

    def request(self, action):
        self.sock.sendto(
            json.dumps({"action": action}).encode(),
            ("127.0.0.1", self.window._capture_socket.localPort()),
        )
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            self.app.processEvents()
            try:
                return json.loads(self.sock.recv(4096))
            except BlockingIOError:
                time.sleep(.001)
        self.fail("no console reply")

    def run_teleop(self, *, ready=True):
        self.window.proc = Mock()
        self.window.proc.poll.return_value = None
        self.window._quest_ready = ready
        self.window._apply_record_state()

    def test_record_label_redo_and_reject_invalid_states(self):
        window = self.window
        with patch("openarm_app.main_window.send_record") as send, \
             patch("openarm_app.main_window.QMessageBox.question") as question:
            self.run_teleop()
            window.batch_edit.setText("shortcut_test")
            self.assertFalse(self.request("right-short")["ok"])
            self.assertTrue(self.request("left-short")["ok"])
            self.assertTrue(window._recording)
            window._apply_record_state()
            self.assertFalse(window.rec_start.isEnabled())
            self.assertTrue(window.rec_stop.isEnabled())
            while_recording = self.request("left-long")
            self.assertFalse(while_recording["ok"])
            self.assertNotEqual(while_recording["message"], "再长按一次确认重录(5 秒内)")
            self.assertFalse(self.request("right-long")["ok"])
            self.assertTrue(self.request("left-short")["ok"])
            self.assertFalse(self.request("left-short")["ok"])
            self.assertTrue(self.request("right-short")["ok"])
            armed = self.request("left-long")
            self.assertFalse(armed["ok"])
            self.assertEqual(armed["message"], "再长按一次确认重录(5 秒内)")
            self.assertTrue(self.request("left-long")["ok"])
            self.assertFalse(self.request("left-long")["ok"])
            self.assertTrue(self.request("left-short")["ok"])
            self.assertTrue(self.request("left-short")["ok"])
            self.assertTrue(self.request("right-long")["ok"])
            self.assertEqual([call.args[0] for call in send.call_args_list], [
                "start,shortcut_test", "stop", "success", "redo",
                "start,shortcut_test", "stop", "fail"])
            question.assert_not_called()
        window.proc = None
        self.assertFalse(self.request("left-short")["ok"])
        self.assertEqual(
            self.request("left-short")["message"],
            "当前不能执行此采集操作，请检查录制状态。",
        )

    def test_controller_redo_arms_then_confirms_after_stop_success_and_fail(self):
        window = self.window
        self.run_teleop()
        window.batch_edit.setText("redo_batch")
        with patch("openarm_app.main_window.send_record") as send, \
             patch("openarm_app.main_window.QMessageBox.question") as question:
            self.assertTrue(self.request("left-short")["ok"])
            self.assertTrue(self.request("left-short")["ok"])
            self.assertTrue(window.rec_redo.isEnabled())
            first = self.request("left-long")
            self.assertFalse(first["ok"])
            self.assertEqual(first["message"], "再长按一次确认重录(5 秒内)")
            window._redo_armed_until = time.monotonic() - 0.01
            expired = self.request("left-long")
            self.assertFalse(expired["ok"])
            self.assertEqual(expired["message"], "再长按一次确认重录(5 秒内)")
            self.assertNotIn("redo", [call.args[0] for call in send.call_args_list])
            confirmed = self.request("left-long")
            self.assertTrue(confirmed["ok"])
            self.assertFalse(window._can_redo)

            self.assertTrue(self.request("left-short")["ok"])
            self.assertTrue(self.request("left-short")["ok"])
            self.assertTrue(self.request("right-short")["ok"])
            self.assertTrue(window.rec_redo.isEnabled())
            self.assertFalse(self.request("left-long")["ok"])
            self.assertTrue(self.request("left-long")["ok"])

            self.assertTrue(self.request("left-short")["ok"])
            self.assertTrue(self.request("left-short")["ok"])
            self.assertTrue(self.request("right-long")["ok"])
            self.assertTrue(window.rec_redo.isEnabled())
            self.assertFalse(self.request("left-long")["ok"])
            self.assertTrue(self.request("left-long")["ok"])
            question.assert_not_called()
            self.assertEqual([call.args[0] for call in send.call_args_list], [
                "start,redo_batch", "stop", "redo",
                "start,redo_batch", "stop", "success", "redo",
                "start,redo_batch", "stop", "fail", "redo",
            ])

    def test_other_gesture_cancels_armed_redo(self):
        window = self.window
        self.run_teleop()
        window.batch_edit.setText("redo_batch")
        with patch("openarm_app.main_window.send_record") as send:
            self.assertTrue(self.request("left-short")["ok"])
            self.assertTrue(self.request("left-short")["ok"])
            self.assertFalse(self.request("left-long")["ok"])
            self.assertTrue(self.request("right-short")["ok"])
            again = self.request("left-long")
            self.assertFalse(again["ok"])
            self.assertEqual(again["message"], "再长按一次确认重录(5 秒内)")
            self.assertNotIn("redo", [call.args[0] for call in send.call_args_list])

    def test_redo_button_confirms_before_discard(self):
        window = self.window
        self.run_teleop()
        window.batch_edit.setText("button_batch")
        with patch("openarm_app.main_window.send_record") as send:
            self.assertTrue(window._record("start"))
            self.assertTrue(window._record("stop"))
            with patch("openarm_app.main_window.QMessageBox.question",
                       return_value=QMessageBox.StandardButton.No) as question:
                window.rec_redo.click()
            question.assert_called_once()
            self.assertEqual(question.call_args.args[1], "丢弃重录")
            self.assertTrue(window._can_redo)
            self.assertNotIn("redo", [call.args[0] for call in send.call_args_list])
            with patch("openarm_app.main_window.QMessageBox.question",
                       return_value=QMessageBox.StandardButton.Yes):
                window.rec_redo.click()
            self.assertFalse(window._can_redo)
            self.assertEqual(send.call_args_list[-1].args[0], "redo")

            self.assertTrue(window._record("start"))
            self.assertTrue(window._record("stop"))
            self.assertTrue(window._record("success"))
            self.assertTrue(window.rec_redo.isEnabled())
            with patch("openarm_app.main_window.QMessageBox.question",
                       return_value=QMessageBox.StandardButton.No):
                self.assertFalse(window._record("redo"))
            self.assertTrue(window._can_redo)
            with patch("openarm_app.main_window.QMessageBox.question",
                       return_value=QMessageBox.StandardButton.Yes):
                self.assertTrue(window._record("redo"))
            self.assertFalse(window._can_redo)
            self.assertEqual(window.record_hint.text(), "这一条已丢弃。再次开始会用同一个序号。")

    def test_startup_blocks_recording_until_quest_banner(self):
        window = self.window
        with tempfile.TemporaryDirectory() as directory:
            window.log_path = Path(directory) / "app.log"
            window.log_path.write_text("booting\n", encoding="utf-8")
            window.proc = Mock()
            window.proc.poll.return_value = None
            window._started()
            window._poll_timer.stop()
            self.assertEqual(window.status.text(), "启动中…")
            self.assertFalse(window._quest_ready)
            self.assertFalse(window.rec_start.isEnabled())
            self.assertFalse(window._record("start"))
            self.assertEqual(window.record_hint.text(), "遥操尚未就绪，暂时不能录制。")
            for action in ("left-short", "left-long", "right-short", "right-long"):
                reply = self.request(action)
                self.assertFalse(reply["ok"])
                self.assertEqual(reply["message"], "遥操尚未就绪，暂时不能录制。")

            window.log_path.write_text(
                "see [quest-cartesian] bimanual running.\n[quest-cartesian] running. extra\n",
                encoding="utf-8",
            )
            window._refresh_log()
            self.assertFalse(window._quest_ready)
            self.assertEqual(window.status.text(), "启动中…")

            window._quest_skeleton_requested = True
            window.log_path.write_text("ready\n[quest-cartesian] bimanual running.\n", encoding="utf-8")
            window._refresh_log()
            self.assertTrue(window._quest_ready)
            self.assertEqual(window.status.text(), "仿真运行中")
            self.assertTrue(window.rec_start.isEnabled())
            self.assertIn("肘腕模式", window.record_hint.text())

            window.radio_real.setChecked(True)
            window._quest_ready = False
            window._quest_skeleton_requested = False
            window.status.setText("启动中…")
            window.log_path.write_text("[quest-cartesian] running.\n", encoding="utf-8")
            window._refresh_log()
            self.assertEqual(window.status.text(), "真机运行中")
            self.assertTrue(window._record_available())

            window.proc.poll.return_value = 7
            window._poll()
            self.assertFalse(window._quest_ready)
            self.assertIsNone(window.proc)
            self.assertIn("返回码 7", window.status.text())
            self.assertFalse(window.rec_start.isEnabled())

    def test_start_action_shows_pending_and_blocks_shortcuts(self):
        with patch("openarm_app.main_window.threading.Thread") as thread:
            self.window._start()
        self.assertEqual(self.window.status.text(), "正在启动…")
        self.assertFalse(self.window.btn_start.isEnabled())
        thread.return_value.start.assert_called_once()
        reply = self.request("left-short")
        self.assertFalse(reply["ok"])
        self.assertEqual(reply["message"], "遥操尚未就绪，暂时不能录制。")
        self.assertFalse(self.window.rec_start.isEnabled())

    def test_spawn_and_idle_clear_ready_state(self):
        window = self.window
        window._quest_ready = True
        window._redo_armed_until = time.monotonic() + 5
        window._idle("已停止")
        self.assertFalse(window._quest_ready)
        self.assertEqual(window._redo_armed_until, 0.0)
        self.assertEqual(window.status.text(), "已停止")
        window._quest_ready = True
        window._redo_armed_until = time.monotonic() + 5
        with tempfile.TemporaryDirectory() as directory:
            window.log_path = Path(directory) / "app.log"
            with patch("openarm_app.main_window.subprocess.Popen", return_value=Mock()):
                window._spawn("sim", "can1", "can0")
            self.assertFalse(window._quest_ready)
            self.assertEqual(window._redo_armed_until, 0.0)
            window._close_log()


if __name__ == "__main__":
    unittest.main()
