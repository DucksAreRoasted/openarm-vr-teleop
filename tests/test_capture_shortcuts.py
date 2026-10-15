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
    from PySide6.QtWidgets import QApplication
    from openarm_app.main_window import MainWindow
except ImportError:
    MainWindow = None


class GestureTests(unittest.TestCase):
    def test_short_cancel_and_long_while_held(self):
        for hand in ("left", "right"):
            gestures = CaptureShortcuts()
            gestures.update(hand, {"stickPressed": False}, 0)
            self.assertIsNone(gestures.update(hand, {"stickPressed": True}, .1))
            self.assertEqual(
                gestures.update(hand, {"stickPressed": False}, .3),
                f"{hand}-short",
            )
            self.assertIsNone(gestures.update(hand, {"stickPressed": False}, .31))

            gestures = CaptureShortcuts()
            gestures.update(hand, {"stickPressed": False}, 0)
            gestures.update(hand, {"stickPressed": True}, .1)
            instant = .1
            while instant < 1.2:
                instant = min(1.2, instant + .1)
                self.assertIsNone(gestures.update(hand, {"stickPressed": True}, instant))
            self.assertIsNone(gestures.update(hand, {"stickPressed": False}, instant + .01))

            gestures = CaptureShortcuts()
            gestures.update(hand, {"stickPressed": False}, 0)
            gestures.update(hand, {"stickPressed": True}, .1)
            instant = .1
            fired = None
            while fired is None and instant < 2.0:
                instant = min(2.0, instant + .1)
                fired = gestures.update(hand, {"stickPressed": True}, instant)
            self.assertEqual(fired, f"{hand}-long")
            self.assertGreaterEqual(instant - .1, 1.5)
            self.assertIsNone(gestures.update(hand, {"stickPressed": True}, instant + .1))
            self.assertIsNone(gestures.update(hand, {"stickPressed": False}, instant + .2))

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
        with patch.dict(os.environ, {"OPENARM_QUEST_CAPTURE_PORT": "0", "OPENARM_QUEST_UDP_HOST": "127.0.0.1"}), \
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
            self.assertNotIn("回收站", while_recording["message"])
            self.assertFalse(self.request("right-long")["ok"])
            self.assertTrue(self.request("left-short")["ok"])
            self.assertFalse(self.request("left-short")["ok"])
            self.assertTrue(self.request("right-short")["ok"])
            discarded = self.request("left-long")
            self.assertTrue(discarded["ok"])
            self.assertIn("回收站", discarded["message"])
            self.assertEqual(window.rec_redo.text(), "撤销丢弃")
            restored = self.request("left-long")
            self.assertTrue(restored["ok"])
            self.assertIn("已恢复", restored["message"])
            self.assertEqual(window.rec_redo.text(), "丢弃重录")
            self.assertTrue(self.request("left-long")["ok"])
            self.assertTrue(self.request("left-short")["ok"])
            self.assertTrue(self.request("left-short")["ok"])
            self.assertTrue(self.request("right-long")["ok"])
            self.assertEqual([call.args[0] for call in send.call_args_list], [
                "start,shortcut_test", "stop", "success", "redo", "undo", "redo",
                "start,shortcut_test", "stop", "fail"])
            question.assert_not_called()
        window.proc = None
        self.assertFalse(self.request("left-short")["ok"])
        self.assertEqual(
            self.request("left-short")["message"],
            "当前不能执行此采集操作，请检查录制状态。",
        )

    def test_controller_redo_discards_immediately_after_stop_success_and_fail(self):
        window = self.window
        self.run_teleop()
        window.batch_edit.setText("redo_batch")
        with patch("openarm_app.main_window.send_record") as send, \
             patch("openarm_app.main_window.QMessageBox.question") as question:
            self.assertTrue(self.request("left-short")["ok"])
            self.assertTrue(self.request("left-short")["ok"])
            self.assertTrue(window.rec_redo.isEnabled())
            discarded = self.request("left-long")
            self.assertTrue(discarded["ok"])
            self.assertIn("回收站", discarded["message"])
            self.assertFalse(window._can_redo)
            self.assertEqual(window.rec_redo.text(), "撤销丢弃")
            window._undo_until = time.monotonic() - 0.01
            expired = self.request("left-long")
            self.assertFalse(expired["ok"])
            self.assertEqual([call.args[0] for call in send.call_args_list], [
                "start,redo_batch", "stop", "redo"])

            self.assertTrue(self.request("left-short")["ok"])
            self.assertTrue(self.request("left-short")["ok"])
            self.assertTrue(self.request("right-short")["ok"])
            self.assertTrue(window.rec_redo.isEnabled())
            self.assertTrue(self.request("left-long")["ok"])

            self.assertTrue(self.request("left-short")["ok"])
            self.assertTrue(self.request("left-short")["ok"])
            self.assertTrue(self.request("right-long")["ok"])
            self.assertTrue(window.rec_redo.isEnabled())
            self.assertTrue(self.request("left-long")["ok"])
            question.assert_not_called()
            self.assertEqual([call.args[0] for call in send.call_args_list], [
                "start,redo_batch", "stop", "redo",
                "start,redo_batch", "stop", "success", "redo",
                "start,redo_batch", "stop", "fail", "redo",
            ])

    def test_other_gesture_keeps_undo_and_second_long_restores(self):
        window = self.window
        self.run_teleop()
        window.batch_edit.setText("redo_batch")
        with patch("openarm_app.main_window.send_record") as send:
            self.assertTrue(self.request("left-short")["ok"])
            self.assertTrue(self.request("left-short")["ok"])
            self.assertTrue(self.request("left-long")["ok"])
            self.assertFalse(self.request("right-short")["ok"])
            restored = self.request("left-long")
            self.assertTrue(restored["ok"])
            self.assertEqual([call.args[0] for call in send.call_args_list], [
                "start,redo_batch", "stop", "redo", "undo"])
            self.assertTrue(window._can_redo)
            self.assertTrue(window._awaiting_label)

    def test_redo_button_discards_without_dialog_and_can_undo(self):
        window = self.window
        self.run_teleop()
        window.batch_edit.setText("button_batch")
        with patch("openarm_app.main_window.send_record") as send, \
             patch("openarm_app.main_window.QMessageBox.question") as question:
            self.assertTrue(window._record("start"))
            self.assertTrue(window._record("stop"))
            window.rec_redo.click()
            question.assert_not_called()
            self.assertFalse(window._can_redo)
            self.assertEqual(window.rec_redo.text(), "撤销丢弃")
            self.assertEqual(send.call_args_list[-1].args[0], "redo")
            window.rec_redo.click()
            self.assertTrue(window._can_redo)
            self.assertTrue(window._awaiting_label)
            self.assertEqual(send.call_args_list[-1].args[0], "undo")
            self.assertEqual(window.record_hint.text(), "已恢复这一条。可以标注、重录，或开始下一条。")

            self.assertTrue(window._record("success"))
            self.assertTrue(window.rec_redo.isEnabled())
            self.assertTrue(window._record("redo"))
            self.assertFalse(window._can_redo)
            self.assertFalse(window._awaiting_label)
            self.assertIn("回收站", window.record_hint.text())

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

            window.log_path.write_text("ready\n[quest-cartesian] bimanual running.\n", encoding="utf-8")
            window._refresh_log()
            self.assertTrue(window._quest_ready)
            self.assertEqual(window.status.text(), "仿真运行中")
            self.assertTrue(window.rec_start.isEnabled())
            self.assertIn("可以开始录", window.record_hint.text())

            window.radio_real.setChecked(True)
            window._quest_ready = False
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
        window._undo_until = time.monotonic() + 5
        window._idle("已停止")
        self.assertFalse(window._quest_ready)
        self.assertEqual(window._undo_until, 0.0)
        self.assertEqual(window.status.text(), "已停止")
        window._quest_ready = True
        window._undo_until = time.monotonic() + 5
        with tempfile.TemporaryDirectory() as directory:
            window.log_path = Path(directory) / "app.log"
            with patch("openarm_app.main_window.subprocess.Popen", return_value=Mock()):
                window._spawn("sim", "can1", "can0")
            self.assertFalse(window._quest_ready)
            self.assertEqual(window._undo_until, 0.0)
            window._close_log()


if __name__ == "__main__":
    unittest.main()
