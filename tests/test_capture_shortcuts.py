"""Capture gestures and the console's state-gated UDP shortcut interface."""
import json
import os
from pathlib import Path
import socket
import sys
import time
import unittest
from unittest.mock import patch

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
    def test_record_label_redo_and_reject_invalid_states(self):
        app = QApplication.instance() or QApplication([])
        with patch.dict(os.environ, {"OPENARM_QUEST_CAPTURE_PORT": "0"}), \
             patch("openarm_app.main_window.local_ip", return_value="127.0.0.1"), \
             patch("openarm_app.main_window.list_usb_cameras", return_value=[]):
            window = MainWindow()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)

        def request(action):
            sock.sendto(json.dumps({"action": action}).encode(), ("127.0.0.1", window._capture_socket.localPort()))
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline:
                app.processEvents()
                try:
                    return json.loads(sock.recv(4096))
                except BlockingIOError:
                    time.sleep(.001)
            self.fail("no console reply")

        try:
            with patch.object(window, "_running", return_value=True), \
                 patch("openarm_app.main_window.send_record") as send:
                window._apply_record_state()
                window.batch_edit.setText("shortcut_test")
                self.assertFalse(request("right-short")["ok"])
                self.assertTrue(request("left-short")["ok"])
                self.assertTrue(window._recording)
                window._apply_record_state()
                self.assertFalse(window.rec_start.isEnabled())
                self.assertTrue(window.rec_stop.isEnabled())
                self.assertFalse(request("left-long")["ok"])
                self.assertFalse(request("right-long")["ok"])
                self.assertTrue(request("left-short")["ok"])
                self.assertFalse(request("left-short")["ok"])
                self.assertTrue(request("right-short")["ok"])
                self.assertTrue(request("left-long")["ok"])
                self.assertFalse(request("left-long")["ok"])
                self.assertTrue(request("left-short")["ok"])
                self.assertTrue(request("left-short")["ok"])
                self.assertTrue(request("right-long")["ok"])
                self.assertEqual([call.args[0] for call in send.call_args_list], [
                    "start,shortcut_test", "stop", "success", "redo", "start,shortcut_test", "stop", "fail"])
            self.assertFalse(request("left-short")["ok"])
        finally:
            sock.close()
            window._capture_socket.close()
            window.close()


if __name__ == "__main__":
    unittest.main()
