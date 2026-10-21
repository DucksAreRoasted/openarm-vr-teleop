"""Offscreen smoke test for the QML operator console."""

import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QSG_RHI_BACKEND", "software")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "script"))

try:
    from PySide6.QtWidgets import QApplication
    from openarm_app.main_window import ConsoleController, create_engine
    QT_AVAILABLE = True
except ImportError:
    QT_AVAILABLE = False


@unittest.skipUnless(QT_AVAILABLE, "PySide6 required")
class ConsoleQmlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_shell_pages_mode_lock_and_collapsed_log(self):
        from PySide6.QtCore import QSettings
        with tempfile.TemporaryDirectory() as directory:
            settings = QSettings(str(Path(directory) / "settings.ini"), QSettings.Format.IniFormat)
            self._run_shell(settings)

    def test_friction_preview_then_confirmed_write(self):
        from PySide6.QtCore import QSettings
        from PySide6.QtWidgets import QMessageBox
        with tempfile.TemporaryDirectory() as directory:
            settings = QSettings(str(Path(directory) / "settings.ini"), QSettings.Format.IniFormat)
            with patch("openarm_app.main_window.local_ip", return_value="127.0.0.1"), \
                 patch("openarm_app.main_window.list_usb_cameras", return_value=[]), \
                 patch("openarm_app.main_window.QSettings", return_value=settings):
                console = ConsoleController()
            try:
                preview = ("FRICTION_LOG_PATH=/tmp/friction_x.csv\n"
                           "[left] J4: Fc 0.400 → 0.330  Fv 0.0301 → 0.0250\n"
                           "[right] J4: 保留 Fc 0.400 Fv 0.0301  [warn] 过大")
                with patch("openarm_app.main_window.QMessageBox.question",
                           return_value=QMessageBox.StandardButton.Yes) as question, \
                     patch("openarm_app.main_window.run_friction_write",
                           return_value=(True, "已写入")) as write, \
                     patch("openarm_app.main_window.QMessageBox.information"):
                    console._friction_done(False, True, preview)
                    self.assertIn("J4", question.call_args.args[2])
                    for _ in range(50):
                        self.app.processEvents()
                        if write.called and not console._configuration_busy:
                            break
                        time.sleep(0.02)
                    self.assertEqual(write.call_args.args[0], Path("/tmp/friction_x.csv"))
                with patch("openarm_app.main_window.QMessageBox.warning") as warning, \
                     patch("openarm_app.main_window.run_friction_write") as write:
                    console._friction_done(False, True, "FRICTION_LOG_PATH=/tmp/f.csv\n[warn] 没有关节通过检查")
                    warning.assert_called_once()
                    write.assert_not_called()
            finally:
                console.close()

    def _item_texts(self, item):
        import shiboken6
        seen = set()
        found = []

        def identity(obj):
            pointer = shiboken6.getCppPointer(obj)
            return pointer[0] if pointer else id(obj)

        def walk(obj):
            if obj is None:
                return
            key = identity(obj)
            if key in seen:
                return
            seen.add(key)
            value = obj.property("text") if hasattr(obj, "property") else None
            if isinstance(value, str) and value:
                found.append(value)
            for child in obj.children():
                walk(child)

        walk(item)
        return found

    def _run_shell(self, settings):
        with patch("openarm_app.main_window.local_ip", return_value="127.0.0.1"), \
             patch("openarm_app.main_window.list_usb_cameras", return_value=[]), \
             patch("openarm_app.main_window.QSettings", return_value=settings):
            console = ConsoleController()
        engine, console = create_engine(console)
        self.app.processEvents()
        try:
            self.assertTrue(engine.rootObjects(), "QML console failed to load")
            self.assertFalse(console.logExpanded)
            console.setPage("capture")
            self.assertEqual(console.page, "capture")
            console.setPage("gravity")
            self.assertEqual(console.page, "gravity")
            console.setPage("data")
            self.assertEqual(console.page, "data")
            console.setPage("replay")
            console.setPage("motors")
            console.setPage("teach")
            self.assertEqual(console.page, "teach")
            console.setPage("shape")
            self.assertEqual(console.page, "teach")
            self.app.processEvents()
            root = engine.rootObjects()[0]
            from PySide6.QtCore import QObject
            replay = root.findChild(QObject, "replayPage")
            self.assertIsNotNone(replay)
            self.assertIsNone(root.findChild(QObject, "trajectoryPage"))
            replay_text = "\n".join(self._item_texts(replay))
            window_text = "\n".join(self._item_texts(root))
            self.assertNotIn("肩前固定轨迹", replay_text)
            self.assertNotIn("利萨如", window_text)
            self.assertIn("实时关节角度", window_text)
            for label in ("拖动示教", "VR录制", "遥操数据管理", "遥操数据回放", "电机参数"):
                self.assertIn(label, window_text)
            console.setPage("teleop")
            console.setMode("real")
            self.assertEqual(console.mode, "real")
            console.proc = Mock()
            console.proc.poll.return_value = None
            console._apply_record_state()
            self.assertTrue(console.running)
            console.setMode("sim")
            self.assertEqual(console.mode, "real")
            self.assertFalse(console.canStart)
            self.assertTrue(console.canStop)
        finally:
            console.proc = None
            console.close()
            for root in engine.rootObjects():
                root.close()
