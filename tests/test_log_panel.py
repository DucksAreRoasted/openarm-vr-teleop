"""Live-log behavior checks using disposable files and offscreen Qt widgets."""
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "script"))
try:
    from PySide6.QtCore import QSettings
    from PySide6.QtWidgets import QApplication
    from openarm_app.log_panel import IncrementalLogReader, LogPanel, classify
    from openarm_app.main_window import MainWindow
    QT_AVAILABLE = True
except ImportError:
    QT_AVAILABLE = False


@unittest.skipUnless(QT_AVAILABLE, "PySide6 required")
class LogReaderTests(unittest.TestCase):
    def test_incremental_utf8_truncation_and_replacement(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "input.log"
            reader = IncrementalLogReader()
            encoded = "机械臂\n".encode()
            path.write_bytes(encoded[:2])
            self.assertEqual(reader.read(path), "")
            with path.open("ab") as handle:
                handle.write(encoded[2:])
            self.assertEqual(reader.read(path), "机械臂\n")
            self.assertEqual(reader.read(path), "")
            path.write_bytes(b"x")
            self.assertEqual(reader.read(path), "x")
            replacement = path.with_suffix(".new")
            replacement.write_text("rotated\n")
            replacement.replace(path)
            self.assertEqual(reader.read(path), "rotated\n")

    def test_clock_skew_warning_is_not_an_error(self):
        line = "gmake[3]: 警告：检测到时钟错误。您的构建版本可能是不完整的。"
        self.assertEqual(classify(line), "WARN")
        self.assertEqual(classify("[ERROR] failed"), "ERROR")
        self.assertEqual(classify("启动失败"), "ERROR")
        self.assertEqual(
            classify("[replay] PROGRESS time=1 duration=12 phase=playing error=0.01"),
            "INFO",
        )
        self.assertEqual(classify("[replay][ERROR] alignment timed out"), "ERROR")


@unittest.skipUnless(QT_AVAILABLE, "PySide6 required")
class LogPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name)
        self.settings = QSettings(str(self.path / "settings.ini"), QSettings.Format.IniFormat)
        self.panel = LogPanel(settings=self.settings, archive_path=self.path / "complete.log")
        self.panel.set_expanded(True)
        self.panel.resize(1100, 300)
        self.panel.show()
        self.app.processEvents()

    def tearDown(self):
        if self.panel._dialog:
            self.panel._dialog.close()
        self.panel.close()
        if self.panel._archive:
            self.panel._archive.close()
        self.directory.cleanup()

    def test_sources_partial_lines_colors_and_filtering(self):
        self.panel.enqueue("[WARN] 控制", "控制器")
        self.panel.enqueue("[ERROR] 标定失败\n", "标定")
        self.panel.enqueue("器告警\n[DEBUG] diagnostics\n", "控制器")
        self.panel.enqueue("[quest-ui] recording saved\n", "控制器")
        self.panel.flush()
        text = self.panel.view.toPlainText()
        self.assertIn("控制器告警", text)
        self.assertIn("标定失败", text)
        self.assertNotIn("diagnostics", text)
        self.assertEqual((self.panel.warnings, self.panel.errors), (1, 1))
        self.assertEqual(self.panel.entries[-1].source, "录制")
        first = self.panel.view.document().begin().begin().fragment()
        self.assertEqual(first.charFormat().foreground().color().name(), "#ff6b6b")
        self.panel.source.setCurrentText("控制器")
        self.panel.filter.setText("告警")
        self.panel.rebuild()
        self.assertIn("控制器告警", self.panel.view.toPlainText())
        self.assertNotIn("标定失败", self.panel.view.toPlainText())
        self.panel.regex.setChecked(True)
        self.panel.filter.setText("[")
        self.panel.rebuild()
        self.assertFalse(self.panel.filter_error.isHidden())
        self.assertEqual(self.panel.view.toPlainText(), "")

    def test_scroll_up_does_not_follow_and_latest_resumes(self):
        self.panel.enqueue("".join(f"line {i}\n" for i in range(100)), "控制器")
        self.panel.flush()
        self.app.processEvents()
        bar = self.panel.view.verticalScrollBar()
        bar.setValue(10)
        before = bar.value()
        self.panel.enqueue("new line\n", "控制器")
        self.panel.flush()
        self.assertEqual(bar.value(), before)
        self.assertEqual(self.panel.unread, 1)
        self.panel.follow_latest()
        self.assertEqual(bar.value(), bar.maximum())
        self.assertEqual(self.panel.unread, 0)

    def test_pause_archives_then_resumes_and_clear_keeps_file(self):
        self.panel.pause.setChecked(True)
        self.panel.enqueue("[ERROR] still collected\n", "标定")
        self.panel.flush()
        self.assertEqual(self.panel.view.toPlainText(), "")
        self.assertIn("still collected", (self.path / "complete.log").read_text())
        self.assertEqual(self.panel.errors, 1)
        self.panel.pause.setChecked(False)
        self.assertIn("still collected", self.panel.view.toPlainText())
        self.panel.clear()
        self.assertEqual(self.panel.errors, 0)
        self.assertEqual(self.panel.view.toPlainText(), "")
        self.assertIn("still collected", (self.path / "complete.log").read_text())

    def test_high_rate_is_bounded_but_archive_is_complete(self):
        self.panel.enqueue("".join(f"sequence {i}\n" for i in range(6500)))
        self.assertLessEqual(len(self.panel.pending), 5000)
        self.panel.flush()
        text = self.panel.view.toPlainText()
        self.assertIn("省略 6000 行", text)
        self.assertIn("sequence 6499", text)
        self.assertLessEqual(self.panel.view.blockCount(), 5000)
        self.assertEqual(len((self.path / "complete.log").read_text().splitlines()), 6500)

    def test_detach_close_reattaches_same_live_view_and_remembers_size(self):
        view = self.panel.view
        self.panel.toggle_detached()
        self.app.processEvents()
        dialog = self.panel._dialog
        dialog.resize(1000, 650)
        self.panel.enqueue("detached output\n")
        self.panel.flush()
        self.assertIn("detached output", view.toPlainText())
        dialog.close()
        self.app.processEvents()
        self.assertIsNone(self.panel._dialog)
        self.assertIs(self.panel.view, view)
        self.assertIs(self.panel.body.parentWidget(), self.panel)
        self.assertIsNotNone(self.settings.value("log/dialogGeometry"))
        self.panel.set_expanded(False)
        self.panel.save_settings()
        self.assertFalse(self.settings.value("log/expanded", type=bool))

    def test_search_highlights_and_errors_are_navigable(self):
        self.panel.enqueue("[INFO] good\n[ERROR] broken\n")
        self.panel.flush()
        self.panel.open_find()
        self.panel.find.setText("broken")
        self.assertEqual(len(self.panel.view.extraSelections()), 1)
        self.panel.next_error()
        self.assertIn("ERROR", self.panel.view.textCursor().block().text())

    def test_fatal_styling_and_status_survive_duplicate_causes(self):
        events = []
        self.panel.fatal_received.connect(events.append)
        self.panel.enqueue("[FATAL] stopped\n[FATAL] stopped\n[INFO] diagnostics\n")
        self.panel.flush()
        self.assertEqual(len(events), 2)
        self.assertEqual(len(self.panel._seen_fatal), 1)
        self.assertEqual(self.panel.errors, 2)
        self.assertFalse(self.panel.fatal_banner.isHidden())
        first = self.panel.view.document().begin().begin().fragment().charFormat()
        self.assertEqual(first.background().color().name(), "#c62828")
        self.assertEqual(first.fontWeight(), 700)
        last = self.panel.view.document().lastBlock().begin().fragment().charFormat()
        self.assertEqual(last.fontWeight(), 400)

    def test_controller_increment_does_not_overwrite_calibration(self):
        with patch("openarm_app.main_window.local_ip", return_value="127.0.0.1"), \
             patch("openarm_app.main_window.list_usb_cameras", return_value=[]), \
             patch("openarm_app.main_window.QSettings", return_value=self.settings):
            window = MainWindow()
        try:
            window.log_path = self.path / "controller.log"
            window.log_path.write_text("first\n")
            window._refresh_log()
            window._calib_log_line("[calib] hold\n")
            with window.log_path.open("a") as handle:
                handle.write("last without newline")
            window._refresh_log()
            window._refresh_log()
            window._idle("stopped")
            text = window.log.toPlainText()
            self.assertEqual(text.count("first"), 1)
            self.assertEqual(text.count("[calib] hold"), 1)
            self.assertEqual(text.count("last without newline"), 1)
        finally:
            window.close()

    def test_panel_height_and_collapsed_state_survive_reopening(self):
        def create_window():
            with patch("openarm_app.main_window.local_ip", return_value="127.0.0.1"), \
                 patch("openarm_app.main_window.list_usb_cameras", return_value=[]), \
                 patch("openarm_app.main_window.QSettings", return_value=self.settings):
                return MainWindow()

        window = create_window()
        self.assertFalse(window.logExpanded)
        window.log_panel.enqueue("[FATAL] boom\n")
        self.assertTrue(window.logExpanded)
        self.assertIn("严重错误", window.status.text())
        window.setLogExpanded(False)
        window.setLogExpanded(True)
        window.setLogHeight(300)
        window.close()
        window = create_window()
        self.assertTrue(window.logExpanded)
        self.assertAlmostEqual(window.logHeight, 300, delta=5)
        window.log_panel.enqueue("[FATAL] boom\n")
        self.assertTrue(window.logExpanded)
        self.assertIn("严重错误", window.status.text())
        window.setLogExpanded(False)
        window.close()
        window = create_window()
        self.assertFalse(window.logExpanded)
        self.assertFalse(window.log_panel.toggle.isChecked())
        window.close()
