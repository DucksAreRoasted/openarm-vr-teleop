"""Bounded live console with incremental file input and a detachable viewer."""
from __future__ import annotations

import codecs
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import os
import re

from PySide6.QtCore import QAbstractListModel, QModelIndex, QSettings, Qt, QTimer, Signal
from PySide6.QtGui import QAction, QColor, QKeySequence, QShortcut, QTextCharFormat, QTextCursor
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDialog, QHBoxLayout, QLabel, QLineEdit,
    QMenu, QPlainTextEdit, QPushButton, QTextEdit, QToolButton, QVBoxLayout, QWidget,
)

ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
COLORS = {"DEBUG": "#7b8794", "INFO": "#d9e2ec", "WARN": "#f7c948",
          "ERROR": "#ff6b6b", "FATAL": "#ffffff"}


class IncrementalLogReader:
    """Read new bytes only; tolerate UTF-8 splits, truncation and file replacement."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.generation = getattr(self, "generation", 0) + 1
        self.offset = 0
        self.identity = None
        self.decoder = codecs.getincrementaldecoder("utf-8")("replace")

    def read(self, path: Path, *, final: bool = False) -> str:
        try:
            with path.open("rb") as handle:
                stat = os.fstat(handle.fileno())
                identity = (stat.st_dev, stat.st_ino)
                if identity != self.identity or stat.st_size < self.offset:
                    self.reset()
                self.identity = identity
                handle.seek(self.offset)
                data = handle.read(1024 * 1024)
                self.offset = handle.tell()
                return self.decoder.decode(data, final=final and self.offset >= stat.st_size)
        except FileNotFoundError:
            return ""


@dataclass(frozen=True)
class LogEntry:
    time: datetime
    level: str
    source: str
    text: str

    def display(self) -> str:
        stamp = self.time.strftime("%H:%M:%S.%f")[:-3]
        return f"{stamp}  {self.level:<5}  {self.source}  {self.text}"


def classify(text: str) -> str:
<<<<<<< HEAD
    # Only a level marker. The startup banner says "fatal lock" and is not a fault.
    if re.search(r"\[FATAL\]|\bFATAL:", text, re.IGNORECASE):
=======
    if re.search(r"\bFATAL\b", text, re.IGNORECASE):
>>>>>>> origin/main
        level = "FATAL"
    else:
        warning = re.search(r"\bWARN(?:ING)?\b|警告", text, re.IGNORECASE)
        # Progress lines carry a tracking field `error=0.01`. That is not a fault.
        if re.search(r"\bERROR\b(?!\s*=)|失败", text, re.IGNORECASE) or ("错误" in text and not warning):
            level = "ERROR"
        elif warning:
            level = "WARN"
        elif re.search(r"\bDEBUG\b", text, re.IGNORECASE):
            level = "DEBUG"
        else:
            level = "INFO"
    return level


class LogLineModel(QAbstractListModel):
    """Visible log rows for the QML console. The widget view stays for existing checks."""

    DisplayRole = Qt.ItemDataRole.UserRole + 1
    LevelRole = Qt.ItemDataRole.UserRole + 2

    def __init__(self, parent=None):
        super().__init__(parent)
        self._rows: list[LogEntry] = []

    def roleNames(self):  # noqa: N802
        return {
            int(Qt.ItemDataRole.DisplayRole): b"displayText",
            self.DisplayRole: b"display",
            self.LevelRole: b"level",
        }

    def rowCount(self, parent=QModelIndex()):  # noqa: N802
        if parent.isValid():
            return 0
        return len(self._rows)

    def data(self, index, role=Qt.ItemDataRole.DisplayRole):  # noqa: N802
        if not index.isValid() or not (0 <= index.row() < len(self._rows)):
            return None
        entry = self._rows[index.row()]
        if role in (self.DisplayRole, Qt.ItemDataRole.DisplayRole):
            return entry.display()
        if role == self.LevelRole:
            return entry.level
        return None

    def replace(self, entries) -> None:
        self.beginResetModel()
        self._rows = list(entries)
        self.endResetModel()

    def append(self, entries) -> None:
        if not entries:
            return
        start = len(self._rows)
        self.beginInsertRows(QModelIndex(), start, start + len(entries) - 1)
        self._rows.extend(entries)
        self.endInsertRows()
        overflow = len(self._rows) - LogPanel.MAX_LINES
        if overflow > 0:
            self.beginRemoveRows(QModelIndex(), 0, overflow - 1)
            del self._rows[:overflow]
            self.endRemoveRows()


class LogDialog(QDialog):
    closing = Signal()

    def closeEvent(self, event):  # noqa: N802
        self.closing.emit()
        event.accept()


class LogPanel(QWidget):
    layout_changed = Signal()
    fatal_received = Signal(str)
    countsChanged = Signal()
    MAX_LINES = 5000

    def __init__(self, parent=None, *, settings=None, archive_path: Path | None = None):
        super().__init__(parent)
        self.settings = settings if settings is not None else QSettings("OpenArm", "TeleopConsole")
        self.entries = deque(maxlen=self.MAX_LINES)
        self.pending = deque(maxlen=self.MAX_LINES)
        self.partials: dict[str, str] = {}
        self.warnings = self.errors = self.unread = self.dropped = 0
        self._rendering = False
        self._follow = True
        self._seen_fatal = set()
        self._dialog = None
        self._archive = archive_path.open("a", encoding="utf-8") if archive_path else None
        self.archive_path = archive_path
        if self._archive:
            archive = self._archive
            self.destroyed.connect(lambda *_: archive.close())
        self.lines = LogLineModel(self)
        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self._layout.setSpacing(4)
        self.header = QWidget()
        self.header.setStyleSheet("QPushButton, QToolButton { padding: 2px 8px; min-height: 20px; }")
        header = QHBoxLayout(self.header)
        header.setContentsMargins(6, 0, 6, 0)
        self.toggle = QToolButton()
        self.toggle.setText("▾ 实时日志")
        self.toggle.setCheckable(True)
        self.toggle.setChecked(True)
        self.toggle.clicked.connect(lambda checked: self.set_expanded(checked))
        header.addWidget(self.toggle)
        self.counts = QPushButton("⚠ W 0   ✖ E 0")
        self.counts.setToolTip("跳转到下一条可见错误；必要时取消过滤")
        self.counts.clicked.connect(self.next_error)
        header.addWidget(self.counts)
        self.archive_label = QLabel("完整日志已保存" if archive_path else "最多保留 5000 行")
        self.archive_label.setObjectName("Hint")
        self.archive_label.setToolTip(str(archive_path or ""))
        header.addWidget(self.archive_label, 1)
        self.detach_button = QPushButton("独立窗口 ↗")
        self.detach_button.clicked.connect(self.toggle_detached)
        header.addWidget(self.detach_button)
        self._layout.addWidget(self.header)
        self.body = QWidget()
        body_layout = QVBoxLayout(self.body)
        body_layout.setContentsMargins(0, 0, 0, 0)
        body_layout.setSpacing(4)
        tools = QHBoxLayout()
        self.level_button = QToolButton()
        self.level_button.setText("级别 ▾")
        self.level_button.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup)
        menu = QMenu(self.level_button)
        self.levels = {}
        for level in COLORS:
            action = QAction(level, menu)
            action.setCheckable(True)
            action.setChecked(level != "DEBUG")
            action.toggled.connect(self.schedule_filter)
            menu.addAction(action)
            self.levels[level] = action
        self.level_button.setMenu(menu)
        tools.addWidget(self.level_button)
        self.source = QComboBox()
        self.source.addItems(["全部来源", "控制器", "标定", "录制", "控制台"])
        self.source.currentTextChanged.connect(self.schedule_filter)
        tools.addWidget(self.source)
        self.filter = QLineEdit()
        self.filter.setPlaceholderText("过滤日志内容…")
        self.filter.setClearButtonEnabled(True)
        self.filter.textChanged.connect(self.schedule_filter)
        tools.addWidget(self.filter, 1)
        self.regex = QCheckBox("正则")
        self.regex.toggled.connect(self.schedule_filter)
        tools.addWidget(self.regex)
        self.pause = QPushButton("暂停显示")
        self.pause.setCheckable(True)
        self.pause.toggled.connect(self._pause_changed)
        self.pause.setToolTip("只暂停显示，日志仍持续采集并写入文件")
        tools.addWidget(self.pause)
        self.new_button = QPushButton("↓ 最新")
        self.new_button.clicked.connect(self.follow_latest)
        tools.addWidget(self.new_button)
        clear = QPushButton("清空")
        clear.setToolTip("清空面板，不删除日志文件")
        clear.clicked.connect(self.clear)
        tools.addWidget(clear)
        copy = QPushButton("复制全部")
        copy.setToolTip("复制当前缓冲中的全部日志，包括被过滤的内容")
        copy.clicked.connect(lambda: QApplication.clipboard().setText(
            "\n".join(entry.display() for entry in [*self.entries, *self.pending])))
        tools.addWidget(copy)
        body_layout.addLayout(tools)
        self.filter_error = QLabel()
        self.filter_error.setStyleSheet("color: #f7c948;")
        self.filter_error.hide()
        body_layout.addWidget(self.filter_error)
        self.fatal_banner = QLabel()
        self.fatal_banner.setWordWrap(True)
        self.fatal_banner.setStyleSheet("color: white; background: #c62828; padding: 6px;")
        self.fatal_banner.hide()
        body_layout.addWidget(self.fatal_banner)
        self.find_row = QWidget()
        find_layout = QHBoxLayout(self.find_row)
        find_layout.setContentsMargins(0, 0, 0, 0)
        find_layout.addWidget(QLabel("查找"))
        self.find = QLineEdit()
        self.find.setPlaceholderText("查找并高亮（Enter 下一处）")
        self.find.textChanged.connect(self.highlight_matches)
        self.find.returnPressed.connect(self.find_next)
        find_layout.addWidget(self.find, 1)
        close_find = QPushButton("关闭查找")
        close_find.clicked.connect(self.find_row.hide)
        find_layout.addWidget(close_find)
        self.find_row.hide()
        body_layout.addWidget(self.find_row)
        self.view = QPlainTextEdit()
        self.view.setReadOnly(True)
        self.view.setMaximumBlockCount(self.MAX_LINES)
        self.view.setUndoRedoEnabled(False)
        self.view.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self.view.verticalScrollBar().valueChanged.connect(self._scroll_changed)
        body_layout.addWidget(self.view, 1)
        self._layout.addWidget(self.body, 1)
        self._filter_timer = QTimer(self)
        self._filter_timer.setSingleShot(True)
        self._filter_timer.setInterval(150)
        self._filter_timer.timeout.connect(self.rebuild)
        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.flush)
        self.timer.start()
        self.shortcut = QShortcut(QKeySequence("Ctrl+F"), self)
        self.shortcut.activated.connect(self.open_find)
        self.set_expanded(self.settings.value("log/expanded", False, type=bool))

    def enqueue(self, text: str, source: str = "控制台", *, final=False):
        text = self.partials.pop(source, "") + ANSI.sub("", text).replace("\r\n", "\n").replace("\r", "\n")
        lines = text.split("\n")
        tail = lines.pop()
        if final and tail:
            lines.append(tail)
        elif tail:
            self.partials[source] = tail
        now = datetime.now().astimezone()
        archive = []
        for line in lines:
            line_source = source
            if source == "控制器" and re.search(r"\[(?:record(?:er|ing)?|quest-ui|dataset)[\]\-]", line, re.IGNORECASE):
                line_source = "录制"
            entry = LogEntry(now, classify(line), line_source, line)
            if len(self.pending) == self.MAX_LINES:
                self.dropped += 1
            self.pending.append(entry)
            self.warnings += entry.level == "WARN"
            self.errors += entry.level in ("ERROR", "FATAL")
            if entry.level == "FATAL":
                self.fatal_received.emit(line)
            if entry.level == "FATAL" and line not in self._seen_fatal:
                self._seen_fatal.add(line)
                self.fatal_banner.setText("严重错误：" + line[:500])
                self.fatal_banner.show()
            archive.append(f"{now.isoformat(timespec='milliseconds')} [{entry.level}] [{line_source}] {line}\n")
        if self._archive and archive:
            self._archive.write("".join(archive))
            self._archive.flush()

    def flush(self):
        self.counts.setText(f"⚠ W {self.warnings}   ✖ E {self.errors}")
        self.toggle.setStyleSheet("color: #ff6b6b;" if self.errors and not self.toggle.isChecked() else "")
        if self.pause.isChecked():
            self.new_button.setText(f"↓ 新日志 {len(self.pending) + self.dropped} 条")
            self.countsChanged.emit()
            return
        if not self.pending:
            return
        omitted = self.dropped + max(0, len(self.pending) - 500)
        self.dropped = 0
        batch = list(self.pending)[-500:]
        self.pending.clear()
        if omitted:
            batch.insert(0, LogEntry(datetime.now().astimezone(), "WARN", "控制台",
                                    f"…省略 {omitted} 行高频日志，完整内容见日志文件"))
        self.entries.extend(batch)
        visible = [entry for entry in batch if self.matches(entry)]
        self._insert(visible)
        if not self._follow:
            self.unread += len(visible)
        self.new_button.setText(f"↓ 新日志 {self.unread} 条" if self.unread else "↓ 最新")
        self.countsChanged.emit()

    def matches(self, entry):
        if not self.levels[entry.level].isChecked():
            return False
        if self.source.currentIndex() and entry.source != self.source.currentText():
            return False
        query = self.filter.text()
        if self.regex.isChecked():
            try:
                return re.search(query, entry.text, re.IGNORECASE) is not None
            except re.error:
                return False
        return query.casefold() in entry.text.casefold()

    def _insert(self, entries, *, update_model=True):
        if not entries:
            return
        if update_model:
            self.lines.append(entries)
        bar = self.view.verticalScrollBar()
        old_value = bar.value()
        follow = self._follow and bar.value() >= bar.maximum() - 4
        self._rendering = True
        cursor = QTextCursor(self.view.document())
        cursor.movePosition(QTextCursor.MoveOperation.End)
        cursor.beginEditBlock()
        for entry in entries:
            if not self.view.document().isEmpty():
                cursor.insertBlock()
            fmt = QTextCharFormat()
            fmt.setForeground(QColor(COLORS[entry.level]))
            fmt.setFontWeight(400)
            if entry.level == "FATAL":
                fmt.setBackground(QColor("#c62828"))
                fmt.setFontWeight(700)
            cursor.insertText(entry.display(), fmt)
        cursor.endEditBlock()
        bar.setValue(bar.maximum() if follow else old_value)
        self._rendering = False
        self.highlight_matches()

    def _scroll_changed(self, value):
        if self._rendering:
            return
        self._follow = value >= self.view.verticalScrollBar().maximum() - 4
        if self._follow:
            self.unread = 0
            self.new_button.setText("↓ 最新")

    def follow_latest(self):
        self.pause.setChecked(False)
        self._follow = True
        self.unread = 0
        self.flush()
        self.view.verticalScrollBar().setValue(self.view.verticalScrollBar().maximum())
        self.new_button.setText("↓ 最新")

    def _pause_changed(self, paused):
        self.pause.setText("继续显示" if paused else "暂停显示")
        if not paused:
            self.flush()

    def schedule_filter(self, *_):
        self._filter_timer.start()

    def rebuild(self):
        error = ""
        if self.regex.isChecked():
            try:
                re.compile(self.filter.text())
            except re.error as exc:
                error = f"正则表达式无效：{exc}"
        self.filter_error.setText(error)
        self.filter_error.setVisible(bool(error))
        visible = [entry for entry in self.entries if self.matches(entry)]
        self.lines.replace(visible)
        self._rendering = True
        self.view.clear()
        self._rendering = False
        self._insert(visible, update_model=False)
        if self._follow:
            self.view.verticalScrollBar().setValue(self.view.verticalScrollBar().maximum())
        self.countsChanged.emit()

    def clear(self):
        self.pending.clear()
        self.entries.clear()
        self.partials.clear()
        self.warnings = self.errors = self.unread = self.dropped = 0
        self._seen_fatal.clear()
        self.fatal_banner.hide()
        self.lines.replace([])
        self.view.clear()
        self._follow = True
        self.flush()
        self.new_button.setText("↓ 最新")

    def next_error(self):
        self.set_expanded(True)
        self.pause.setChecked(False)
        self.flush()
        if not any(entry.level in ("ERROR", "FATAL") and self.matches(entry) for entry in self.entries):
            if any(entry.level in ("ERROR", "FATAL") for entry in self.entries):
                self.levels["ERROR"].setChecked(True)
                self.levels["FATAL"].setChecked(True)
                self.source.setCurrentIndex(0)
                self.filter.clear()
                self.rebuild()
        cursor = self.view.textCursor()
        start = cursor.blockNumber() + 1
        blocks = []
        block = self.view.document().begin()
        while block.isValid():
            if re.search(r"\s(?:ERROR|FATAL)\s", block.text()):
                blocks.append(block)
            block = block.next()
        if not blocks:
            self.counts.setToolTip("当前过滤条件下没有可见错误，请调整级别、来源或文本过滤")
            return
        block = next((block for block in blocks if block.blockNumber() >= start), blocks[0])
        self.view.setTextCursor(QTextCursor(block))
        self.view.centerCursor()
        self._follow = False

    def open_find(self):
        self.set_expanded(True)
        self.find_row.show()
        self.find.setFocus()

    def highlight_matches(self):
        selections = []
        query = self.find.text()
        if query:
            cursor = QTextCursor(self.view.document())
            for _ in range(1000):
                cursor = self.view.document().find(query, cursor)
                if cursor.isNull():
                    break
                selection = QTextEdit.ExtraSelection()
                selection.cursor = cursor
                selection.format.setBackground(QColor("#775b12"))
                selections.append(selection)
        self.view.setExtraSelections(selections)

    def find_next(self):
        if not self.view.find(self.find.text()):
            cursor = self.view.textCursor()
            cursor.movePosition(QTextCursor.MoveOperation.Start)
            self.view.setTextCursor(cursor)
            self.view.find(self.find.text())

    def set_expanded(self, expanded):
        if self._dialog:
            self._dialog.raise_()
            return
        self.toggle.setChecked(expanded)
        self.toggle.setText("▾ 实时日志" if expanded else "▸ 实时日志")
        self.body.setVisible(expanded)
        self.setMinimumHeight(120 if expanded else 32)
        self.setMaximumHeight(16777215 if expanded else 32)
        self.layout_changed.emit()

    def toggle_detached(self):
        if self._dialog:
            self._dialog.close()
            return
        self.set_expanded(True)
        dialog = LogDialog(self.window())
        dialog.setWindowTitle("OpenArm · 实时日志")
        dialog.setMinimumSize(760, 420)
        layout = QVBoxLayout(dialog)
        layout.addWidget(self.body)
        self._dialog = dialog
        self.shortcut.setParent(self.body)
        geometry = self.settings.value("log/dialogGeometry")
        if geometry:
            dialog.restoreGeometry(geometry)
        else:
            screen = dialog.screen().availableGeometry()
            dialog.resize(min(1280, int(screen.width() * .9)), min(800, int(screen.height() * .85)))
        dialog.closing.connect(self._reattach)
        self.detach_button.setText("收回面板 ↙")
        self.toggle.setEnabled(False)
        self.setMinimumHeight(32)
        self.setMaximumHeight(32)
        self.layout_changed.emit()
        dialog.show()

    def _reattach(self):
        dialog = self._dialog
        self.settings.setValue("log/dialogGeometry", dialog.saveGeometry())
        self._layout.addWidget(self.body, 1)
        self._dialog = None
        self.toggle.setEnabled(True)
        self.detach_button.setText("独立窗口 ↗")
        self.set_expanded(True)
        dialog.deleteLater()

    def save_settings(self):
        self.settings.setValue("log/expanded", self.toggle.isChecked())
        if self._dialog:
            self.settings.setValue("log/dialogGeometry", self._dialog.saveGeometry())
        if self._archive:
            self._archive.flush()
