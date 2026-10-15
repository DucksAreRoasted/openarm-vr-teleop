"""Exclusive teaching replay with an independent, interruptible controller process."""
from __future__ import annotations

import os
from datetime import datetime
import json
from pathlib import Path
import re
import signal
import sys
import tempfile

from PySide6.QtCore import QProcess, Qt, QTimer
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDialog, QFileDialog, QFormLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
    QMessageBox, QPlainTextEdit, QPushButton, QScrollArea, QSlider, QSpinBox, QVBoxLayout, QWidget,
)

from .services import ROOT
from .theme import fit_window

# 控制端日后可能在进度行追加 loop=<n> loops=<总数或 inf>；字段缺失时回退到界面状态。
_PHASE_LABELS = {
    "aligning": "接入录制首帧",
    "startup": "执行起手轨迹",
    "connecting": "衔接录制首帧",
    "preparing": "准备返回轨迹，保持末帧",
    "playing": "回放中",
    "paused": "已暂停",
    "complete": "已完成，保持末帧",
    "ready": "已到达起手位置，等待开始",
}
_PROGRESS_RE = re.compile(r"PROGRESS time=([\d.eE+-]+) duration=([\d.eE+-]+) phase=(\w+)")
_LOOP_RE = re.compile(r"(?<![A-Za-z])loop\s*=\s*(\d+)\b")
_LOOPS_RE = re.compile(r"loops\s*=\s*(inf|\d+)\b", re.IGNORECASE)


class TrajectoryPath(QLineEdit):
    def __init__(self, text: str = "") -> None:
        super().__init__(text)
        self.setAcceptDrops(True)
        self.setPlaceholderText("选择片段目录或双臂 CSV，也可拖入文件/目录")

    def dragEnterEvent(self, event) -> None:
        if event.mimeData().hasUrls() and len(event.mimeData().urls()) == 1 and event.mimeData().urls()[0].isLocalFile():
            event.acceptProposedAction()
        else:
            event.ignore()

    def dropEvent(self, event) -> None:
        if event.mimeData().hasUrls() and len(event.mimeData().urls()) == 1:
            path = event.mimeData().urls()[0].toLocalFile()
            if path:
                self.setText(path)
                event.acceptProposedAction()


class ReplayDialog(QDialog):
    def __init__(self, parent=None, *, source: str = "", left_can: str = "can1", right_can: str = "can0") -> None:
        super().__init__(parent)
        self.setWindowTitle("动作回放")
        self.setWindowModality(Qt.WindowModality.ApplicationModal)
        self.left_can, self.right_can = left_can, right_can
        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self.process.readyReadStandardOutput.connect(self._read_output)
        self.process.finished.connect(self._finished)
        self.process.errorOccurred.connect(self._error)
        self._buffer = ""
        self._paused = False
        self._closing = False
        self._stopping = False
        self._error_text = ""
        self._playing = False
        self._complete = False
        self._ready = False
        self._pending_action: str | None = None
        self._duration = 0.0
        self._aligning = False
        self._cycle = 1
        self._run_id = 0
        self._log_path: Path | None = None
        layout = QVBoxLayout(self)
        title = QLabel("动作回放")
        title.setObjectName("HeroTitle")
        layout.addWidget(title)
        description = QLabel("选择动作 → 到达起手位置 → 点击开始回放。回放前请停止遥操控制器。")
        description.setObjectName("Hint")
        description.setWordWrap(True)
        layout.addWidget(description)
        configuration = QWidget()
        configuration_layout = QVBoxLayout(configuration)
        configuration_layout.setContentsMargins(0, 0, 0, 0)
        configuration_scroll = QScrollArea()
        configuration_scroll.setWidgetResizable(True)
        configuration_scroll.setWidget(configuration)
        layout.addWidget(configuration_scroll, 1)
        source_box = QGroupBox("1 · 选择动作")
        source_layout = QVBoxLayout(source_box)
        paths = QHBoxLayout()
        self.source = TrajectoryPath(source)
        paths.addWidget(self.source, 1)
        self.folder = QPushButton("选片段目录")
        self.file = QPushButton("选 CSV")
        self.folder.clicked.connect(self._choose_folder)
        self.file.clicked.connect(self._choose_file)
        paths.addWidget(self.folder); paths.addWidget(self.file)
        source_layout.addLayout(paths)
        configuration_layout.addWidget(source_box)
        settings_box = QGroupBox("2 · 回放设置")
        options = QFormLayout(settings_box)
        self.mode = QComboBox(); self.mode.addItems(["仿真预览（按录制时间）", "真机回放"])
        self.speed = QComboBox()
        for label, value in (("0.25 倍", .25), ("0.5 倍", .5), ("1 倍", 1.0)):
            self.speed.addItem(label, value)
        self.speed.setCurrentIndex(2)
        self.units = QComboBox(); self.units.addItems(["源数据：真机夹爪角度", "源数据：仿真夹爪位移"])
        self.source.textChanged.connect(self._source_changed)
        self._source_changed(source)
        options.addRow("运行环境", self.mode)
        options.addRow("播放速度", self.speed)
        options.addRow("录制来源", self.units)
        repeats = QHBoxLayout()
        self.repeat = QSpinBox()
        self.repeat.setRange(0, 10000)
        self.repeat.setValue(1)
        self._active_repeat = 1
        self._finish_requested = False
        self.repeat.setSpecialValueText("持续循环")
        self.repeat.setSuffix(" 遍")
        self.repeat.setToolTip("仿真预览直接从首帧循环；真机保持使能并慢速返回首帧，返回路径须检查障碍物。")
        repeats.addWidget(self.repeat)
        self.forever = QCheckBox("持续循环")
        self.forever.toggled.connect(self._forever_changed)
        self.repeat.valueChanged.connect(self._repeat_changed)
        repeats.addWidget(self.forever)
        options.addRow("播放次数", repeats)
        playback_hint = QLabel("先点击“到达起手位置”，双臂完整执行你的起手轨迹，再平滑衔接录制首帧并保持；到位后点击“开始回放”。全部播完保持末帧，可返回录制首帧后再次开始。设置多遍时，每遍之间自动循环。仿真预览不验证真机跟踪或返回路径。")
        playback_hint.setWordWrap(True)
        playback_hint.setObjectName("Hint")
        options.addRow(playback_hint)
        configuration_layout.addWidget(settings_box)
        self.safety = QCheckBox(f"真机确认：CAN 已配置（左 {left_can}、右 {right_can}），工作区清空，急停可用")
        layout.addWidget(self.safety)
        self.mode.currentIndexChanged.connect(self._update_controls)
        self.safety.toggled.connect(self._update_controls)
        controls = QHBoxLayout()
        self.prepare = QPushButton("到达起手位置"); self.prepare.setObjectName("Accent")
        self.start = QPushButton("开始回放"); self.start.setObjectName("Primary")
        self.pause = QPushButton("暂停")
        self.finish_loop = QPushButton("结束循环")
        self.stop = QPushButton("停止并释放"); self.stop.setObjectName("Danger")
        self.close_button = QPushButton("关闭")
        self.prepare.clicked.connect(self._prepare)
        self.start.clicked.connect(self._start)
        self.pause.clicked.connect(self._pause)
        self.finish_loop.clicked.connect(self._finish_loop)
        self.stop.clicked.connect(self._stop)
        self.close_button.clicked.connect(self.reject)
        for button in (self.prepare, self.start, self.pause, self.finish_loop, self.stop, self.close_button): controls.addWidget(button)
        layout.addLayout(controls)
        self.progress = QSlider(Qt.Orientation.Horizontal)
        self.progress.setRange(0, 1000)
        self.progress.setTracking(False)
        self.progress.setToolTip("仿真预览中拖动滑块，松开后跳转；暂停时可定位查看姿态。")
        self.progress.sliderMoved.connect(self._scrub_preview)
        self.progress.valueChanged.connect(self._seek)
        layout.addWidget(self.progress)
        self.progress_time = QLabel("0.0 / 0.0 秒")
        layout.addWidget(self.progress_time)
        self.status = QLabel("请选择轨迹，先到达起手位置；到位后才能开始回放。默认仿真、1 倍原速。")
        self.status.setWordWrap(True); layout.addWidget(self.status)
        self.log_toggle = QCheckBox("显示运行日志")
        layout.addWidget(self.log_toggle)
        self.log = QPlainTextEdit(); self.log.setReadOnly(True); self.log.setMaximumBlockCount(1000)
        self.log.setMinimumHeight(100)
        self.log.setVisible(False)
        self.log_toggle.toggled.connect(self.log.setVisible)
        layout.addWidget(self.log, 1)
        self.prepare.setToolTip("检查文件后，完整执行 Quest 起手轨迹，再平滑衔接录制首帧并保持，等待手动开始。播完后直接平滑返回录制首帧，不重复起手轨迹；修改动作或设置需先停止并释放。")
        self.start.setToolTip("机械臂到达起手位置后，点击此按钮才按所设速度和次数回放动作。")
        self.pause.setToolTip("暂停轨迹时间，双臂继续保持当前位置附近；继续后从暂停位置播放。")
        self.finish_loop.setToolTip("播完当前这一遍后结束循环并保持末帧；暂停时需先继续。")
        self.stop.setToolTip("停止编译或回放；控制器退出并失能双臂，请先托住机械臂。")
        self.folder.setToolTip("选择批次下的片段目录（例如 recordings/sessions/批次/01）。")
        self.file.setToolTip("选择按左 8、右 8 顺序保存的合并轨迹 CSV。")
        self.units.setToolTip("按录制来源选择夹爪单位；仿真和真机之间回放会转换夹爪行程。")
        self.speed.setToolTip("仿真：录制时间相对现实时间的倍率。真机：播放速度上限，受安全限速影响时会自动放慢。")
        self.close_button.setToolTip("关闭前先停止回放并释放双臂。")
        self._update_controls()
        fit_window(self, preferred_w=950, preferred_h=600, min_w=750, min_h=480)

    def _running(self) -> bool:
        return self.process.state() != QProcess.ProcessState.NotRunning

    def _forever_changed(self, checked: bool) -> None:
        self.repeat.setValue(0 if checked else max(1, self.repeat.value()))

    def _repeat_changed(self, value: int) -> None:
        self.forever.blockSignals(True)
        self.forever.setChecked(value == 0)
        self.forever.blockSignals(False)

    def _update_controls(self, *_args) -> None:
        running = self._running()
        for widget in (self.source, self.folder, self.file, self.mode, self.speed, self.units, self.safety):
            widget.setEnabled(not running)
        available = not self._stopping and not self._closing and self._pending_action is None
        for widget in (self.repeat, self.forever):
            widget.setEnabled(available and (not running or self._complete or self._ready))
        safe = self.mode.currentIndex() == 0 or self.safety.isChecked()
        self.prepare.setText("返回录制首帧" if running and self._complete else "到达起手位置")
        self.prepare.setEnabled((not running or self._complete) and available and safe)
        self.start.setEnabled(running and self._ready and available and safe)
        self.pause.setEnabled(running and self._playing and not self._complete and not self._stopping
                              and not self._ready and self._pending_action is None)
        self.finish_loop.setEnabled(running and self._playing and not self._aligning
                                    and not self._complete and not self._ready and available
                                    and self._active_repeat == 0 and not self._finish_requested)
        self.stop.setEnabled(running and not self._stopping)
        self.progress.setEnabled(running and self._playing and self.mode.currentIndex() == 0
                                 and self._duration > 0 and not self._stopping and not self._aligning
                                 and not self._ready and self._pending_action is None)

    def _scrub_preview(self, value: int) -> None:
        self.progress_time.setText(f"定位到 {self._duration * value / 1000:.1f} / {self._duration:.1f} 秒")

    def _seek(self, *_args) -> None:
        if not self._running() or not self.progress.isEnabled() or self.mode.currentIndex() != 0:
            return
        target = self._duration * self.progress.sliderPosition() / 1000
        self.process.write(f"seek {target:.9f}\n".encode())

    def _choose_folder(self) -> None:
        path = QFileDialog.getExistingDirectory(self, "选择动作片段目录", str(ROOT / "recordings/sessions"))
        if path: self.source.setText(path)

    def _source_changed(self, text: str) -> None:
        path = Path(text).expanduser()
        directory = path if path.is_dir() else path.parent
        try:
            meta = json.loads((directory / "collection_mode.json").read_text(encoding="utf-8"))
            if meta.get("gripper_units") in ("meters", "radians"):
                self.units.setCurrentIndex(1 if meta["gripper_units"] == "meters" else 0)
        except (OSError, ValueError, AttributeError):
            pass

    def _choose_file(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "选择双臂轨迹", str(ROOT / "recordings/sessions"), "轨迹 (*.csv)")
        if path: self.source.setText(path)

    def _start(self) -> None:
        if not self._running() or not self._ready or self._pending_action is not None or self._stopping or self._closing:
            return
        command = b"play\n"
        if self.repeat.value() != self._active_repeat or self._finish_requested:
            command = f"repeat {self.repeat.value()}\n".encode() + command
        if self.process.write(command) != len(command):
            self.status.setText("开始指令发送失败，请停止并释放后重试。")
            return
        self._active_repeat = self.repeat.value()
        self._finish_requested = False
        self._pending_action = "play"
        self._ready = False
        self._append_log("[replay-ui] command=play")
        self.status.setText("正在开始回放…")
        self._update_controls()

    def _prepare(self) -> None:
        if self._running():
            if not self._complete or self._pending_action is not None or self._stopping or self._closing:
                return
            if self.process.write(b"prepare\n") != len(b"prepare\n"):
                self.status.setText("起手位置指令发送失败，请停止并释放后重试。")
                return
            self._pending_action = "prepare"
            self._ready = False
            self._complete = False
            self._paused = False
            self._aligning = True
            self._cycle = 1
            self.pause.setText("暂停")
            self.progress.blockSignals(True)
            self.progress.setValue(0)
            self.progress.blockSignals(False)
            self.progress_time.setText(f"0.0 / {self._duration:.1f} 秒")
            self._append_log("[replay-ui] command=prepare")
            self.status.setText("正在返回录制首帧…")
            self._update_controls()
            return
        source = Path(self.source.text()).expanduser()
        if not self.source.text().strip() or not source.exists():
            QMessageBox.warning(self, "轨迹无效", "请选择存在的片段目录或双臂 CSV。")
            return
        real = self.mode.currentIndex() == 1
        if real and (not self.safety.isChecked() or not self.left_can or not self.right_can or self.left_can == self.right_can):
            QMessageBox.warning(self, "无法回放", "请确认两个不同的 CAN 接口及真机安全条件。")
            return
        self.log.clear(); self.progress.setValue(0)
        self._duration = 0.0
        self._aligning = False
        self.progress_time.setText("0.0 / 0.0 秒")
        self._run_id += 1
        self._buffer = ""; self._error_text = ""; self._paused = False
        self._stopping = False; self._complete = False; self._playing = False
        self._ready = False
        self._pending_action = None
        self._cycle = 1
        self.pause.setText("暂停")
        command = [str(ROOT / "script/replay_teaching.py"), str(source.resolve()), "--real" if real else "--sim",
                   "--speed", str(self.speed.currentData()), "--left-can", self.left_can, "--right-can", self.right_can]
        if self.units.currentIndex() == 1: command.append("--source-meters")
        self._active_repeat = self.repeat.value()
        self._finish_requested = False
        command.extend(["--repeat", str(self.repeat.value()), "--wait-for-start"])
        self._log_path = Path(tempfile.gettempdir()) / (
            f"openarm_teaching_replay_{datetime.now():%Y%m%d_%H%M%S_%f}_{os.getpid()}.log")
        try:
            self._log_path.touch(exist_ok=False)
        except OSError as error:
            self._log_path = None
            self.log.appendPlainText(f"[replay-ui] 无法创建日志：{error}")
        if self._log_path:
            print(f"[replay-ui] log={self._log_path}", flush=True)
            self._append_log(f"[replay-ui] log={self._log_path}")
        self._append_log("[replay-ui] command=" + json.dumps([sys.executable, *command], ensure_ascii=False))
        self.process.setWorkingDirectory(str(ROOT))
        self.process.start(sys.executable, command)
        self.status.setText("正在检查文件并编译回放控制器…")
        self._update_controls()

    def _append_log(self, line: str) -> None:
        self.log.appendPlainText(line)
        if self._log_path is not None:
            try:
                with self._log_path.open("a", encoding="utf-8") as handle:
                    handle.write(line + "\n")
            except OSError as error:
                self._log_path = None
                self.log.appendPlainText(f"[replay-ui] 日志写入失败：{error}")

    def _read_output(self) -> None:
        self._buffer += bytes(self.process.readAllStandardOutput()).decode("utf-8", errors="replace")
        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
            self._append_log(line)
            if "[replay][ERROR]" in line: self._error_text = line
            if "[replay] ALIGNING" in line:
                self._pending_action = None
                self._ready = False
                self._complete = False
                self._playing = True
                self._aligning = True
                cycle = re.search(r"cycle=(\d+)", line)
                if cycle: self._cycle = int(cycle[1])
                self.status.setText("正在沿起手轨迹准备并衔接首帧…")
            if "[replay] PLAYING" in line:
                self._pending_action = None
                self._ready = False
                self._complete = False
                self._playing = True
                self._aligning = False
            if "[replay] READY" in line and self._pending_action != "play":
                self._pending_action = None
                self._ready = True
                self._playing = False
                self._aligning = False
                self._complete = False
                self._paused = False
                self.pause.setText("暂停")
                self.status.setText("已到达起手位置并保持，请点击“开始回放”。")
            match = _PROGRESS_RE.search(line)
            if match and not self._stopping and self._pending_action is None:
                instant, duration = float(match[1]), float(match[2])
                self._duration = duration
                if not self.progress.isSliderDown():
                    self.progress.blockSignals(True)
                    self.progress.setValue(int(1000 * instant / duration) if duration > 0 else 0)
                    self.progress.blockSignals(False)
                    self.progress_time.setText(f"{instant:.1f} / {duration:.1f} 秒")
                self._complete = match[3] == "complete"
                self._ready = match[3] == "ready"
                self._paused = match[3] == "paused"
                if match[3] in {"startup", "connecting", "aligning", "preparing"}:
                    self._aligning = True
                self.pause.setText("继续" if self._paused else "暂停")
                self.status.setText(self._format_progress(line, match[3], instant, duration))
            if "[replay] COMPLETE" in line and self._pending_action is None:
                self._complete = True
        self._update_controls()

    def _format_progress(self, line: str, phase: str, instant: float, duration: float) -> str:
        loop = _LOOP_RE.search(line)
        loops = _LOOPS_RE.search(line)
        if loop:
            self._cycle = int(loop.group(1))
        if loops:
            raw = loops.group(1)
            total = "∞" if raw.lower() == "inf" or int(raw) == 0 else str(int(raw))
        else:
            total = "∞" if self.repeat.value() == 0 else str(self.repeat.value())
        if phase in {"startup", "connecting"}:
            timing = re.search(r"preparation_time=([\d.eE+-]+).*startup_duration=([\d.eE+-]+).*preparation_duration=([\d.eE+-]+)", line)
            if timing:
                elapsed, startup, preparation = map(float, timing.groups())
                instant, duration = (min(elapsed, startup), startup) if phase == "startup" else (max(0, elapsed-startup), max(0, preparation-startup))
        label = _PHASE_LABELS.get(phase, phase)
        if self._finish_requested and phase not in {"complete", "ready"}:
            label += " · 本遍结束后停止循环"
        return f"第 {self._cycle} / {total} 遍 · {label}  {instant:.1f} / {duration:.1f} 秒"

    def _finish_loop(self) -> None:
        if not self.finish_loop.isEnabled(): return
        command = b"finish_loop\n"
        if self.process.write(command) != len(command):
            self.status.setText("结束循环指令发送失败，请重试。")
            return
        self._finish_requested = True
        self._append_log("[replay-ui] command=finish_loop")
        self.status.setText("已请求结束循环，播完当前这一遍后保持末帧；暂停时请先继续。")
        self._update_controls()

    def _pause(self) -> None:
        if not self.pause.isEnabled(): return
        self._paused = not self._paused
        self.process.write(b"pause\n" if self._paused else b"resume\n")
        self.pause.setText("继续" if self._paused else "暂停")

    def _stop(self) -> None:
        if not self._running(): return
        self._stopping = True
        self.status.setText("正在停止并释放双臂…")
        self._signal(signal.SIGINT)
        run_id = self._run_id
        QTimer.singleShot(5000, lambda: self._signal(signal.SIGTERM) if self._stopping and self._run_id == run_id else None)
        self._update_controls()

    def _signal(self, sig: int) -> None:
        if not self._running(): return
        try:
            pid = int(self.process.processId())
            # Starting may report PID 0; signalling group 0 would stop the GUI.
            if pid <= 0 or os.getpgid(pid) != pid:
                QTimer.singleShot(50, lambda: self._signal(sig) if self._stopping else None)
                return
            os.killpg(pid, sig)
        except ProcessLookupError:
            # The Python launcher may not have established its session yet.
            QTimer.singleShot(50, lambda: self._signal(sig) if self._stopping else None)

    def _finished(self, code: int, _status) -> None:
        self._read_output()
        if self._buffer:
            self._append_log(self._buffer)
            if "[replay][ERROR]" in self._buffer: self._error_text = self._buffer
            self._buffer = ""
        self._append_log(f"[replay-ui] EXIT code={code} status={_status}")
        self.status.setText("已停止并释放" if self._stopping else self._error_text or f"回放进程已退出（{code}）")
        self._playing = False
        self._ready = False
        self._pending_action = None
        self._complete = False
        self._stopping = False
        self._update_controls()
        if self._closing: super().reject()

    def _error(self, error) -> None:
        if error == QProcess.ProcessError.FailedToStart:
            self.status.setText("回放进程启动失败：" + self.process.errorString())
            self._update_controls()

    def reject(self) -> None:
        if self._running():
            self._closing = True
            self._stop()
        else:
            super().reject()

    def closeEvent(self, event: QCloseEvent) -> None:
        if self._running():
            event.ignore(); self.reject()
        else:
            event.accept()
