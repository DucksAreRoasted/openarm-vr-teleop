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
    QCheckBox, QComboBox, QDialog, QFileDialog, QHBoxLayout, QLabel, QLineEdit,
    QMessageBox, QPlainTextEdit, QProgressBar, QPushButton, QVBoxLayout,
)

from .services import ROOT
from .theme import fit_window


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
        self._run_id = 0
        self._log_path: Path | None = None
        layout = QVBoxLayout(self)
        description = QLabel("先用 Quest 遥操、KER 或同向示教录制动作，停录并标注后，停止并释放控制器，再选择片段回放。\n"
                             "回放驱动左右臂及夹爪；先慢速接入首帧，结束后保持末帧，点击停止才释放。")
        description.setWordWrap(True)
        layout.addWidget(description)
        paths = QHBoxLayout()
        self.source = TrajectoryPath(source)
        paths.addWidget(self.source, 1)
        self.folder = QPushButton("选片段目录")
        self.file = QPushButton("选 CSV")
        self.folder.clicked.connect(self._choose_folder)
        self.file.clicked.connect(self._choose_file)
        paths.addWidget(self.folder); paths.addWidget(self.file)
        layout.addLayout(paths)
        options = QHBoxLayout()
        self.mode = QComboBox(); self.mode.addItems(["仿真回放", "真机回放"])
        self.speed = QComboBox()
        for label, value in (("0.25 倍", .25), ("0.5 倍", .5), ("1 倍", 1.0)):
            self.speed.addItem(label, value)
        self.speed.setCurrentIndex(1)
        self.units = QComboBox(); self.units.addItems(["源数据：真机夹爪角度", "源数据：仿真夹爪位移"])
        self.source.textChanged.connect(self._source_changed)
        self._source_changed(source)
        options.addWidget(self.mode); options.addWidget(self.speed); options.addWidget(self.units)
        layout.addLayout(options)
        self.safety = QCheckBox(f"真机确认：CAN 已配置（左 {left_can}、右 {right_can}），工作区清空，急停可用")
        layout.addWidget(self.safety)
        self.mode.currentIndexChanged.connect(self._update_controls)
        self.safety.toggled.connect(self._update_controls)
        controls = QHBoxLayout()
        self.start = QPushButton("开始回放"); self.start.setObjectName("Primary")
        self.pause = QPushButton("暂停")
        self.stop = QPushButton("停止并释放"); self.stop.setObjectName("Danger")
        self.close_button = QPushButton("关闭")
        self.start.clicked.connect(self._start)
        self.pause.clicked.connect(self._pause)
        self.stop.clicked.connect(self._stop)
        self.close_button.clicked.connect(self.reject)
        for button in (self.start, self.pause, self.stop, self.close_button): controls.addWidget(button)
        layout.addLayout(controls)
        self.progress = QProgressBar(); self.progress.setRange(0, 1000)
        layout.addWidget(self.progress)
        self.status = QLabel("请选择轨迹；默认仿真、0.5 倍速度。真机前请先验证仿真。")
        self.status.setWordWrap(True); layout.addWidget(self.status)
        self.log = QPlainTextEdit(); self.log.setReadOnly(True); self.log.setMaximumBlockCount(1000)
        layout.addWidget(self.log, 1)
        self.start.setToolTip("检查整段文件和关节限位，通过后慢速接入首帧，再播放。不会回放录制时的力矩。")
        self.pause.setToolTip("暂停轨迹时间，双臂继续保持当前位置附近；继续后从暂停位置播放。")
        self.stop.setToolTip("停止编译或回放；控制器退出并失能双臂，请先托住机械臂。")
        self.folder.setToolTip("选择批次下的片段目录（例如 recordings/sessions/批次/01）。")
        self.file.setToolTip("选择按左 8、右 8 顺序保存的合并轨迹 CSV。")
        self.units.setToolTip("按录制来源选择夹爪单位；仿真和真机之间回放会转换夹爪行程。")
        self.speed.setToolTip("设置播放速度上限；受安全限速影响时会自动放慢，保持轨迹顺序。")
        self.close_button.setToolTip("关闭前先停止回放并释放双臂。")
        self._update_controls()
        fit_window(self, preferred_w=950, preferred_h=600, min_w=750, min_h=480)

    def _running(self) -> bool:
        return self.process.state() != QProcess.ProcessState.NotRunning

    def _update_controls(self, *_args) -> None:
        running = self._running()
        for widget in (self.source, self.folder, self.file, self.mode, self.speed, self.units, self.safety):
            widget.setEnabled(not running)
        self.start.setEnabled(not running and (self.mode.currentIndex() == 0 or self.safety.isChecked()))
        self.pause.setEnabled(running and self._playing and not self._complete and not self._stopping)
        self.stop.setEnabled(running and not self._stopping)

    def _choose_folder(self) -> None:
        path = QFileDialog.getExistingDirectory(self, "选择动作片段目录", str(ROOT / "recordings/sessions"))
        if path: self.source.setText(path)

    def _source_changed(self, text: str) -> None:
        path = Path(text).expanduser()
        directory = path if path.is_dir() else path.parent
        try:
            meta = json.loads((directory / "collection_mode.json").read_text(encoding="utf-8"))
            if meta.get("mode") == "ker_v1" and meta.get("gripper_units") in ("meters", "radians"):
                self.units.setCurrentIndex(1 if meta["gripper_units"] == "meters" else 0)
        except (OSError, ValueError, AttributeError):
            pass

    def _choose_file(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "选择双臂轨迹", str(ROOT / "recordings/sessions"), "轨迹 (*.csv)")
        if path: self.source.setText(path)

    def _start(self) -> None:
        if self._running(): return
        source = Path(self.source.text()).expanduser()
        if not self.source.text().strip() or not source.exists():
            QMessageBox.warning(self, "轨迹无效", "请选择存在的片段目录或双臂 CSV。")
            return
        real = self.mode.currentIndex() == 1
        if real and (not self.safety.isChecked() or not self.left_can or not self.right_can or self.left_can == self.right_can):
            QMessageBox.warning(self, "无法回放", "请确认两个不同的 CAN 接口及真机安全条件。")
            return
        self.log.clear(); self.progress.setValue(0)
        self._run_id += 1
        self._buffer = ""; self._error_text = ""; self._paused = False
        self._stopping = False; self._complete = False; self._playing = False
        self.pause.setText("暂停")
        command = [str(ROOT / "script/replay_teaching.py"), str(source.resolve()), "--real" if real else "--sim",
                   "--speed", str(self.speed.currentData()), "--left-can", self.left_can, "--right-can", self.right_can]
        if self.units.currentIndex() == 1: command.append("--source-meters")
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
            if "[replay] ALIGNING" in line: self.status.setText("正在慢速接入首帧…")
            if "[replay] PLAYING" in line: self._playing = True
            match = re.search(r"PROGRESS time=([\d.e+-]+) duration=([\d.e+-]+) phase=(\w+)", line)
            if match and not self._stopping:
                instant, duration = float(match[1]), float(match[2])
                phase = {"aligning": "接入首帧", "playing": "回放中", "paused": "已暂停", "complete": "已完成，保持末帧"}[match[3]]
                self.progress.setValue(int(1000*instant/duration) if duration > 0 else 0)
                self.status.setText(f"{phase}  {instant:.1f} / {duration:.1f} 秒")
            if "[replay] COMPLETE" in line: self._complete = True
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
