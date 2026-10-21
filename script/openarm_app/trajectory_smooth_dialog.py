"""Offline smoothing controls and a link to replay the generated derivative."""
from __future__ import annotations

import json
from pathlib import Path
import sys

from PySide6.QtCore import QProcess, Signal
from PySide6.QtWidgets import (QComboBox, QDialog, QDoubleSpinBox, QFormLayout,
                               QHBoxLayout, QLabel, QPushButton, QVBoxLayout)

from .services import ROOT
from .theme import APP_STYLE, fit_window


class TrajectorySmoothDialog(QDialog):
    replay_requested = Signal(str)

    def __init__(self, source: Path, parent=None) -> None:
        super().__init__(parent)
        self.setStyleSheet(APP_STYLE)
        self.source = source
        self.report: dict | None = None
        self._output = bytearray()
        self.setWindowTitle("平滑录制动作")
        layout = QVBoxLayout(self)
        hint = QLabel("减少动作中的短时抖动，生成独立副本。首尾位置、动作时长和夹爪时序保持原样。平滑后的动作需要先预览检查。")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        settings = QFormLayout()
        self.strength = QComboBox()
        for label, value in (("轻度 · 80 ms", 80), ("标准 · 120 ms", 120), ("较强 · 200 ms", 200)):
            self.strength.addItem(label, value)
        self.strength.setCurrentIndex(1)
        self.deviation = QDoubleSpinBox()
        self.deviation.setRange(.1, 10)
        self.deviation.setDecimals(1)
        self.deviation.setValue(3)
        self.deviation.setSuffix(" °")
        settings.addRow("平滑强度", self.strength)
        settings.addRow("关节最大改动", self.deviation)
        layout.addLayout(settings)
        note = QLabel("训练动作指令副本会单独保存；实测观测、图像和时间戳保留原值。平滑可能改变接触动作，需检查后再用于训练。")
        note.setWordWrap(True)
        note.setObjectName("Hint")
        layout.addWidget(note)
        self.status = QLabel(str(source))
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        buttons = QHBoxLayout()
        self.generate = QPushButton("生成平滑副本")
        self.generate.setObjectName("Primary")
        self.replay = QPushButton("回放平滑副本…")
        self.replay.setEnabled(False)
        close = QPushButton("关闭")
        self.generate.clicked.connect(self._generate)
        self.replay.clicked.connect(self._replay)
        close.clicked.connect(self.reject)
        for button in (self.generate, self.replay, close): buttons.addWidget(button)
        layout.addLayout(buttons)
        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self.process.readyReadStandardOutput.connect(self._read)
        self.process.finished.connect(self._finished)
        self.process.errorOccurred.connect(self._error)
        fit_window(self, preferred_w=620, preferred_h=380, min_w=480, min_h=320)

    def _generate(self) -> None:
        if self.process.state() != QProcess.ProcessState.NotRunning: return
        self.report = None
        self._output.clear()
        self.generate.setEnabled(False)
        self.replay.setEnabled(False)
        self.strength.setEnabled(False)
        self.deviation.setEnabled(False)
        self.status.setText("正在离线平滑动作…")
        self.process.start(sys.executable, [str(ROOT / "script/smooth_recording.py"),
                           str(self.source), "--window-ms", str(self.strength.currentData()),
                           "--max-deviation-deg", str(self.deviation.value())])

    def _read(self) -> None:
        self._output.extend(bytes(self.process.readAllStandardOutput()))

    def _error(self, error) -> None:
        if error == QProcess.ProcessError.FailedToStart:
            self._finished(-1, QProcess.ExitStatus.CrashExit)

    def _finished(self, code: int, status) -> None:
        self._read()
        self.generate.setEnabled(True)
        self.strength.setEnabled(True)
        self.deviation.setEnabled(True)
        text = self._output.decode("utf-8", errors="replace").strip()
        try:
            if code != 0 or status != QProcess.ExitStatus.NormalExit:
                raise ValueError(text or self.process.errorString())
            self.report = json.loads(text.splitlines()[-1])
            stats = self.report["trajectory_stats"]
            peak = max(stats["max_deviation_deg"])
            before, after = stats["velocity_rms_before_rad_s"], stats["velocity_rms_after_rad_s"]
            self.status.setText(
                f"已生成 {stats['frames']} 帧，时长 {stats['duration_s']:.2f} 秒。\n"
                f"最大位置改动 {peak:.2f}°；关节速度 RMS {before:.3f} → {after:.3f} rad/s。\n"
                f"保存目录：{self.report['output_dir']}" +
                ("\n已同时生成训练动作指令副本。" if self.report.get("training_actions") else ""))
            self.replay.setEnabled(True)
        except (ValueError, KeyError, IndexError, TypeError) as error:
            self.report = None
            self.status.setText(f"平滑失败：{error}")

    def _replay(self) -> None:
        if self.report is None: return
        source = self.report["output_dir"]
        self.accept()
        self.replay_requested.emit(source)

    def reject(self) -> None:
        if self.process.state() != QProcess.ProcessState.NotRunning:
            self.process.kill()
            self.process.waitForFinished(1000)
        super().reject()
