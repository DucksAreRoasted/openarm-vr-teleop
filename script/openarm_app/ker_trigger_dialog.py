"""Measure KER gripper endpoints without launching a follower."""
from collections import deque
import json
import os
from pathlib import Path
import tempfile
import time

import yaml
from PySide6.QtCore import QProcess, QTimer
from PySide6.QtWidgets import QDialog, QLabel, QMessageBox, QPlainTextEdit, QPushButton, QVBoxLayout

from ker_bridge import ROOT, ker_python, load_mapping, trigger_endpoints


class KerTriggerDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("KER 双扳机行程标定")
        self.resize(680, 540)
        self.path = Path(os.environ.get("OPENARM_KER_CONFIG", str(ROOT / "config/ker.yaml")))
        self.mapping = load_mapping(self.path)
        self.original_config = self.path.read_bytes()
        self.samples = deque(maxlen=30)
        self.released = None
        self._buffer = ""
        self._closing = False
        self._error = ""
        self.minimum = None
        self.maximum = None
        self.device_stamp = None
        layout = QVBoxLayout(self)
        self.hint = QLabel("1. 完全松开两侧夹爪扳机，保持不动，然后采集松开位置。\n"
                           "2. 同时压到底并保持不动，采集压下位置并保存。\n"
                           "此窗口只读取 KER，不启动从臂。保存后仍需双扳机连续压住 2 秒才能遥操。")
        self.hint.setWordWrap(True)
        layout.addWidget(self.hint)
        self.readout = QLabel("正在连接 KER…")
        layout.addWidget(self.readout)
        self.channels = QPlainTextEdit()
        self.channels.setReadOnly(True)
        self.channels.setMaximumHeight(180)
        layout.addWidget(self.channels)
        reset = QPushButton("清零通道变化统计")
        reset.clicked.connect(self._reset_changes)
        layout.addWidget(reset)
        instructions = QLabel("排查时保持机械臂不动，只反复按左扳机；清零统计后再测右侧。\n"
                              "默认左扳机 CH15、右扳机 CH07（从 0 编号）。其他通道变化可能提示映射或接线问题。\n"
                              f"原始读数日志：{Path(tempfile.gettempdir()) / 'openarm_ker_inspect.jsonl'}")
        instructions.setWordWrap(True)
        layout.addWidget(instructions)
        self.capture = QPushButton("采集双扳机松开位置")
        self.capture.setEnabled(False)
        self.capture.clicked.connect(self._capture)
        layout.addWidget(self.capture)
        close = QPushButton("关闭")
        close.clicked.connect(self.close)
        layout.addWidget(close)
        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self.process.readyReadStandardOutput.connect(self._read)
        self.process.finished.connect(self._finished)
        self.process.errorOccurred.connect(self._failed)
        args = [str(ROOT / "script/ker_bridge.py"), "--inspect", "--config", str(self.path),
                "--transport", os.environ.get("OPENARM_KER_TRANSPORT", "usb"),
                "--serial-port", os.environ.get("OPENARM_KER_SERIAL_PORT", "/dev/ttyACM0")]
        self.process.start(ker_python(), args)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(100)

    def _read(self):
        self._buffer += bytes(self.process.readAllStandardOutput()).decode("utf-8", errors="replace")
        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
            if line.startswith("[ker-inspect] "):
                frame = json.loads(line.split(" ", 1)[1])
                raw = frame["angles"]
                self.samples.append((time.monotonic(), raw))
                self.device_stamp = frame.get("timestamp", frame.get("seq", "未提供"))
                if self.minimum is None:
                    self.minimum, self.maximum = raw.copy(), raw.copy()
                else:
                    self.minimum = [min(a,b) for a,b in zip(self.minimum, raw)]
                    self.maximum = [max(a,b) for a,b in zip(self.maximum, raw)]
            elif "[ERROR]" in line:
                self._error = line
        self._refresh()

    def _refresh(self):
        fresh = self.process.state() == QProcess.ProcessState.Running and bool(self.samples) and time.monotonic()-self.samples[-1][0] < .25
        self.capture.setEnabled(fresh and len(self._recent()) >= 5)
        if fresh:
            raw = self.samples[-1][1]
            left = raw[self.mapping["left"]["gripper_channel"]]
            right = raw[self.mapping["right"]["gripper_channel"]]
            self.readout.setText(f"左扳机：{left:.2f}°    右扳机：{right:.2f}°    设备时间戳/序号：{self.device_stamp}\n"
                                 "按动扳机时，两侧对应读数应随之变化。")
            if self.minimum is not None:
                rows = []
                for channel in range(16):
                    delta = self.maximum[channel]-self.minimum[channel]
                    label = "左扳机" if channel == self.mapping["left"]["gripper_channel"] else "右扳机" if channel == self.mapping["right"]["gripper_channel"] else "关节"
                    rows.append(f"CH{channel:02d}  {label}  当前 {raw[channel]:+.3f}°  累计变化 {delta:.3f}°")
                self.channels.setPlainText("\n".join(rows))
        elif self.samples:
            self.readout.setText(self._error or "数据已中断，不能采集或保存。")

    def _recent(self):
        now = time.monotonic()
        return [raw for stamp, raw in self.samples if now-stamp <= .5]

    def _reset_changes(self):
        self.minimum = self.maximum = None
        self.channels.clear()

    def _capture(self):
        if not self.capture.isEnabled():
            return
        samples = self._recent()
        try:
            if self.released is None:
                # Check stability before accepting the released endpoint.
                for side in ("left", "right"):
                    values = [raw[self.mapping[side]["gripper_channel"]] for raw in samples]
                    if max(values)-min(values) > 2:
                        raise ValueError(f"{side} 读数不稳定，请松开后保持不动半秒再采集")
                self.released = samples
                self.samples.clear()
                self.capture.setText("采集双扳机压到底位置并保存")
                self.hint.setText("现在同时将两侧扳机压到底，保持不动半秒，再点击采集并保存。")
                self._refresh()
                return
            result = trigger_endpoints(self.mapping, self.released, samples)
            if self.path.read_bytes() != self.original_config:
                raise ValueError("配置文件已被其他程序修改，请关闭窗口后重新标定")
            # Keep the original for rollback. Atomic replacement avoids partial YAML.
            backup = self.path.with_name(self.path.name + ".before-trigger-calibration")
            backup.write_bytes(self.path.read_bytes())
            temporary = None
            try:
                with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=self.path.parent,
                                                 prefix=".ker-calibration-", delete=False) as handle:
                    temporary = Path(handle.name)
                    yaml.safe_dump(result, handle, allow_unicode=True, sort_keys=False)
                load_mapping(temporary)
                temporary.replace(self.path)
            finally:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)
            QMessageBox.information(self, "已保存", "已保存实测扳机行程。关闭后启动 KER 仿真，\n"
                                    "再同时压到底并保持 2 秒。七关节方向与零点标定状态保持原值。")
            self.close()
        except (ValueError, OSError) as error:
            QMessageBox.warning(self, "标定未保存", str(error))

    def _failed(self, _error):
        self._error = self.process.errorString()
        self.readout.setText(self._error)
        self.capture.setEnabled(False)

    def _finished(self, _code, _status):
        if not self._closing:
            self.readout.setText(self._error or "KER 读取已结束，请关闭并重新连接。")
            self.capture.setEnabled(False)

    def closeEvent(self, event):
        self._closing = True
        self.timer.stop()
        self.process.terminate()
        if not self.process.waitForFinished(1500):
            self.process.kill()
            self.process.waitForFinished(500)
        super().closeEvent(event)
