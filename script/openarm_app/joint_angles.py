"""Loopback receiver for measured arm joint angles; no CAN access."""
import json
import math
import time

from PySide6.QtCore import QObject, Property, Signal, QTimer
from PySide6.QtNetwork import QHostAddress, QUdpSocket


class JointAngles(QObject):
    changed = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._samples = {}
        self.socket = QUdpSocket(self)
        self.available = self.socket.bind(
            QHostAddress(QHostAddress.SpecialAddress.LocalHost), 0)
        self.socket.readyRead.connect(self._receive)
        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.changed.emit)
        self.timer.start()

    def clear(self):
        self._samples.clear()
        while self.socket.hasPendingDatagrams():
            self.socket.receiveDatagram()
        self.changed.emit()

    def _receive(self):
        # Bound work per event; the display consumes only the latest sample.
        for _ in range(128):
            if not self.socket.hasPendingDatagrams():
                break
            data = self.socket.receiveDatagram().data().data()
            try:
                packet = json.loads(data)
                side, angles = packet['side'], packet['q']
                age = float(packet['age'])
                if side not in ('left', 'right') or len(angles) != 7:
                    continue
                if not all(type(q) in (int, float) and math.isfinite(q) for q in angles):
                    continue
                if not math.isfinite(age) or not 0 <= age < 0.5:
                    continue
                self._samples[side] = (angles, time.monotonic() - age)
            except (ValueError, KeyError, TypeError):
                continue

    @Property('QVariantList', notify=changed)
    def rows(self):
        rows = []
        now = time.monotonic()
        for side, label in (('left', '左臂'), ('right', '右臂')):
            sample = self._samples.get(side)
            fresh = sample is not None and now - sample[1] < 0.5
            rows.append({
                'side': label,
                'status': '实时反馈' if fresh else ('反馈超时' if sample else '无数据'),
                'values': [f'{q:.4f} rad / {math.degrees(q):.2f}°' for q in sample[0]]
                          if fresh else ['—'] * 7,
            })
        return rows
