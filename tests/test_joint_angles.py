"""Measured joint telemetry validation and stale-state handling."""
import json
import os
from pathlib import Path
import socket
import sys
import time
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'script'))
try:
    from PySide6.QtCore import QCoreApplication
    from openarm_app.joint_angles import JointAngles
    QT_AVAILABLE = True
except ImportError:
    QT_AVAILABLE = False


@unittest.skipUnless(QT_AVAILABLE, 'PySide6 required')
class JointAngleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    def test_udp_validation_latest_and_timeout(self):
        receiver = JointAngles()
        self.assertTrue(receiver.available)
        self.assertEqual(receiver.rows[0]['status'], '无数据')
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            for packet in ({'side': 'left', 'q': [0.1] * 7, 'age': 0},
                           {'side': 'left', 'q': [0.2] * 7, 'age': 0},
                           {'side': 'right', 'q': [float('nan')] * 7, 'age': 0},
                           {'side': 'right', 'q': [1] * 6, 'age': 0}):
                sender.sendto(json.dumps(packet).encode(), ('127.0.0.1', receiver.socket.localPort()))
            receiver._receive()
        self.assertEqual(receiver.rows[0]['status'], '实时反馈')
        self.assertIn('0.2000 rad', receiver.rows[0]['values'][0])
        self.assertEqual(receiver.rows[1]['status'], '无数据')
        with patch('openarm_app.joint_angles.time.monotonic', return_value=time.monotonic() + 1):
            self.assertEqual(receiver.rows[0]['status'], '反馈超时')
            self.assertEqual(receiver.rows[0]['values'], ['—'] * 7)
        receiver.clear()
        self.assertEqual(receiver.rows[0]['status'], '无数据')
        receiver.socket.close()
        receiver.timer.stop()
