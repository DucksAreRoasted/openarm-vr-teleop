"""Independent camera capture freshness and shared snapshot transport."""
import io
import sys
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'script'))
import lerobot_cameras as cameras


class FakeConnection:
    def __init__(self, reply=b'', command=b'', stop=None):
        self.reply = reply
        self.command = command
        self.sent = b''
        self.stop = stop

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def settimeout(self, timeout):
        pass

    def sendall(self, data):
        self.sent += data

    def makefile(self, mode):
        return io.BytesIO(self.reply)

    def recv(self, size):
        command, self.command = self.command, b''
        return command

    def close(self):
        if self.stop:
            self.stop.set()


class HeadCameraTests(unittest.TestCase):
    def test_old_frame_is_not_served_after_capture_stalls(self):
        hub = cameras.ViewHub()
        frame = np.zeros((8, 8, 3), dtype=np.uint8)
        with patch.object(cameras.time, 'monotonic', return_value=10):
            hub.push('head', frame)
        with patch.object(cameras.time, 'monotonic', return_value=10.5):
            self.assertIsNotNone(hub.latest('head')[0])
        with patch.object(cameras.time, 'monotonic', return_value=11.1):
            self.assertIsNone(hub.latest('head')[0])

    def test_shared_service_produces_decodable_head_jpeg(self):
        hub = cameras.ViewHub()
        hub.push('head', np.full((12, 16, 3), [240, 10, 20], dtype=np.uint8))
        stop = threading.Event()
        conn = FakeConnection(command=b'snapshot head\n', stop=stop)
        class Server:
            def settimeout(self, timeout): pass
            def accept(self): return conn, None
            def close(self): pass
        cameras._serve_view(Server(), hub, stop)
        header, jpeg = conn.sent.split(b'\n', 1)
        self.assertEqual(header, f'jpeg {len(jpeg)}'.encode())
        image = Image.open(io.BytesIO(jpeg))
        self.assertEqual(image.size, (16, 12))
        self.assertGreater(image.getpixel((0, 0))[0], 230)



if __name__ == '__main__':
    unittest.main()
