"""Check calibration progress forwarding without opening CAN or moving arms."""
import io
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import calibrate_gravity


class CalibrationOutputTests(unittest.TestCase):
    def test_progress_arrives_before_child_exit_and_output_is_retained(self):
        with tempfile.TemporaryDirectory() as directory:
            acknowledgement = Path(directory) / "progress_received"

            class ProgressSink(io.StringIO):
                def write(self, text):
                    if text == "采数进行中\n":
                        acknowledgement.touch()
                    return super().write(text)

            child = """
import os, pathlib, sys, time
assert os.environ['PYTHONUNBUFFERED'] == '1'
print('采数进行中', flush=True)
deadline = time.monotonic() + 5
while not pathlib.Path(sys.argv[1]).exists():
    if time.monotonic() >= deadline:
        sys.exit(23)
    time.sleep(0.01)
print('诊断输出', file=sys.stderr, flush=True)
sys.stdout.write('LOG_PATH=/tmp/disposable.csv')
sys.exit(7)
"""
            sink = ProgressSink()
            with patch.object(calibrate_gravity.sys, "stdout", sink):
                result = calibrate_gravity.run(
                    [sys.executable, "-c", child, str(acknowledgement)]
                )
            expected = "采数进行中\n诊断输出\nLOG_PATH=/tmp/disposable.csv"
            self.assertEqual(result.returncode, 7)
            self.assertEqual(result.stdout, expected)
            self.assertTrue(sink.getvalue().endswith(expected))
            # Ignore the command preview, which contains the child's source text.
            self.assertEqual(sink.getvalue().splitlines()[-3:], expected.splitlines())


if __name__ == "__main__":
    unittest.main()
