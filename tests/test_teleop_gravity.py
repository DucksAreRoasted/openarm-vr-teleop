"""Production control feedforward, tested without motors or CAN sockets."""
import os
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]


class TeleopGravityTests(unittest.TestCase):
    def test_mit_feedforward_and_fault_stop(self):
        binary = ROOT / 'build/teleop_gravity_test'
        if not binary.is_file():
            if os.environ.get('CI'):
                self.fail('Build teleop_gravity_test before running tests')
            self.skipTest('Build teleop_gravity_test before running tests')
        result = subprocess.run([str(binary), str(ROOT / 'assets/v1.urdf')],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('teleop_gravity_test: ok', result.stdout)
