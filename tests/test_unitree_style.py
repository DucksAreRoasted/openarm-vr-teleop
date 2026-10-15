"""Real OpenArm mapping/IK regression for the default Quest preset."""
import os
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]


class UnitreeStyleTests(unittest.TestCase):
    def test_mapping_and_bounded_ik(self):
        binary = ROOT / 'build/unitree_style_test'
        if not binary.is_file():
            if os.environ.get('CI'):
                self.fail('Build unitree_style_test before running tests')
            self.skipTest('Build unitree_style_test before running tests')
        result = subprocess.run([str(binary), str(ROOT / 'assets/v1.urdf')],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('unitree_style_test: ok', result.stdout)
