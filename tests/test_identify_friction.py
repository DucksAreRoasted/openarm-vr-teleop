"""Offline friction fit checks; never initialize CAN hardware."""
import csv
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import identify_friction  # noqa: E402
from lib.gravity_yaml_io import load_params  # noqa: E402

FC = 0.42
FV = 0.03


def write_sweep(path: Path, *, gravity_hysteresis: float = 0.0, seed: int = 0,
                joint: int = 0, fc: float = FC, fv: float = FV, right_scale: float = 1.0,
                plateau: bool = False) -> None:
    rng = np.random.default_rng(seed)
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t", "side", "joint", "speed", "dir", "q_cmd", "q", "dq_cmd", "dq",
                         "tau_g", "tau_cmd", "mtau"])
        joints = (joint,) if joint else range(1, 8)
        for side in ("left", "right"):
            scale = right_scale if side == "right" else 1.0
            for joint_index in joints:
                for speed in (0.05, 0.15, 0.30):
                    for direction in (1, -1):
                        for q in np.linspace(0.9, 1.8, 300):
                            gravity = 1.5 * np.sin(q)
                            friction = scale * fc if plateau else scale * (fc * np.tanh(8 * speed) + fv * speed)
                            # Position-dependent extra torque only on one direction
                            # mimics a gravity error that does not cancel.
                            extra = gravity_hysteresis * np.sin(8 * q) if direction > 0 else 0.0
                            tau = gravity + direction * friction + extra + rng.normal(0, 0.01)
                            writer.writerow([0, side, joint_index, speed, direction, q, q, direction * speed,
                                             direction * speed, gravity, tau, tau])


class IdentifyFrictionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.dir = Path(self.temp.name)
        self.yaml = self.dir / "gravity_comp.yaml"
        shutil.copy(ROOT / "config" / "gravity_comp.yaml", self.yaml)

    def tearDown(self):
        self.temp.cleanup()

    def test_recovers_derated_friction_and_keeps_other_keys(self):
        log = self.dir / "friction_1.csv"
        write_sweep(log)
        before = load_params(self.yaml)
        self.assertEqual(identify_friction.main([str(log), "--write", str(self.yaml)]), 0)
        after = load_params(self.yaml)
        # J4 cap is 0.8 / 0.05; the derated values stay below it.
        self.assertAlmostEqual(after["left_friction_fc"][3], 0.85 * FC, delta=0.02)
        self.assertAlmostEqual(after["right_friction_fv"][3], 0.85 * FV, delta=0.006)
        self.assertEqual(after["left_friction_k"][3], 8.0)
        self.assertEqual(after["right_friction_k"][0], 8.0)
        # Wrist cap is 0.35; the same 0.42 Nm fit is rejected and the old value stays.
        self.assertEqual(after["left_friction_fc"][6], before["left_friction_fc"][6])
        self.assertEqual(after["left_friction_k"][6], before["left_friction_k"][6])
        for key in ("left_gravity_scale", "left_link_mass_scale", "g_scale"):
            self.assertEqual(after[key], before[key])

    def test_dry_run_leaves_file_untouched(self):
        log = self.dir / "friction_1.csv"
        write_sweep(log)
        original = self.yaml.read_bytes()
        self.assertEqual(identify_friction.main([str(log), "--write", str(self.yaml), "--dry-run"]), 0)
        self.assertEqual(self.yaml.read_bytes(), original)

    def test_position_dependent_gravity_error_is_rejected(self):
        log = self.dir / "friction_1.csv"
        write_sweep(log, gravity_hysteresis=0.4)
        original = self.yaml.read_bytes()
        self.assertEqual(identify_friction.main([str(log), "--write", str(self.yaml)]), 0)
        self.assertEqual(self.yaml.read_bytes(), original)

    def test_gripper_writes_the_smaller_side_and_leaves_arm_friction(self):
        log = self.dir / "friction_grip.csv"
        write_sweep(log, joint=8, fc=0.10, fv=0.0, right_scale=1.4, plateau=True)
        before = load_params(self.yaml)
        self.assertEqual(identify_friction.main([str(log), "--write", str(self.yaml)]), 0)
        after = load_params(self.yaml)
        self.assertAlmostEqual(after["gripper_friction_fc"], 0.85 * 0.10, delta=0.01)
        self.assertLess(after["gripper_friction_fv"], 0.01)
        self.assertEqual(after["gripper_friction_k"], 8.0)
        self.assertEqual(after["left_friction_fc"], before["left_friction_fc"])
        self.assertEqual(after["right_friction_k"], before["right_friction_k"])

    def test_rejects_non_friction_log(self):
        log = self.dir / "friction_bad.csv"
        log.write_text("t,side,traj\n0,left,100\n")
        self.assertEqual(identify_friction.main([str(log), "--write", str(self.yaml), "--dry-run"]), 1)


if __name__ == "__main__":
    unittest.main()
