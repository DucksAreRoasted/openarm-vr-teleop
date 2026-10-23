"""Unitree xr_teleoperate pieces used by OpenArm: basis, filter, and IPOPT IK."""

from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "script" / "unitree_openarm"))

from weighted_moving_filter import WeightedMovingFilter  # noqa: E402
from xr_basis import R_ROBOT_OPENXR, openxr_pose_to_robot  # noqa: E402


class UnitreeBasisAndFilterTests(unittest.TestCase):
    def test_openxr_up_is_robot_up(self):
        self.assertTrue(np.allclose(R_ROBOT_OPENXR @ np.array([0.0, 1.0, 0.0]), [0.0, 0.0, 1.0]))
        self.assertTrue(np.allclose(R_ROBOT_OPENXR @ np.array([0.0, 0.0, -1.0]), [1.0, 0.0, 0.0]))
        pose = np.eye(4)
        pose[:3, 3] = [0.0, 0.2, -0.1]
        mapped = openxr_pose_to_robot(pose)
        self.assertAlmostEqual(mapped[0, 3], 0.1)
        self.assertAlmostEqual(mapped[2, 3], 0.2)

    def test_weighted_filter_matches_unitree_weights(self):
        filt = WeightedMovingFilter(np.array([0.4, 0.3, 0.2, 0.1]), 2)
        for value in (1.0, 2.0, 3.0, 4.0):
            filt.add_data(np.array([value, -value]))
        expected = 0.4 * 4 + 0.3 * 3 + 0.2 * 2 + 0.1 * 1
        self.assertAlmostEqual(filt.filtered_data[0], expected)
        self.assertAlmostEqual(filt.filtered_data[1], -expected)


class OpenArmIpoptTests(unittest.TestCase):
    def test_solve_returns_near_the_seed_pose(self):
        try:
            import pinocchio as pin
            from openarm_arm_ik import OpenArmArmIK
        except ImportError as error:
            self.skipTest(str(error))
        ik = OpenArmArmIK(str(ROOT / "assets" / "v1.urdf"), "left")
        seed = np.array([0.2, -0.5, 0.1, 1.3, -0.4, 0.2, -0.3])
        pin.forwardKinematics(ik.model, ik.data, seed)
        pin.updateFramePlacements(ik.model, ik.data)
        target = np.eye(4)
        target[:3, :3] = ik.data.oMf[ik.frame_id].rotation
        target[:3, 3] = ik.data.oMf[ik.frame_id].translation
        solved = ik.solve(target, seed)
        self.assertIsNotNone(solved)
        pin.forwardKinematics(ik.model, ik.data, solved)
        pin.updateFramePlacements(ik.model, ik.data)
        position_error = np.linalg.norm(ik.data.oMf[ik.frame_id].translation - target[:3, 3])
        self.assertLess(position_error, 0.01)


if __name__ == "__main__":
    unittest.main()
