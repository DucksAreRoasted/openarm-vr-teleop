"""Vertical data must be paired and include a separate validation pose."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from lib.calibration_coverage import validation_mask, require_vertical_pairs
from test_runtime_gravity_fit import two_sided_fixture


class CalibrationCoverageTests(unittest.TestCase):
    def fixture(self):
        data, gravity, params = two_sided_fixture(np.full(7, 0.2))
        # Repurpose four shoulder poses without changing stage/sample counts.
        for old, new in zip((312, 313, 314, 315), (350, 351, 352, 353)):
            data["traj"][data["traj"] == old] = new
        return data, gravity, params

    def test_vertical_centre_is_held_out(self):
        traj = np.array([100, 110, 111, 200, 210, 211, 300, 310, 311, 350, 351, 352, 353])
        mask = validation_mask(traj)
        self.assertTrue(mask[traj == 351].all())
        self.assertFalse(mask[np.isin(traj, [350, 352, 353])].any())
        self.assertTrue(mask[traj == 310].all())

    def test_requires_both_approaches_on_both_arms(self):
        from lib.runtime_gravity_fit import fit_runtime
        data, gravity, params = self.fixture()
        require_vertical_pairs(data)
        fit_runtime(data, gravity, params)
        bad = (data["_side"] == "left") & (data["traj"] == 351) & (data["approach"] == -1)
        kept = ~bad
        data = {key: value[kept] for key, value in data.items()}
        with self.assertRaisesRegex(ValueError, "垂直附近.*351"):
            require_vertical_pairs(data)
        with self.assertRaisesRegex(ValueError, "垂直附近"):
            fit_runtime(data, gravity[kept], params)

    def test_explicit_single_approach_fit_remains_supported(self):
        from lib.runtime_gravity_fit import fit_runtime
        data, gravity, params = self.fixture()
        data["approach"][:] = 0
        _, reports = fit_runtime(data, gravity, params)
        self.assertTrue(all(row["friction_estimate"] is None for row in reports))


if __name__ == "__main__":
    unittest.main()
