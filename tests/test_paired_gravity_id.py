"""Friction-cancelled inertial identification on synthetic two-sided holds."""
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from lib.gravity_yaml_io import load_params, write_params
from lib.paired_gravity_id import (cad_theta, distal_masses, identify_side, regressors,
                                   runtime_theta, PairedPoses)
from lib.pinocchio_gravity_id import KP_HOLD, analytical_static_regressor
from lib.urdf_arm_gravity import load_arm_model

URDF = ROOT / 'assets/v1.urdf'
MIRROR = np.array([-1, -1, 1, 1, -1, -1, -1.0])


def true_theta(arm):
    theta = cad_theta(arm).copy()
    for i in range(7):
        m = arm.masses[i+1] * (1.0 + 0.12*np.sin(i + 1.0))
        com = arm.coms[i+1] + 0.012*np.array([np.cos(i), np.sin(2*i), np.cos(3*i)])
        theta[4*i], theta[4*i+1:4*i+4] = m, m*com
    return theta


def two_sided_log(params, friction, sides=('left', 'right')):
    rng = np.random.default_rng(3)
    lo = np.array([-1.4, -1.5, -0.6, 0.2, -0.8, -0.5, -0.7])
    hi = np.array([0.3, -0.1, 0.6, 1.9, 0.8, 0.5, 0.7])
    rows, t = [], 0.0
    for side in sides:
        arm = load_arm_model(str(URDF), side)
        theta = true_theta(arm)
        mirror = MIRROR if side == 'right' else np.ones(7)
        distal = distal_masses(params, side)
        for pose in range(27):
            stage = pose // 9 + 1
            goal = mirror * rng.uniform(lo, hi)
            for sign in (1, -1):
                q = goal + sign * mirror * 0.04  # a sticky joint stops near its release side
                y = analytical_static_regressor(arm, q)
                physical = y[:, :28] @ theta + sum(y[:, 24]*m + y[:, 25:28] @ (m*o) for m, o in distal)
                tau_g = y[:, :28] @ cad_theta(arm)
                err = (physical - sign*mirror*friction - tau_g) / KP_HOLD
                for _ in range(20):
                    t += 0.02
                    row = dict(t=t, traj=stage*100 + pose, approach=sign, ff_gain=0.92, _side=side)
                    for j in range(7):
                        for key, val in (('q_meas', q[j]), ('q_cmd', goal[j]), ('e', err[j]), ('dq', 0.0),
                                         ('dq_meas', 0.0), ('g_model', tau_g[j]), ('tau_g', tau_g[j])):
                            row[key + str(j+1)] = val
                    rows.append(row)
                t += 1.0
    data = {key: np.array([r[key] for r in rows]) for key in rows[0]}
    return data


class PairedIdentificationTests(unittest.TestCase):
    def setUp(self):
        self.params = load_params(Path('/nonexistent.yaml'))

    def prediction_error(self, result, side):
        arm = load_arm_model(str(URDF), side)
        rng = np.random.default_rng(11)
        mirror = MIRROR if side == 'right' else np.ones(7)
        q = mirror * rng.uniform([-1.2, -1.3, -0.5, 0.3, -0.6, -0.4, -0.6],
                                 [0.2, -0.2, 0.5, 1.8, 0.6, 0.4, 0.6], size=(40, 7))
        poses = PairedPoses(np.zeros(40, dtype=int), q, q, np.zeros((40, 7)))
        y, _ = regressors(arm, poses, [])
        fitted = runtime_theta(arm, result.mass_scale, result.com_delta)
        return (np.sqrt(np.mean((np.einsum('njk,k->nj', y, fitted - true_theta(arm)))**2, axis=0)),
                np.sqrt(np.mean((np.einsum('njk,k->nj', y, cad_theta(arm) - true_theta(arm)))**2, axis=0)))

    def test_recovers_link_gravity_despite_static_friction(self):
        friction = np.array([0.3, 0.3, 0.15, 0.2, 0.1, 0.06, 0.08])
        data = two_sided_log(self.params, friction)
        for side in ('left', 'right'):
            result = identify_side(data, side, URDF, self.params)
            fitted, cad = self.prediction_error(result, side)
            self.assertLess(np.max(fitted[:4]), 0.35 * np.max(cad[:4]), side)
            self.assertLess(np.max(result.validation_rms), 0.05, side)
            self.assertEqual(result.clipped, [])

    def test_one_sided_log_is_rejected(self):
        data = two_sided_log(self.params, np.zeros(7), sides=('left',))
        data = {k: v[data['approach'] > 0] if isinstance(v, np.ndarray) else v for k, v in data.items()}
        with self.assertRaisesRegex(ValueError, '两侧都停稳'):
            identify_side(data, 'left', URDF, self.params)


class YamlSharedMassScaleTests(unittest.TestCase):
    def test_shared_mass_scale_is_removed_and_not_reinjected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'g.yaml'
            write_params(path, {'link_mass_scale': [1.0]*7, 'left_link_mass_scale': [1.1]*7})
            self.assertIn('link_mass_scale', load_params(path))
            write_params(path, {'left_link_mass_scale': [1.2]*7}, remove=('link_mass_scale',))
            self.assertNotIn('link_mass_scale', load_params(path))
            write_params(path, {'left_gravity_bias': [0.1]*7})
            keys = [line.strip().split(':')[0] for line in path.read_text().splitlines()]
            self.assertNotIn('link_mass_scale', keys)
            self.assertEqual(load_params(path)['left_link_mass_scale'], [1.2]*7)


if __name__ == '__main__':
    unittest.main()
