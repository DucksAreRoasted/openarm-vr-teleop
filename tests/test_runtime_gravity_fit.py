"""Offline calibration checks; never initialize CAN hardware."""
import copy
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from lib.runtime_gravity_fit import fit_runtime, runtime_gravity
from lib.gravity_yaml_io import load_params
from lib.pinocchio_gravity_id import KP_HOLD
import calibrate_gravity


def fixture():
    params = load_params(Path(__file__).resolve().parents[1] / 'config/gravity_comp.yaml')
    rows = []
    gravity = []
    gain = params['g_scale']
    for side in ('left', 'right'):
        for group in range(18):
            stage = group // 6 + 1
            q = np.full(7, group * 0.04)
            g = np.sin(group * 0.6 + np.arange(7)) * 2.0
            scale = np.array(params[side+'_gravity_scale'])
            bias = np.array(params[side+'_gravity_bias'])
            tau_g = gain * (scale*g + bias)
            desired = gain * ((scale+0.1)*g + bias+0.03)
            err = (desired-tau_g)/KP_HOLD
            for sample in range(25):
                row = dict(t=group+sample*0.02, traj=stage*100+group,
                           ff_gain=gain, _side=side)
                for j in range(7):
                    for key, val in [('q_meas',q[j]), ('q_cmd',q[j]+err[j]),
                                     ('e',err[j]), ('dq',0), ('dq_meas',0),
                                     ('g_model',g[j]), ('tau_g',tau_g[j])]:
                        row[key+str(j+1)] = val
                rows.append(row); gravity.append(g)
    data = {key: np.array([r[key] for r in rows]) for key in rows[0]}
    return data, np.array(gravity), params


class RuntimeFitTests(unittest.TestCase):
    def test_recovers_corrections_without_double_derating(self):
        data, gravity, params = fixture()
        original = copy.deepcopy(params)
        updates, report = fit_runtime(data, gravity, params)
        self.assertTrue(all(row['accepted'] for row in report))
        self.assertEqual(params, original)
        self.assertEqual(set(updates), {'left_gravity_scale','right_gravity_scale',
                                       'left_gravity_bias','right_gravity_bias'})
        for side in ('left','right'):
            np.testing.assert_allclose(updates[side+'_gravity_scale'],
                                       np.array(params[side+'_gravity_scale'])+0.1, atol=.02)

    def test_rejects_missing_stages_and_legacy_logs(self):
        data, gravity, params = fixture()
        with self.assertRaisesRegex(ValueError, 'ff_gain'):
            fit_runtime({k:v for k,v in data.items() if k != 'ff_gain'}, gravity, params)
        data['traj'][:] = 100
        with self.assertRaisesRegex(ValueError, '完整'):
            fit_runtime(data, gravity, params)

    def test_rejects_changed_physical_model(self):
        data, gravity, params = fixture()
        with self.assertRaisesRegex(ValueError, '物理模型'):
            fit_runtime(data, gravity+0.1, params)

    def test_keeps_unexcited_joint(self):
        data, gravity, params = fixture()
        gravity[:,0] = 0; data['g_model1'][:] = 0
        updates, report = fit_runtime(data, gravity, params)
        for side in ('left','right'):
            self.assertEqual(updates[side+'_gravity_scale'][0],params[side+'_gravity_scale'][0])
            self.assertEqual(updates[side+'_gravity_bias'][0],params[side+'_gravity_bias'][0])
        self.assertFalse(report[0]['accepted'])

    def test_no_change_when_existing_model_is_correct(self):
        data, gravity, params = fixture()
        for j in range(1,8):
            data[f'e{j}'][:] = 0
        updates, report = fit_runtime(data, gravity, params)
        self.assertFalse(any(row['accepted'] for row in report))
        self.assertEqual(updates['left_gravity_scale'],params['left_gravity_scale'])

    def test_rejects_candidate_that_worsens_held_out_poses(self):
        data, gravity, params = fixture()
        for stage in (1,2,3):
            for offset in (1,4):
                group = (stage-1)*6+offset
                selected = data['traj'] == stage*100+group
                for j in range(1,8):
                    data[f'e{j}'][selected] *= -1
        updates, report = fit_runtime(data, gravity, params)
        self.assertFalse(any(row['accepted'] for row in report))
        self.assertEqual(updates['right_gravity_scale'],params['right_gravity_scale'])

    def test_rejects_saturated_samples(self):
        data, gravity, params = fixture()
        for j in range(1,8):
            data[f'tau_ff{j}'] = np.zeros(len(gravity))
        data['tau_ff1'][:] = 20.0
        with self.assertRaisesRegex(ValueError, '完整'):
            fit_runtime(data, gravity, params)

    def test_abort_does_not_seed_or_overwrite_working_file(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory)/'gravity.yaml'
            original = b'working configuration\n'
            target.write_bytes(original)
            args = SimpleNamespace(write=target, dry_run=False, skip_motion=False,
                                   seed=True, g_scale=None, speed_max=.05, ee_payload=None, urdf_scale=None)
            with patch.object(calibrate_gravity, 'run', return_value=
                              subprocess.CompletedProcess([],130,'aborted')):
                self.assertEqual(calibrate_gravity.calibrate_runtime(args,{}),130)
            self.assertEqual(target.read_bytes(), original)
            self.assertEqual(next(Path(directory).glob('*.backup*')).read_bytes(),original)


if __name__ == '__main__':
    unittest.main()
