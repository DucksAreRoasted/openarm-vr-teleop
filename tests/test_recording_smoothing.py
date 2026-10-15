"""Offline derivatives reduce jitter without rewriting measurements or timing."""
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "script"))
from smooth_recording import smooth_positions, smooth_recording


def recording(folder, *, torques=True):
    t = np.linspace(0, 2, 2001)
    q = .15*np.sin(2*np.pi*.5*t) + .025*np.sin(2*np.pi*18*t)
    path = folder / "real_teleop_dual_merged.csv"
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        names = (lambda i: (f"q{i}", f"dq{i}", f"tau{i}")) if torques else (lambda i: (f"q{i}", f"dq{i}"))
        writer.writerow(["time"] + [n for i in range(16) for n in names(i)])
        for instant, angle in zip(t, q):
            row = [f"{instant:.6f}"]
            for i in range(16):
                position = (0 if instant < 1 else .04) if i%8 == 7 else (1.57 if i%8 == 3 else angle)
                row.extend((f"{position:.9f}", "0.000000000"))
                if torques: row.append("0.123456789")
            writer.writerow(row)
    with (folder / "lerobot_episode.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time"] + [f"{s}_{kind}{i}" for s in ("l", "r") for kind in ("obs", "act") for i in range(8)] + ["unix"])
        for instant in t[::20]:
            writer.writerow([format(instant, '.6f')] + [format((.3 if kind == 'obs' else float(np.interp(instant,t,q))) if i != 7 else .02, '.9f') for s in ('l','r') for kind in ('obs','act') for i in range(8)] + [str(1000+instant)])
    (folder / "collection_mode.json").write_text('{"gripper_units":"meters"}')
    return path


class RecordingSmoothingTests(unittest.TestCase):
    def test_noise_attenuation_low_frequency_shape_endpoints_and_deviation(self):
        t = np.linspace(0, 5, 5001)
        clean = .3*np.sin(2*np.pi*.5*t)
        noisy = clean + .025*np.sin(2*np.pi*18*t)
        q, dq, stats = smooth_positions(t, noisy[:, None], 120, 3)
        middle = (t>.3)&(t<4.7)
        self.assertLess(np.std(q[middle,0]-clean[middle]), .2*np.std(noisy[middle]-clean[middle]))
        np.testing.assert_array_equal(q[[0,-1],0], noisy[[0,-1]])
        self.assertLessEqual(stats['max_deviation_deg'][0], 3+1e-9)
        self.assertLess(stats['velocity_rms_after_rad_s'], stats['velocity_rms_before_rad_s']*.4)
        # A symmetric filter keeps the peak of a clean slow gesture in place.
        smooth_clean, _, _ = smooth_positions(t, clean[:,None],120,3)
        self.assertEqual(np.argmax(smooth_clean[:1000,0]),np.argmax(clean[:1000]))
        np.testing.assert_allclose(dq, np.gradient(q,t,axis=0,edge_order=2))
        large = 2*noisy[:,None]
        limited, _, st = smooth_positions(t,large,200,.1)
        self.assertLessEqual(st['max_deviation_deg'][0],.1+1e-8)
        self.assertGreaterEqual(limited.min(), large.min()-1e-12)
        self.assertLessEqual(limited.max(), large.max()+1e-12)

    def test_irregular_sampling_has_same_time_domain_effect(self):
        rng=np.random.default_rng(7)
        t=np.r_[0,np.cumsum(rng.uniform(.0005,.0015,3000))]
        clean=.2*np.sin(2*np.pi*.5*t)
        noisy=clean+.02*np.sin(2*np.pi*20*t)
        result,_,_=smooth_positions(t,noisy[:,None],120,3)
        mid=(t>.3)&(t<t[-1]-.3)
        self.assertLess(np.std(result[mid,0]-clean[mid]),.003)

    def test_derivatives_preserve_raw_state_gripper_torque_and_metadata(self):
        for torques in (False,True):
            with self.subTest(torques=torques), tempfile.TemporaryDirectory() as directory:
                folder=Path(directory); path=recording(folder,torques=torques)
                original={p.name:p.read_bytes() for p in folder.iterdir()}
                report=smooth_recording(folder)
                for name, data in original.items(): self.assertEqual((folder/name).read_bytes(),data)
                with path.open() as f: before=list(csv.DictReader(f))
                with Path(report['trajectory']).open() as f: after=list(csv.DictReader(f))
                self.assertEqual(len(before),len(after))
                for raw,smooth in zip(before,after):
                    for key in ('time','q7','dq7','q15','dq15'):
                        self.assertEqual(raw[key],smooth[key])
                    if torques:
                        for i in range(16): self.assertEqual(raw[f'tau{i}'],smooth[f'tau{i}'])
                for key in [f'q{i}' for i in range(16)]:
                    self.assertEqual(float(before[0][key]),float(after[0][key]))
                    self.assertEqual(float(before[-1][key]),float(after[-1][key]))
                with (folder/'lerobot_episode.csv').open() as f: before_actions=list(csv.DictReader(f))
                with Path(report['training_actions']).open() as f: after_actions=list(csv.DictReader(f))
                for raw,smooth in zip(before_actions,after_actions):
                    for key in raw:
                        if '_act' not in key or key.endswith('act7'): self.assertEqual(raw[key],smooth[key])
                self.assertEqual(json.loads((Path(report['output_dir'])/'collection_mode.json').read_text())['gripper_units'],'meters')
                again=smooth_recording(folder)
                self.assertNotEqual(again['output_dir'],report['output_dir'])
                binary=ROOT/'build/openarm_teaching_replay'
                if binary.is_file():
                    check=subprocess.run([str(binary),'--csv',report['trajectory'],'--source-meters','--check'],capture_output=True,text=True)
                    self.assertEqual(check.returncode,0,check.stdout+check.stderr)

    def test_invalid_input_creates_no_derivative(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);path=recording(folder)
            for bad in (0, float('nan'), 501):
                with self.assertRaises(ValueError): smooth_recording(folder,window_ms=bad)
            with self.assertRaises(ValueError): smooth_recording(folder,max_deviation_deg=0)
            text=path.read_text();path.write_text(text.replace('0.001000,','0.000000,',1))
            with self.assertRaises(ValueError): smooth_recording(folder)
            self.assertFalse((folder/'processed').exists())
        with self.assertRaisesRegex(ValueError,'间断'):
            smooth_positions(np.array([0,.001,.002,.5]),np.zeros((4,1)),120,3)

    @unittest.skipUnless((ROOT/'build/openarm_teaching_replay').is_file(),'build replay first')
    def test_smoothed_copy_plays_to_completion_in_simulation(self):
        import time
        from replay_teaching import resolve_scene_xml
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);recording(folder)
            report=smooth_recording(folder)
            log=folder/'preview.log'
            with log.open('w') as output:
                process=subprocess.Popen([str(ROOT/'build/openarm_teaching_replay'), '--sim',
                    '--csv',report['trajectory'],'--source-meters','--xml',resolve_scene_xml()],
                    env={**os.environ,'OPENARM_MUJOCO_HEADLESS':'1'},stdin=subprocess.PIPE,
                    stdout=output,stderr=subprocess.STDOUT,text=True)
                try:
                    deadline=time.monotonic()+10
                    while '[replay] COMPLETE' not in log.read_text():
                        self.assertIsNone(process.poll(),log.read_text())
                        self.assertLess(time.monotonic(),deadline,log.read_text())
                        time.sleep(.02)
                    process.stdin.write('stop\n');process.stdin.flush()
                    self.assertEqual(process.wait(timeout=3),0)
                finally:
                    if process.poll() is None:
                        process.terminate();process.wait(timeout=3)
                    process.stdin.close()

    def test_original_label_and_export_survive_smoothing(self):
        from unittest.mock import patch
        from openarm_app import dataset_manager as dm
        from openarm_dataset_export import discover_labeled_episodes, write_openarm_dataset
        import pandas as pd
        with tempfile.TemporaryDirectory() as directory:
            batch=Path(directory)/'batch';episode=batch/'01';episode.mkdir(parents=True)
            recording(episode)
            with patch.object(dm,'SESSIONS_ROOT',Path(directory)):
                dm.set_outcome('batch','01','success',export=False)
            report=smooth_recording(episode)
            self.assertEqual((episode/'outcome.txt').read_text().strip(),'success')
            episodes=discover_labeled_episodes(batch,1.)
            self.assertEqual(len(episodes),1)
            out=Path(directory)/'export'
            self.assertEqual(write_openarm_dataset(batch,out,'test','operator','lab',1.),out)
            exported=pd.read_parquet(out/'episodes/1/obs/arms/left/state.parquet')
            np.testing.assert_allclose(np.stack(exported.qpos.to_numpy())[:,:7],.3)
            action=pd.read_parquet(out/'episodes/1/action/arms/left/state.parquet')
            np.testing.assert_allclose(np.stack(action.qpos.to_numpy()),episodes[0]['left_act'])
            self.assertTrue(Path(report['training_actions']).is_file())

    def test_standalone_csv_needs_no_training_data(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);path=recording(folder)
            (folder/'lerobot_episode.csv').unlink()
            report=smooth_recording(path)
            self.assertNotIn('training_actions',report)
            self.assertTrue(Path(report['trajectory']).is_file())


os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
try:
    from PySide6.QtCore import QProcess
    from PySide6.QtWidgets import QApplication
    from openarm_app.trajectory_smooth_dialog import TrajectorySmoothDialog
    QT_AVAILABLE=True
except ImportError:
    QT_AVAILABLE=False


@unittest.skipUnless(QT_AVAILABLE,'Qt required')
class SmoothingDialogTests(unittest.TestCase):
    def test_data_manager_blocks_live_recordings_and_opens_smoothing(self):
        from unittest.mock import patch
        from openarm_app.data_manage_dialog import DataManageDialog
        from openarm_app.dataset_manager import EpisodeInfo
        app=QApplication.instance() or QApplication([])
        ep=EpisodeInfo('01',Path('/tmp/disposable/01'),'success',100,1.,0,False)
        with patch('openarm_app.data_manage_dialog.dm.list_batches',return_value=[]):
            dialog=DataManageDialog(current_batch='disposable')
        with patch.object(dialog,'_selected_batch_name',return_value='disposable'), \
             patch.object(dialog,'_selected_episode',return_value=ep):
            dialog.set_teleop_state(teleop_running=True,recording_busy=True)
            self.assertFalse(dialog.btn_smooth.isEnabled())
            self.assertFalse(dialog.btn_replay.isEnabled())
            dialog.set_teleop_state(teleop_running=False)
            self.assertTrue(dialog.btn_smooth.isEnabled())
            self.assertTrue(dialog.btn_export.isEnabled())
            with patch('openarm_app.data_manage_dialog.TrajectorySmoothDialog') as factory:
                dialog.btn_smooth.click()
                factory.assert_called_once_with(ep.path,dialog)
                factory.return_value.exec.assert_called_once()
        dialog.close()
        app.processEvents()

    def test_generate_then_replay_and_failed_input(self):
        import time
        app=QApplication.instance() or QApplication([])
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);recording(folder)
            dialog=TrajectorySmoothDialog(folder)
            sources=[];dialog.replay_requested.connect(sources.append)
            dialog.generate.click()
            self.assertFalse(dialog.generate.isEnabled())
            deadline=time.monotonic()+10
            while dialog.process.state()!=QProcess.ProcessState.NotRunning or not dialog.generate.isEnabled():
                app.processEvents()
                self.assertLess(time.monotonic(),deadline)
                time.sleep(.01)
            self.assertIsNotNone(dialog.report,dialog.status.text())
            self.assertTrue(dialog.replay.isEnabled())
            output=dialog.report['output_dir']
            dialog.replay.click()
            self.assertEqual(sources,[output])
            broken=TrajectorySmoothDialog(folder/'missing.csv')
            broken.generate.click()
            deadline=time.monotonic()+10
            while broken.process.state()!=QProcess.ProcessState.NotRunning or not broken.generate.isEnabled():
                app.processEvents(); self.assertLess(time.monotonic(),deadline);time.sleep(.01)
            self.assertFalse(broken.replay.isEnabled())
            self.assertIn('平滑失败',broken.status.text())
            broken.close()
