"""Bounded J6 spring dynamics and frozen-integral lifecycle; no CAN."""
from pathlib import Path
import ctypes
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "ros2/src/openarm_moveit_config"


class ImpedanceHoldTests(unittest.TestCase):
    def test_hold_and_disturbance_simulations(self):
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / "impedance_hold_test"
            compiled = subprocess.run(
                ["c++", "-std=c++17", "-Wall", "-Wextra", "-Wpedantic",
                 "-I", str(PACKAGE / "scripts"), "-I", str(ROOT / "src"),
                 str(ROOT / "tests/impedance_hold_test.cpp"), "-o", str(binary)],
                capture_output=True, text=True,
            )
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            result = subprocess.run(
                [str(binary), str(PACKAGE / "config/impedance_profiles.yaml")],
                capture_output=True, text=True,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_mujoco_sustained_push_release_and_noisy_hold(self):
        try:
            import mujoco
        except ImportError:
            self.skipTest("MuJoCo unavailable")
        import numpy as np
        import yaml

        profiles = yaml.safe_load((PACKAGE / "config/impedance_profiles.yaml").read_text())["profiles"]
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            wrapper = directory / "spring.cpp"
            wrapper.write_text(
                '#include "impedance_hold.hpp"\n'
                'extern "C" double spring(double e, double c, double w) {\n'
                'return nonlinear_spring_torque(e, c, w);}\n'
            )
            library = directory / "spring.so"
            subprocess.run(
                ["c++", "-std=c++17", "-shared", "-fPIC", "-I", str(PACKAGE / "scripts"),
                 str(wrapper), "-o", str(library)], check=True, capture_output=True,
            )
            bridge = ctypes.CDLL(str(library))
            bridge.spring.argtypes = [ctypes.c_double] * 3
            bridge.spring.restype = ctypes.c_double
            for name in ("P2", "P3"):
                profile = profiles[name]
                for inertia in (0.002, 0.04):
                    deflections = []
                    for enabled in (False, True):
                        with self.subTest(profile=name, inertia=inertia, enabled=enabled):
                            kp = profile["kp"][5]
                            kd = profile["kd"][5] + (profile["restore_kd"][5] if enabled else 0)
                            cap = profile["restore_tau_max"][5] if enabled else 0
                            xml = f'''<mujoco><option timestep=".001" gravity="0 0 0"/>
                            <worldbody><body><joint name="j6" type="hinge" axis="0 0 1" frictionloss=".12"/>
                            <inertial pos="0 0 0" mass="1" diaginertia="{inertia} {inertia} {inertia}"/>
                            </body></worldbody><actuator><general joint="j6" gainprm="{kp}"
                            biasprm="0 -{kp} -{kd}" biastype="affine"/></actuator></mujoco>'''
                            model = mujoco.MjModel.from_xml_string(xml)
                            data = mujoco.MjData(model)
                            extra = 0.0
                            tail = []
                            for tick in range(60000):
                                if tick == 3000:
                                    data.qpos[0] = 0.02
                                    data.qvel[0] = 0
                                if tick % 10 == 0:
                                    # Assumed small sensor noise, not a measured motor noise model.
                                    observed = float(data.qpos[0]) + 0.0002 * np.sin(tick * 0.17)
                                    extra = bridge.spring(-observed, cap, profile["restore_width"])
                                    self.assertLessEqual(abs(extra), cap + 1e-12)
                                # Sustained external push, then release without a target change.
                                data.qfrc_applied[0] = extra + (0.5 if 15000 <= tick < 25000 else 0)
                                mujoco.mj_step(model, data)
                                if tick == 24999:
                                    deflections.append(abs(float(data.qpos[0])))
                                if tick >= 50000:
                                    tail.append(float(data.qpos[0]))
                            self.assertTrue(np.isfinite(data.qpos).all())
                            if enabled:
                                self.assertLess(float(np.ptp(tail)), 0.001)
                                self.assertLess(abs(float(np.mean(tail))), 0.002)
                    self.assertLess(deflections[1], deflections[0] * 0.85)


if __name__ == "__main__":
    unittest.main()
