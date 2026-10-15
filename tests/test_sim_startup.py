"""Simulation reaches the configured start pose before Quest becomes ready."""
import csv
import json
import os
from pathlib import Path
import queue
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "script"))
from replay_teaching import resolve_scene_xml


class StartupLauncherTests(unittest.TestCase):
    def test_launcher_passes_shared_startup_configuration(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ("assets", "build", "bin", "models/OpenArm_V1"):
                (root / name).mkdir(parents=True)
            shutil.copy(ROOT / "run.sh", root / "run.sh")
            for name in ("assets/v1.urdf", "assets/steady_action_on_table.csv",
                         "models/OpenArm_V1/scene_table.xml"):
                (root / name).touch()
            cmake = root / "bin/cmake"
            cmake.write_text("#!/bin/sh\nexit 0\n")
            cmake.chmod(0o755)
            control = root / "build/quest_cartesian_control"
            control.write_text("#!/usr/bin/python3\nimport json,sys\nprint(json.dumps(sys.argv[1:]))\n")
            control.chmod(0o755)
            env = {**os.environ, "PATH": f"{root / 'bin'}:{os.environ['PATH']}",
                   "OPENARM_RECORD_PANEL": "0", "OPENARM_QUEST_STARTUP_FILE":
                   str(root / "assets/steady_action_on_table.csv"), "OPENARM_QUEST_STARTUP_MOVE": "1",
                   "OPENARM_TELEOP_URDF": str(root / "assets/v1.urdf"),
                   "OPENARM_MUJOCO_XML": str(root / "models/OpenArm_V1/scene_table.xml")}
            for mode in ("--sim", "--real"):
                result = subprocess.run(["bash", str(root / "run.sh"), mode, "--no-bridge"],
                                        env=env, capture_output=True, text=True, check=True)
                args = json.loads(result.stdout.strip())
                self.assertIn("--bimanual", args)
                self.assertIn("--startup-move", args)
                self.assertIn("--startup-file", args)
                self.assertEqual("--no-sim-table-ready" in args, mode == "--sim")


@unittest.skipUnless((ROOT / "build/quest_cartesian_control").is_file(),
                     "build quest_cartesian_control first")
class SimStartupTests(unittest.TestCase):
    def launch(self, mode, startup_file="assets/steady_action_on_table.csv"):
        if hasattr(self, "process"):
            self.close()
        self.process = subprocess.Popen(
            [str(ROOT / "build/quest_cartesian_control"), "--sim", "--bimanual",
             "--xml", resolve_scene_xml(), "--startup-file", str(startup_file),
             "--startup-transition-sec", ".2", "--teleop-mode", mode,
             "--no-quest-ui", "--quest-port-left", "0", "--quest-port-right", "0"],
            cwd=ROOT, env={**os.environ, "OPENARM_MUJOCO_HEADLESS": "1",
                          "OPENARM_QUEST_UDP_HOST": "127.0.0.1",
                          "OPENARM_TELEOP_TRAJ_LOG": ""},
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.lines = queue.Queue()
        self.output = []
        def read():
            for line in self.process.stdout:
                self.lines.put(line)
        self.reader = threading.Thread(target=read, daemon=True)
        self.reader.start()
        self.addCleanup(self.close)

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self.reader.join(timeout=2)
        self.process.stdout.close()

    def until(self, text):
        end = time.monotonic()+12
        while time.monotonic() < end:
            try:
                line = self.lines.get(timeout=max(.001, end-time.monotonic()))
            except queue.Empty:
                break
            self.output.append(line)
            if text in line:
                return line
        self.fail(f"missing {text}: {''.join(self.output)}")

    def test_profiles_reach_last_frame_before_becoming_ready(self):
        with (ROOT / "assets/steady_action_on_table.csv").open() as handle:
            last = list(csv.DictReader(handle))[-1]
        for mode in ("unitree_style", "pika_style"):
            with self.subTest(mode=mode):
                self.launch(mode)
                self.until("phase=startup")
                self.assertNotIn("sim table-ready pose", "".join(self.output))
                self.until("bimanual running.")
                output = "".join(self.output)
                self.assertIn("target=full_trajectory", output)
                self.assertLess(output.index("startup dual-CSV complete"), output.index("bimanual running."))
                for side, offset in (("left", 0), ("right", 8)):
                    line = next(line for line in self.output if f"sim startup reached side={side}" in line)
                    actual = [float(value) for value in line.split(" q=")[1].split(',')]
                    expected = [float(last[f"q{i}"]) for i in range(offset, offset+8)]
                    expected[7] = (expected[7]+.98)/1.6*.044
                    for a, e in zip(actual, expected):
                        self.assertAlmostEqual(a, e, delta=.001)
                self.close()

    def test_stop_during_startup_does_not_enable_teleop(self):
        self.launch("unitree_style")
        self.until("phase=startup")
        self.process.terminate()
        self.assertEqual(self.process.wait(timeout=5), 0)
        self.reader.join(timeout=2)
        remaining = []
        while not self.lines.empty():
            remaining.append(self.lines.get_nowait())
        self.assertNotIn("bimanual running.", "".join(self.output+remaining))
        self.assertNotIn("startup move complete", "".join(self.output+remaining))
        self.assertNotIn("startup dual-CSV complete", "".join(self.output+remaining))

    def test_startup_visits_intermediate_file_poses_and_preserves_duration(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "startup.csv"
            with path.open("w", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(["time"] + [name for i in range(16) for name in (f"q{i}", f"dq{i}")])
                for t, shoulder in ((0, 0), (.3, .5), (.9, .5), (1.2, 0)):
                    row = [t]
                    for i in range(16):
                        q = -.98 if i%8 == 7 else .5 if i%8 == 3 else shoulder if i%8 == 0 else 0
                        row.extend((q, 0))
                    writer.writerow(row)
            launched = time.monotonic()
            self.launch("unitree_style", path)
            self.until("bimanual running.")
            elapsed = time.monotonic()-launched
            self.assertGreater(elapsed, 1.9)  # .2 transition + 1.2 source + .6 hold
            self.assertLess(elapsed, 3.5)
            samples = [line for line in self.output if "sim startup sample elapsed=" in line]
            shoulders = [float(line.split(" measured=")[1].split(',')[0]) for line in samples]
            self.assertGreater(max(shoulders), .45)  # Last-frame-only motion would stay at zero.
            line = next(line for line in self.output if "sim startup reached side=left" in line)
            self.assertAlmostEqual(float(line.split(" q=")[1].split(',')[0]), 0, delta=.001)
