"""Headless timing checks against the built replay executable (no CAN access)."""
import csv
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "script"))
from replay_teaching import resolve_scene_xml

BINARY = ROOT / "build/openarm_teaching_replay"


@unittest.skipUnless(BINARY.is_file(), "build openarm_teaching_replay first")
class ReplayPreviewTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="replay-preview-test-")
        self.addCleanup(self.temporary.cleanup)

    def start(self, duration=1, speed=1, repeat=1, initial_joint=0, wait_for_playing=True,
              manual_start=False, startup=None, transition_sec=.2):
        path = Path(self.temporary.name) / "trajectory.csv"
        with path.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["time"] + [name for i in range(16) for name in (f"q{i}", f"dq{i}")])
            # Fast motion and gripper changes used to trigger servo slowdowns.
            for instant in (0, duration):
                row = [instant]
                for i in range(16):
                    q = (0.044 if instant else 0) if i % 8 == 7 else (
                        1.57 if i % 8 == 3 else (
                            (1.0 if instant else initial_joint) if i % 8 == 0 else 0))
                    row.extend((q, 0))
                writer.writerow(row)
        self.process = subprocess.Popen(
            [str(BINARY), "--sim", "--csv", str(path), "--source-meters",
             "--xml", resolve_scene_xml(), "--speed", str(speed), "--repeat", str(repeat),
             "--gravity-yaml", "/does-not-exist-preview-needs-no-gravity.yaml"] +
            (["--wait-for-start"] if manual_start else []) +
            (["--startup-file", str(startup), "--startup-transition-sec", str(transition_sec)] if startup else []),
            cwd=ROOT, env={**os.environ, "OPENARM_MUJOCO_HEADLESS": "1"},
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.addCleanup(self.close)
        self.lines = queue.Queue()
        def read():
            for line in self.process.stdout:
                self.lines.put((time.monotonic(), line))
        self.reader = threading.Thread(target=read, daemon=True)
        self.reader.start()
        event = "[replay] READY" if manual_start and wait_for_playing else (
            "[replay] PLAYING" if wait_for_playing else "[replay] ALIGNING")
        return self.wait_for(event)[0]

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGCONT)
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self.reader.join(timeout=2)
        self.process.stdin.close()
        self.process.stdout.close()

    def wait_for(self, text, timeout=8):
        deadline = time.monotonic() + timeout
        seen = []
        while time.monotonic() < deadline:
            try:
                stamp, line = self.lines.get(timeout=max(.001, deadline-time.monotonic()))
            except queue.Empty:
                break
            seen.append(line)
            if text in line:
                return stamp, line
        self.fail(f"Missing {text!r}; output: {''.join(seen)}")

    def test_preparation_follows_middle_of_startup_csv_before_ready(self):
        startup = Path(self.temporary.name) / "startup.csv"
        with startup.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["time"] + [name for i in range(16) for name in (f"q{i}", f"dq{i}")])
            # Endpoints match the recording's first pose. Only the middle
            # proves that preparation actually traverses the user's trajectory.
            for instant, angle in ((0, 0), (1, .2), (2, 0)):
                row = [instant]
                for i in range(16):
                    q = -.98 if i % 8 == 7 else (1.57 if i % 8 == 3 else (angle if i == 0 else 0))
                    row.extend((q, 0))
                writer.writerow(row)
        began = self.start(duration=.2, manual_start=True, wait_for_playing=False, startup=startup)
        _, progress = self.wait_for("preparation_time=1.5")
        self.assertGreater(float(progress.split("q0=")[1].split()[0]), .15)
        ready, _ = self.wait_for("[replay] READY")
        self.assertGreater(ready-began, 1.9)
        self.wait_for("phase=ready")
        self.process.stdin.write("play\n"); self.process.stdin.flush()
        self.wait_for("[replay] COMPLETE")
        returned = time.monotonic()
        self.process.stdin.write("prepare\n"); self.process.stdin.flush()
        _, alignment = self.wait_for("[replay] ALIGNING")
        # Direct connection is dominated by the .044 m gripper movement:
        # 1.875*.044/.02 = 4.125 seconds, without the 2.6-second startup.
        self.assertAlmostEqual(float(alignment.split("transition_seconds=")[1]), 4.125, places=3)
        ready_again, _ = self.wait_for("[replay] READY", timeout=12)
        self.assertAlmostEqual(ready_again-returned, 4.125, delta=.35)
        self.wait_for("phase=ready")

    def test_fast_motion_at_one_and_half_speed(self):
        for speed in (1, .5):
            with self.subTest(speed=speed):
                started = self.start(speed=speed)
                finished, _ = self.wait_for("[replay] COMPLETE")
                self.assertAlmostEqual(finished-started, 1/speed, delta=.25)
                self.process.stdin.write("stop\n"); self.process.stdin.flush()
                self.assertEqual(self.process.wait(timeout=3), 0)
                self.close()

    def test_pause_excludes_paused_time(self):
        started = self.start(duration=2)
        self.process.stdin.write("pause\n"); self.process.stdin.flush()
        _, first = self.wait_for("phase=paused")
        _, second = self.wait_for("phase=paused")
        self.assertEqual(first.split("time=")[1].split()[0], second.split("time=")[1].split()[0])
        resumed = time.monotonic()
        self.process.stdin.write("resume\n"); self.process.stdin.flush()
        finished, _ = self.wait_for("[replay] COMPLETE")
        self.assertAlmostEqual(finished-resumed, 2, delta=.25)
        self.assertGreater(finished-started, 2.7)

    def test_finite_loops_have_no_return_delay(self):
        started = self.start(duration=.4, repeat=3)
        finished, _ = self.wait_for("[replay] COMPLETE")
        self.assertAlmostEqual(finished-started, 1.2, delta=.25)
        _, line = self.wait_for("phase=complete")
        self.assertIn("loop=3 loops=3", line)

    def test_stall_catches_up_to_clock(self):
        started = self.start(duration=1)
        self.process.send_signal(signal.SIGSTOP)
        time.sleep(.65)
        self.process.send_signal(signal.SIGCONT)
        finished, _ = self.wait_for("[replay] COMPLETE")
        self.assertAlmostEqual(finished-started, 1, delta=.25)

    def test_infinite_loops_and_stop(self):
        self.start(duration=.2, repeat=0)
        self.wait_for("[replay] PLAYING loop=3")
        _, line = self.wait_for("mode=preview")
        self.assertIn("loops=inf", line)
        self.process.stdin.write("stop\n"); self.process.stdin.flush()
        self.assertEqual(self.process.wait(timeout=3), 0)

    def test_finish_infinite_loop_holds_last_frame_and_allows_next_play(self):
        self.start(duration=.5, repeat=0, manual_start=True)
        self.command("finish_loop")
        self.wait_for("LOOP_END_REJECTED")
        self.command("play")
        self.wait_for("[replay] PLAYING loop=2")
        self.command("pause")
        self.wait_for("phase=paused")
        self.command("finish_loop")
        self.wait_for("LOOP_END_REQUESTED loop=2")
        self.wait_for("phase=paused")
        self.command("resume")
        self.wait_for("[replay] COMPLETE")
        _, line = self.wait_for("phase=complete")
        self.assertIn("loop=2 loops=2", line)
        self.assertIn("q0=1", line)
        self.wait_for("phase=complete") # remains holding, no new cycle
        self.command("prepare")
        self.wait_for("[replay] READY", timeout=10)
        self.command("repeat 0")
        self.wait_for("REPEAT_SET loops=0")
        self.command("play")
        self.wait_for("[replay] PLAYING loop=2")
        self.command("finish_loop")
        self.wait_for("LOOP_END_REQUESTED")
        self.wait_for("[replay] COMPLETE")
        self.command("stop")
        self.assertEqual(self.process.wait(timeout=3), 0)

    def command(self, text):
        self.process.stdin.write(text + "\n")
        self.process.stdin.flush()

    def test_restart_after_completion_resets_loops_without_restarting_process(self):
        self.start(duration=.2, repeat=2)
        pid = self.process.pid
        self.wait_for("[replay] COMPLETE")
        for _ in range(2):
            sent = time.monotonic()
            self.command("restart")
            self.command("restart")  # The second request during playback is ignored.
            self.wait_for("PLAYING mode=preview loop=1 restart=1")
            _, progress = self.wait_for("phase=playing")
            self.assertIn("loop=1 loops=2", progress)
            completed, _ = self.wait_for("[replay] COMPLETE")
            self.assertAlmostEqual(completed-sent, .4, delta=.25)
            self.assertEqual(self.process.pid, pid)
            self.assertIsNone(self.process.poll())
        self.command("stop")
        self.assertEqual(self.process.wait(timeout=3), 0)

    def test_manual_start_holds_first_frame_until_play_and_again_after_prepare(self):
        self.start(duration=.2, repeat=2, initial_joint=.15, manual_start=True,
                   wait_for_playing=False)
        self.command("play")  # Too early: alignment must finish before play is accepted.
        self.wait_for("[replay] READY")
        for _ in range(2):
            _, line = self.wait_for("phase=ready")
            self.assertIn("time=0 duration=0.2", line)
            self.assertIn("loop=1 loops=2", line)
        self.command("resume")  # Resume is not a substitute for explicit play.
        self.command("seek .1")
        self.wait_for("SEEK_REJECTED: waiting for play")
        self.wait_for("phase=ready")
        sent = time.monotonic()
        self.command("play")
        self.wait_for("[replay] PLAYING")
        completed, _ = self.wait_for("[replay] COMPLETE")
        self.assertAlmostEqual(completed-sent, .4, delta=.25)
        pid = self.process.pid
        for invalid in ("-1", "10001", "2x", "nan"):
            self.command("repeat " + invalid)
            self.wait_for("REPEAT_REJECTED")
        self.command("repeat 3")
        self.wait_for("REPEAT_SET loops=3")
        self.command("prepare")
        self.wait_for("[replay] ALIGNING")
        self.command("pause")
        self.wait_for("phase=paused")
        self.command("resume")
        self.wait_for("[replay] READY", timeout=10)
        _, line = self.wait_for("phase=ready")
        self.assertIn("loop=1 loops=3", line)
        self.assertEqual(self.process.pid, pid)
        self.command("play")
        self.wait_for("[replay] PLAYING")
        self.command("repeat 1")
        self.wait_for("REPEAT_REJECTED")
        self.wait_for("[replay] COMPLETE")
        _, line = self.wait_for("phase=complete")
        self.assertIn("loop=3 loops=3", line)
        self.command("stop")
        self.assertEqual(self.process.wait(timeout=3), 0)

    def test_stop_while_ready_never_starts_playback(self):
        self.start(duration=.2, manual_start=True)
        self.wait_for("phase=ready")
        self.command("stop")
        self.assertEqual(self.process.wait(timeout=3), 0)

    def test_startup_alignment_is_pausable_and_excluded_from_playback(self):
        self.start(duration=1, initial_joint=.3, wait_for_playing=False)
        self.command("pause")
        _, line = self.wait_for("phase=paused")
        self.assertIn("time=0 duration=1", line)
        self.command("seek .5")
        self.wait_for("SEEK_REJECTED: aligning")
        self.command("resume")
        resumed = time.monotonic()
        started, _ = self.wait_for("[replay] PLAYING")
        self.assertGreater(started-resumed, 1.2)
        _, line = self.wait_for("phase=playing")
        self.assertIn("time=0 duration=1", line)
        finished, _ = self.wait_for("[replay] COMPLETE")
        self.assertAlmostEqual(finished-started, 1, delta=.25)

    def test_seek_while_playing_and_after_completion(self):
        self.start(duration=4)
        sent = time.monotonic()
        self.command("seek 3.5")
        self.wait_for("SEEKED time=3.5")
        finished, _ = self.wait_for("[replay] COMPLETE")
        self.assertAlmostEqual(finished-sent, .5, delta=.25)
        self.command("seek 1")
        self.wait_for("SEEKED time=1")
        _, line = self.wait_for("phase=paused")
        self.assertIn("time=1 duration=4", line)
        self.command("seek 2")
        self.wait_for("SEEKED time=2")
        _, line = self.wait_for("phase=paused")
        self.assertIn("time=2 duration=4", line)
        resumed = time.monotonic()
        self.command("resume")
        finished, _ = self.wait_for("[replay] COMPLETE")
        self.assertAlmostEqual(finished-resumed, 2, delta=.25)

    def test_seek_rejects_invalid_and_clamps_out_of_range(self):
        self.start(duration=2)
        self.command("pause")
        self.wait_for("phase=paused")
        for value in ("nan", "inf", "bad", "1garbage"):
            self.command("seek " + value)
            self.wait_for("SEEK_REJECTED")
        self.command("seek -1")
        self.wait_for("SEEKED time=0")
        _, line = self.wait_for("phase=paused")
        self.assertIn("time=0 duration=2", line)
        self.command("seek 99")
        self.wait_for("SEEKED time=2")
        self.wait_for("[replay] COMPLETE")
