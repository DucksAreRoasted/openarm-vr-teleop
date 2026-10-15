#!/usr/bin/env python3
"""Compare real OpenArm controllers using synthetic Quest input in headless MuJoCo.

Never opens CAN. Raw telemetry and the JSON report remain in the output directory.
This measures simulation behavior, not headset/network latency or real-arm safety.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
# (phase, seconds, OpenXR translation axis / rotation axis)
PHASES = [("settle", 1.2, None), ("forward", 6, 2), ("lateral", 3, 0),
          ("vertical", 3, 1), ("yaw", 3, 1), ("pitch", 3, 0), ("roll", 3, 2),
          ("reach", 2, 2), ("release", .5, None), ("regrip", 1, None),
          ("gap", .4, None), ("recovery", 1, None)]


def reserve_ports():
    sockets = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(4)]
    try:
        for sock in sockets:
            sock.bind(("127.0.0.1", 0))
        return [sock.getsockname()[1] for sock in sockets]
    finally:
        for sock in sockets:
            sock.close()


def run_profile(mode, output, xml, hz):
    folder = output / mode
    folder.mkdir()
    ports = reserve_ports()
    environment = {**os.environ, "OPENARM_MUJOCO_HEADLESS": "1",
                   "OPENARM_QUEST_UDP_HOST": "127.0.0.1", "OPENARM_LEROBOT": "0",
                   "OPENARM_TELEOP_TRAJ_LOG": str(folder / "trajectory"),
                   "OPENARM_HM_LOG_DIR": str(folder / "debug"),
                   "OPENARM_HM_DEBUG_LOG": "1", "OPENARM_HM_LOG_HZ": "90"}
    # Keep outputs separate even if a local tuning shell specified a debug file.
    environment.pop("OPENARM_HM_LOG_PATH", None)
    command = [str(ROOT / "build/quest_cartesian_control"), "--sim", "--bimanual",
               "--urdf", str(ROOT / "assets/v1.urdf"), "--xml", str(xml),
               "--teleop-mode", mode, "--quest-port-left", str(ports[0]),
               "--quest-port-right", str(ports[1]), "--quest-ui-record-port", str(ports[2]),
               "--quest-ui-jog-port", str(ports[3])]
    intervals = []
    log_path = folder / "controller.log"
    with log_path.open("w") as log, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        process = subprocess.Popen(command, cwd=ROOT, env=environment, stdout=log,
                                   stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 20
            while "[quest-cartesian] bimanual running." not in log_path.read_text():
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError(log_path.read_text()[-5000:])
                time.sleep(.02)
            anchor = Rotation.from_euler("yx", [45, 30], degrees=True)
            with (folder / "input.csv").open("w") as handle:
                writer = csv.writer(handle)
                writer.writerow(["steady_s", "phase", "grip", "dx", "dy", "dz"])
                for phase, duration, axis in PHASES:
                    print(f"{mode}: {phase}", flush=True)
                    start = time.monotonic()
                    end = start + duration
                    intervals.append((phase, start, end))
                    tick = start
                    while time.monotonic() < end:
                        now = time.monotonic()
                        if process.poll() is not None:
                            raise RuntimeError(log_path.read_text()[-5000:])
                        elapsed = now - start
                        delta = np.zeros(3)
                        rot = anchor
                        if phase in ("forward", "lateral", "vertical"):
                            delta[axis] = .035 * math.sin(math.pi * elapsed)
                        elif phase in ("yaw", "pitch", "roll"):
                            vector = np.zeros(3)
                            vector[axis] = .18 * math.sin(math.pi * elapsed)
                            rot = Rotation.from_rotvec(vector) * anchor
                        elif phase == "reach":
                            delta[axis] = -.9 * min(elapsed, 1)
                        grip = phase != "release" and not (phase == "settle" and elapsed < .4)
                        # Move the hands while released; regrip must capture a new anchor.
                        if phase in ("release", "regrip", "gap", "recovery"):
                            delta[0] = .15
                        writer.writerow([now, phase, int(grip), *delta])
                        if phase != "gap":
                            for arm, port in enumerate(ports[:2]):
                                position = np.array([.2 if arm == 0 else -.2, 1.2, -.3]) + delta
                                payload = [int(now * 1000), int(grip), 0, *position, *rot.as_quat(), 0, 0]
                                sock.sendto(",".join(map(str, payload)).encode(), ("127.0.0.1", port))
                        tick += 1 / hz
                        time.sleep(max(0, tick - time.monotonic()))
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                raise RuntimeError(f"{mode}: controller did not stop")
    if process.returncode != 0:
        raise RuntimeError(log_path.read_text()[-5000:])
    return summarize(folder, intervals)


def summarize(folder, intervals):
    urdf = ET.parse(ROOT / "assets/v1.urdf").getroot()
    result = {}
    for side in ("left", "right"):
        data = np.genfromtxt(folder / "trajectory" / f"{side}_tcp.csv", delimiter=",", names=True,
                             dtype=None, encoding="utf-8")
        actual = np.column_stack([data[f"tcp_{axis}"] for axis in "xyz"])
        requested = np.column_stack([data[f"requested_{axis}"] for axis in "xyz"])
        times = data["steady_s"]
        reference = np.column_stack([data[f"ref{i}"] for i in range(7)])
        limits = [urdf.find(f"./joint[@name='openarm_{side}_joint{i}']/limit") for i in range(1, 8)]
        lower = np.array([float(limit.get("lower")) for limit in limits])
        upper = np.array([float(limit.get("upper")) for limit in limits])
        if not np.isfinite(reference).all():
            raise AssertionError(f"{side}: nonfinite command")
        if np.any(reference < lower - 1e-5) or np.any(reference > upper + 1e-5):
            raise AssertionError(f"{side}: commanded joint outside URDF limits")
        phases = {}
        for phase, start, end in intervals:
            mask = (times >= start + .2) & (times < end) & np.isfinite(requested).all(axis=1)
            if mask.sum() < 3:
                continue
            error = np.linalg.norm(actual[mask] - requested[mask], axis=1)
            item = {"tcp_rmse_mm": float(np.sqrt(np.mean(error ** 2)) * 1000),
                    "tcp_p95_mm": float(np.percentile(error, 95) * 1000),
                    "ik_p95_ms": float(np.percentile(data["ik_ms"][mask], 95)),
                    "max_reference_step_rad": float(np.abs(np.diff(reference[mask], axis=0)).max())}
            if "orientation_error_rad" in data.dtype.names:
                item["orientation_p95_deg"] = float(np.percentile(
                    np.rad2deg(data["orientation_error_rad"][mask]), 95))
            item["elbow_range_deg"] = float(np.rad2deg(np.ptp(reference[mask, 3])))
            if phase == "forward":
                t = times[mask]
                x = requested[mask, 0]
                y = actual[mask, 0]
                correlations = []
                for delay in np.linspace(0, .5, 101):
                    shifted = np.interp(t - delay, t, x)
                    correlations.append(float(np.corrcoef(shifted, y)[0, 1])
                                        if np.std(y) > 1e-6 and np.std(shifted) > 1e-6 else 0)
                item["estimated_sim_lag_ms"] = int(np.argmax(correlations) * 5)
                item["lag_correlation"] = max(correlations)
            if phase in ("forward", "lateral", "vertical"):
                axis = {"forward": 0, "lateral": 1, "vertical": 2}[phase]
                goal = requested[mask, axis]
                variance = np.var(goal)
                item["axis_gain"] = float(np.mean((goal - goal.mean()) *
                    (actual[mask, axis] - actual[mask, axis].mean())) / variance) if variance > 1e-10 else None
            phases[phase] = item
        debug = list((folder / "debug").rglob(f"{side}_*.log"))
        text = "\n".join(p.read_text() for p in debug)
        angles = [float(x) for x in re.findall(r" e_ori=([\d.e+-]+)", text)]
        result[side] = {"urdf_limits_ok": True, "phases": phases,
                        "ik_orientation_residual_p95_rad": float(np.percentile(angles, 95)) if angles else None}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--xml", type=Path, default=ROOT / "models/OpenArm_V1/scene_table.xml")
    parser.add_argument("--modes", nargs="+", choices=("unitree_style", "pika_style"),
                        default=("unitree_style", "pika_style"))
    parser.add_argument("--hz", type=float, default=90)
    args = parser.parse_args()
    if not math.isfinite(args.hz) or not 30 <= args.hz <= 120:
        parser.error("--hz must be between 30 and 120")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    report = {"description": "Synthetic Quest UDP / headless MuJoCo; no real hardware or headset latency",
              "xml": str(args.xml.resolve()), "input_hz": args.hz,
              "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
              "profiles": {}}
    for mode in dict.fromkeys(args.modes):
        report["profiles"][mode] = run_profile(mode, output, args.xml.resolve(), args.hz)
        (output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False))
    print(f"Report: {output / 'report.json'}")


if __name__ == "__main__":
    main()
