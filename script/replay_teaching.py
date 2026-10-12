#!/usr/bin/env python3
"""Build and run the standalone teaching replayer; simulation is the default."""
from __future__ import annotations

import argparse
import csv
import math
import os
import signal
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def build(command: list[str]) -> None:
    # A GUI launch may not inherit ROS's Python module paths from a terminal.
    setup = Path("/opt/ros/humble/setup.bash")
    if setup.is_file():
        command = ["bash", "-c", '. "$1"; shift; exec "$@"', "replay-build", str(setup), *command]
    subprocess.run(command, check=True)


def validate_split(path: Path) -> None:
    """Do not let the legacy merger silently drop malformed demonstration rows."""
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"time"} | {f"q{i}" for i in range(8)} | {f"dq{i}" for i in range(8)}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"{path}: 分臂文件缺少 time/q/dq 列")
        previous = None
        count = 0
        for row in reader:
            try:
                if None in row or any(value is None for value in row.values()):
                    raise ValueError("列数错误")
                values = [float(value) for value in row.values()]
                instant = float(row["time"])
                if not all(math.isfinite(value) for value in values) or instant < 0:
                    raise ValueError("非有限数值或负时间")
                if previous is not None and instant <= previous:
                    raise ValueError("时间未严格递增")
                previous = instant
                count += 1
            except (TypeError, ValueError) as error:
                raise ValueError(f"{path}:{reader.line_num}: {error}") from error
        if count < 2:
            raise ValueError(f"{path}: 至少需要两帧")


def resolve_source(path: Path, temporary: Path) -> Path:
    path = path.expanduser().resolve()
    if path.is_file():
        return path
    if path.is_dir():
        merged = path / "real_teleop_dual_merged.csv"
        if merged.is_file():
            return merged
        left = path / "real_teleop_split/left/real_teleop_left.csv"
        right = path / "real_teleop_split/right/real_teleop_right.csv"
        if left.is_file() and right.is_file():
            validate_split(left)
            validate_split(right)
            from merge_dual_real_recordings import merge
            merged = temporary / "replay.csv"
            merge(left, right, merged, None, "min")
            return merged
    raise ValueError("请选择片段目录（如批次/01）或双臂合并 CSV")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--sim", action="store_true")
    mode.add_argument("--real", action="store_true")
    parser.add_argument("--speed", type=float, default=.5)
    parser.add_argument("--left-can", default="can1")
    parser.add_argument("--right-can", default="can0")
    parser.add_argument("--source-meters", action="store_true")
    parser.add_argument("--check", action="store_true", help="只检查文件与限位，不使能电机")
    args = parser.parse_args()
    if not .1 <= args.speed <= 1:
        parser.error("speed 必须在 0.1～1.0 之间")
    # A private session lets the GUI stop compilation and the servo as one group.
    if os.getpgrp() != os.getpid():
        os.setsid()
    try:
        with tempfile.TemporaryDirectory(prefix="openarm-replay-") as directory:
            source = resolve_source(args.source, Path(directory))
            print("[replay] BUILDING", flush=True)
            build(["cmake", "-S", str(ROOT), "-B", str(ROOT / "build"),
                   f"-DPython3_EXECUTABLE={sys.executable}"])
            build(["cmake", "--build", str(ROOT / "build"), "--target",
                   "openarm_teaching_replay", "-j2"])
            command = [str(ROOT / "build/openarm_teaching_replay"), "--csv", str(source),
                       "--real" if args.real else "--sim", "--speed", str(args.speed),
                       "--urdf", os.environ.get("OPENARM_TELEOP_URDF", str(ROOT / "assets/v1.urdf")),
                       "--xml", os.environ.get("OPENARM_MUJOCO_XML", "/home/ducks/motion-trajectory-studio/model_library/OpenArm_V1/scene_table.xml"),
                       "--gravity-yaml", os.environ.get("OPENARM_GRAVITY_YAML", str(ROOT / "config/gravity_comp.yaml")),
                       "--left-can", args.left_can, "--right-can", args.right_can]
            if args.source_meters:
                command.append("--source-meters")
            if args.check:
                command.append("--check")
            # Keep the wrapper alive to clean temporary merged files after playback.
            child = None
            pending_signals = []
            def forward(sig, _frame):
                if child is None:
                    pending_signals.append(sig)
                elif child.poll() is None:
                    try:
                        child.send_signal(sig)
                    except ProcessLookupError:
                        pass
            signal.signal(signal.SIGINT, forward)
            signal.signal(signal.SIGTERM, forward)
            child = subprocess.Popen(command, cwd=ROOT)
            for sig in pending_signals:
                forward(sig, None)
            # Do not report completion to Qt until the servo has disabled motors.
            return child.wait()
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f"[replay][ERROR] {error}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
