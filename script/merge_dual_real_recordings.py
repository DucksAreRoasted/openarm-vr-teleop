#!/usr/bin/env python3
"""Merge the two eight-channel VR recordings into one editor-ready CSV."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from pathlib import Path


def load(path: Path, count: int = 8):
    times, positions, velocities, torques = [], [], [], []
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        fields = set(reader.fieldnames or [])
        required = {"time"} | {f"q{i}" for i in range(count)} | {f"dq{i}" for i in range(count)}
        missing = sorted(required - fields)
        if missing:
            raise ValueError(f"{path} missing columns: {', '.join(missing)}")
        has_torque = all(f"tau{i}" in fields for i in range(count))
        for row in reader:
            try:
                values = [float(row["time"])]
                values += [float(row[f"q{i}"]) for i in range(count)]
                values += [float(row[f"dq{i}"]) for i in range(count)]
                values += [float(row[f"tau{i}"]) for i in range(count)] if has_torque else [0.0] * count
            except (KeyError, TypeError, ValueError):
                continue
            if not all(math.isfinite(value) for value in values):
                continue
            if times and values[0] <= times[-1]:
                continue
            times.append(values[0])
            positions.append(values[1:1 + count])
            velocities.append(values[1 + count:1 + count * 2])
            torques.append(values[1 + count * 2:])
    if len(times) < 2:
        raise ValueError(f"{path}: not enough valid frames")
    return times, positions, velocities, torques


def interpolate(times, values, instant):
    if instant <= times[0]:
        return values[0]
    if instant >= times[-1]:
        return values[-1]
    high = bisect.bisect_right(times, instant)
    low = high - 1
    alpha = (instant - times[low]) / (times[high] - times[low])
    return [a + (b - a) * alpha for a, b in zip(values[low], values[high])]


def inferred_hz(*series):
    deltas = sorted(
        b - a for times in series for a, b in zip(times, times[1:]) if 1e-5 < b - a < 1.0
    )
    return max(1.0, min(1000.0, 1.0 / deltas[len(deltas) // 2])) if deltas else 1000.0


def merge(left: Path, right: Path, output: Path, hz: float | None, crop: str):
    lt, lq, ldq, ltau = load(left)
    rt, rq, rdq, rtau = load(right)
    durations = (lt[-1] - lt[0], rt[-1] - rt[0])
    duration = min(durations) if crop == "min" else max(durations)
    rate = hz or inferred_hz(lt, rt)
    steps = int(math.floor(duration * rate))
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time"] + [name for i in range(16) for name in (f"q{i}", f"dq{i}", f"tau{i}")])
        for step in range(steps + 1):
            instant = min(step / rate, duration)
            groups = (
                interpolate(lt, lq, lt[0] + instant), interpolate(lt, ldq, lt[0] + instant),
                interpolate(lt, ltau, lt[0] + instant), interpolate(rt, rq, rt[0] + instant),
                interpolate(rt, rdq, rt[0] + instant), interpolate(rt, rtau, rt[0] + instant),
            )
            row = [f"{instant:.6f}"]
            for arm in (0, 1):
                for joint in range(8):
                    base = arm * 3
                    row.extend(f"{groups[base + offset][joint]:.9f}" for offset in range(3))
            writer.writerow(row)
    print(f"[merge] output={output} hz={rate:.3f} duration={duration:.3f}s rows={steps + 1}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--left", required=True, type=Path)
    parser.add_argument("--right", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--hz", type=float)
    parser.add_argument("--crop", choices=("min", "max"), default="min")
    args = parser.parse_args()
    merge(args.left, args.right, args.output, args.hz, args.crop)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
