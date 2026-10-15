#!/usr/bin/env python3
"""Create reversible, offline smoothing derivatives of recorded arm commands."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import tempfile

import numpy as np
from scipy.ndimage import gaussian_filter1d

from replay_teaching import resolve_source


def read_csv(path: Path) -> tuple[list[str], list[list[str]], np.ndarray]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        header = next(reader, [])
        rows = [row for row in reader if row]
    if not header or header[0] != "time" or len(set(header)) != len(header):
        raise ValueError(f"{path.name}: 缺少 time 或存在重复列")
    if len(rows) < 3 or any(len(row) != len(header) for row in rows):
        raise ValueError(f"{path.name}: 至少需要三帧，且每帧列数须一致")
    values = np.asarray(rows, dtype=np.float64)
    if not np.isfinite(values).all():
        raise ValueError(f"{path.name}: 存在非有限数值")
    time = values[:, 0]
    if time[0] < 0 or np.any(np.diff(time) <= 0):
        raise ValueError(f"{path.name}: 时间须非负且严格递增")
    return header, rows, values


def smooth_positions(time: np.ndarray, positions: np.ndarray, window_ms: float,
                     max_deviation_deg: float) -> tuple[np.ndarray, np.ndarray, dict]:
    """Symmetric Gaussian on a uniform time grid, tapered to exact endpoints.

    The taper and per-joint deviation scale are convex blends, so positions
    cannot overshoot the original per-joint range. Commands keep original times.
    """
    if not np.isfinite(window_ms) or not 20 <= window_ms <= 500:
        raise ValueError("平滑窗口须在 20～500 ms 之间")
    if not np.isfinite(max_deviation_deg) or not .1 <= max_deviation_deg <= 10:
        raise ValueError("最大改动须在 0.1～10 度之间")
    time = np.asarray(time, dtype=np.float64)
    positions = np.asarray(positions, dtype=np.float64)
    if (time.ndim != 1 or len(time) < 3 or positions.ndim != 2 or
            len(positions) != len(time) or not np.isfinite(time).all() or
            not np.isfinite(positions).all() or time[0] < 0 or np.any(np.diff(time) <= 0)):
        raise ValueError("平滑输入须为至少三帧有限位置，且时间严格递增")
    delta = np.diff(time)
    step = float(np.median(delta))
    if np.max(delta) > max(.25, 10 * step):
        raise ValueError("轨迹存在较长采样间断，请先检查缺帧，不能跨间断平滑")
    count = max(3, int(np.ceil((time[-1]-time[0])/step))+1)
    if count > 2_000_000:
        raise ValueError("均匀采样超过 200 万帧，请分段处理")
    grid = np.linspace(time[0], time[-1], count)
    uniform = np.column_stack([np.interp(grid, time, col) for col in positions.T])
    window = window_ms / 1000
    filtered = gaussian_filter1d(uniform, sigma=window/4/(grid[1]-grid[0]),
                                 axis=0, mode="reflect", truncate=4)
    filtered = np.column_stack([np.interp(time, grid, col) for col in filtered.T])
    edge = np.clip(np.minimum(time-time[0], time[-1]-time)/window, 0, 1)
    edge = edge**3 * (10 + edge*(-15+6*edge))
    change = (filtered-positions)*edge[:, None]
    change[:, np.ptp(positions, axis=0) < 1e-12] = 0
    peak = np.max(np.abs(change), axis=0)
    gain = np.minimum(1., np.deg2rad(max_deviation_deg)/np.maximum(peak, 1e-15))
    result = positions + change*gain
    result[0], result[-1] = positions[0], positions[-1]
    velocity = np.gradient(result, time, axis=0, edge_order=2)
    original_velocity = np.gradient(positions, time, axis=0, edge_order=2)
    # High-frequency residual around the low-pass curve is a diagnostic, not a
    # claim of improved tracking, collision clearance, or training success.
    stats = {
        "max_deviation_deg": np.rad2deg(np.max(np.abs(result-positions), axis=0)).tolist(),
        "peak_speed_before_rad_s": np.max(np.abs(original_velocity), axis=0).tolist(),
        "peak_speed_after_rad_s": np.max(np.abs(velocity), axis=0).tolist(),
        "velocity_rms_before_rad_s": float(np.sqrt(np.mean(original_velocity**2))),
        "velocity_rms_after_rad_s": float(np.sqrt(np.mean(velocity**2))),
        "frames": len(time), "duration_s": float(time[-1]-time[0]),
    }
    return result, velocity, stats


def _write(path: Path, header: list[str], rows: list[list[str]]) -> None:
    with path.open("x", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        writer.writerows(rows)


def smooth_recording(source: Path, *, window_ms: float = 120,
                     max_deviation_deg: float = 3, output_root: Path | None = None) -> dict:
    source = source.expanduser().resolve()
    source_folder = source if source.is_dir() else source.parent
    with tempfile.TemporaryDirectory(prefix="openarm-smooth-input-") as temporary:
        trajectory = resolve_source(source, Path(temporary))
        header, rows, values = read_csv(trajectory)
        expected = ["time"] + [name for i in range(16) for name in (f"q{i}", f"dq{i}")]
        with_torque = ["time"] + [name for i in range(16) for name in (f"q{i}", f"dq{i}", f"tau{i}")]
        if header not in (expected, with_torque):
            raise ValueError("请选择按左8、右8排列的双臂 time/q/dq 或 time/q/dq/tau CSV")
        joints = [i for i in range(16) if i % 8 != 7]
        qcols = [header.index(f"q{i}") for i in joints]
        dqcols = [header.index(f"dq{i}") for i in joints]
        q, dq, stats = smooth_positions(values[:, 0], values[:, qcols], window_ms, max_deviation_deg)
        for index, (row, positions, velocities) in enumerate(zip(rows, q, dq)):
            if index not in (0, len(rows)-1):
                for col, value in zip(qcols, positions): row[col] = format(float(value), ".17g")
            for col, value in zip(dqcols, velocities): row[col] = format(float(value), ".17g")
        # Finish all validation before creating the derivative directory.
        actions = None
        action_source = source_folder / "lerobot_episode.csv"
        if action_source.is_file():
            ah, ar, av = read_csv(action_source)
            names = [f"{side}_act{i}" for side in ("l", "r") for i in range(7)]
            if not set(names).issubset(ah):
                raise ValueError("训练 CSV 缺少双臂动作指令列")
            cols = [ah.index(name) for name in names]
            aq, _, action_stats = smooth_positions(av[:, 0], av[:, cols], window_ms, max_deviation_deg)
            for index, (row, positions) in enumerate(zip(ar, aq)):
                if index not in (0, len(ar)-1):
                    for col, value in zip(cols, positions): row[col] = format(float(value), ".17g")
            actions = ah, ar, action_stats
        parent = (output_root.expanduser().resolve() if output_root else source_folder / "processed")
        parent.mkdir(parents=True, exist_ok=True)
        output = Path(tempfile.mkdtemp(prefix=f"smooth_{datetime.now():%Y%m%d_%H%M%S}_", dir=parent))
        try:
            target = output / "real_teleop_dual_merged.csv"
            _write(target, header, rows)
            for name in ("collection_mode.json", "outcome.txt"):
                if (source_folder/name).is_file(): shutil.copy2(source_folder/name, output/name)
            report = {
                "method": "symmetric_gaussian_endpoint_taper_v1", "window_ms": window_ms,
                "deviation_limit_deg": max_deviation_deg, "source": str(source),
                "source_trajectory_sha256": hashlib.sha256(trajectory.read_bytes()).hexdigest(),
                "trajectory": str(target), "output_dir": str(output), "joints": joints,
                "trajectory_stats": stats,
                "torque_columns": "original logged values; not recomputed and not used by replay",
                "training_export": "derived actions only; evaluate before using with original observations",
            }
            if actions:
                action_path = output / "lerobot_actions_smoothed.csv"
                _write(action_path, actions[0], actions[1])
                report.update(training_actions=str(action_path), training_action_stats=actions[2],
                              training_source_sha256=hashlib.sha256(action_source.read_bytes()).hexdigest())
            (output/"smoothing_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n")
        except Exception:
            shutil.rmtree(output)
            raise
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--window-ms", type=float, default=120)
    parser.add_argument("--max-deviation-deg", type=float, default=3)
    parser.add_argument("--output-root", type=Path)
    args = parser.parse_args()
    try:
        report = smooth_recording(args.source, window_ms=args.window_ms,
                                  max_deviation_deg=args.max_deviation_deg, output_root=args.output_root)
        print(json.dumps(report, ensure_ascii=False), flush=True)
        return 0
    except Exception as error:
        print(str(error), flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
