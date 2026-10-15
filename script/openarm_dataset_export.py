#!/usr/bin/env python3
"""Export Quest teleop segments to OpenArm Dataset v0.4.0 (+ optional LeRobot).

Reads labeled takes under ``recordings/sessions/<batch>/<NN>/``:

  lerobot_episode.csv   measured qpos (obs) + MIT command qpos (action)
  outcome.txt           success | failure
  cameras/<name>/       NNNNNN.jpg + times.csv

Writes the official layout documented at https://docs.openarm.dev/dataset/ :

  <batch>/openarm_dataset/
    metadata.yaml
    episodes/<id>/obs/arms/{left,right}/state.parquet
    episodes/<id>/action/arms/{left,right}/state.parquet
    episodes/<id>/cameras/<name>/<unix_ns>.jpeg

Then converts to LeRobot when possible:

  1. ``openarm_dataset.Dataset.write(..., format=lerobot_v3.0)`` if installed
  2. else ``openarm-dataset-convert`` on PATH
  3. else the local ``lerobot_collect.py`` fallback (CSV → lerobot/)
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

ROOT = Path(__file__).resolve().parent
CAMERA_CANDIDATES = ("head", "wrist_left", "wrist_right", "ceiling")
VERSION = "0.4.0"


def _load_episode_csv(path: Path):
    rows = []
    with path.open() as handle:
        header = handle.readline()
        if "l_obs0" not in header:
            raise ValueError(f"{path} is not a lerobot_episode.csv")
        for line in handle:
            line = line.strip()
            if not line:
                continue
            parts = [float(item) for item in line.split(",")]
            if len(parts) not in (1 + 4 * 8, 2 + 4 * 8):
                continue
            rows.append(parts)
    if not rows:
        empty = np.zeros((0,), dtype=np.float64)
        z = np.zeros((0, 8), dtype=np.float32)
        return empty, z, z, z, z, empty
    width = max(len(row) for row in rows)
    data = np.zeros((len(rows), width), dtype=np.float64)
    for i, row in enumerate(rows):
        data[i, : len(row)] = row
    times = data[:, 0]
    left_obs = data[:, 1:9].astype(np.float32)
    left_act = data[:, 9:17].astype(np.float32)
    right_obs = data[:, 17:25].astype(np.float32)
    right_act = data[:, 25:33].astype(np.float32)
    unix = data[:, 33] if width > 33 else np.array([], dtype=np.float64)
    return times, left_obs, left_act, right_obs, right_act, unix


def _read_outcome(path: Path) -> bool | None:
    if not path.is_file():
        return None
    text = path.read_text(encoding="utf-8").strip().lower()
    if text in {"success", "1", "true"}:
        return True
    if text in {"failure", "fail", "0", "false"}:
        return False
    return None


def _episode_id(folder: Path) -> str:
    name = folder.name
    if name.isdigit():
        return str(int(name))
    return name


def discover_labeled_episodes(batch: Path, min_duration: float) -> list[dict]:
    files = sorted(batch.glob("*/lerobot_episode.csv"))
    if not files and (batch / "lerobot_episode.csv").is_file():
        files = [batch / "lerobot_episode.csv"]
    episodes = []
    for path in files:
        folder = path.parent
        success = _read_outcome(folder / "outcome.txt")
        if success is None:
            print(f"[openarm-dataset] skip unlabeled {folder.name}", file=sys.stderr)
            continue
        times, left_obs, left_act, right_obs, right_act, unix = _load_episode_csv(path)
        if len(times) == 0:
            print(f"[openarm-dataset] skip empty {folder.name}", file=sys.stderr)
            continue
        duration = float(times[-1] - times[0])
        if duration < min_duration:
            print(
                f"[openarm-dataset] skip {folder.name}: {duration:.2f}s < {min_duration:.2f}s",
                file=sys.stderr,
            )
            continue
        if len(unix) != len(times):
            # Fall back to wall-clock from segment start using relative times.
            base = float(pd.Timestamp.now().timestamp()) - float(times[-1])
            unix = base + times
        episodes.append(
            {
                "id": _episode_id(folder),
                "folder": folder,
                "success": success,
                "times": times,
                "unix": unix.astype(np.float64),
                "left_obs": left_obs,
                "left_act": left_act,
                "right_obs": right_obs,
                "right_act": right_act,
                "cameras": folder / "cameras",
            }
        )
    return episodes


def _write_state_parquet(path: Path, unix: np.ndarray, qpos: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ts = pd.to_datetime((unix * 1e9).astype(np.int64), unit="ns")
    frame = pd.DataFrame(
        {
            "timestamp": ts,
            "qpos": [row.astype(float).tolist() for row in qpos],
        }
    )
    frame.to_parquet(path, index=False)


def _camera_times(folder: Path) -> tuple[np.ndarray, list[Path]]:
    times_path = folder / "times.csv"
    frames = sorted(folder.glob("*.jpg")) + sorted(folder.glob("*.jpeg"))
    if not times_path.is_file() or not frames:
        return np.array([], dtype=np.float64), []
    unix = []
    with times_path.open(encoding="utf-8") as handle:
        next(handle, None)
        for line in handle:
            parts = line.strip().split(",")
            if len(parts) >= 2:
                unix.append(float(parts[1]))
    count = min(len(unix), len(frames))
    return np.asarray(unix[:count], dtype=np.float64), frames[:count]


def _copy_cameras(src_root: Path, dest_root: Path) -> list[str]:
    present = []
    if not src_root.is_dir():
        return present
    for name in CAMERA_CANDIDATES:
        folder = src_root / name
        times, frames = _camera_times(folder)
        if len(frames) == 0:
            continue
        out = dest_root / name
        if out.exists():
            shutil.rmtree(out)
        out.mkdir(parents=True, exist_ok=True)
        for unix_s, src in zip(times, frames):
            stem = f"{int(round(float(unix_s) * 1e9))}"
            dest = out / f"{stem}.jpeg"
            shutil.copy2(src, dest)
        present.append(name)
    return present


def _hz(unix: np.ndarray) -> float:
    if len(unix) < 2:
        return 30.0
    dt = float(np.median(np.diff(unix)))
    if dt <= 0:
        return 30.0
    return float(1.0 / dt)


def write_openarm_dataset(
    batch: Path,
    out: Path,
    task: str,
    operator: str,
    location: str,
    min_duration: float,
) -> Path | None:
    episodes = discover_labeled_episodes(batch, min_duration)
    if not episodes:
        if out.exists():
            shutil.rmtree(out)
        print(f"[openarm-dataset] no labeled episodes under {batch}", file=sys.stderr)
        return None

    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True, exist_ok=True)

    camera_names: set[str] = set()
    obs_hz_l: list[float] = []
    obs_hz_r: list[float] = []
    cam_hz: dict[str, list[float]] = {}

    meta_episodes = []
    for ep in episodes:
        eid = ep["id"]
        base = out / "episodes" / eid
        _write_state_parquet(
            base / "obs" / "arms" / "left" / "state.parquet",
            ep["unix"],
            ep["left_obs"],
        )
        _write_state_parquet(
            base / "obs" / "arms" / "right" / "state.parquet",
            ep["unix"],
            ep["right_obs"],
        )
        _write_state_parquet(
            base / "action" / "arms" / "left" / "state.parquet",
            ep["unix"],
            ep["left_act"],
        )
        _write_state_parquet(
            base / "action" / "arms" / "right" / "state.parquet",
            ep["unix"],
            ep["right_act"],
        )
        present = _copy_cameras(ep["cameras"], base / "cameras")
        camera_names.update(present)
        obs_hz_l.append(_hz(ep["unix"]))
        obs_hz_r.append(_hz(ep["unix"]))
        for name in present:
            times, _frames = _camera_times(ep["cameras"] / name)
            cam_hz.setdefault(name, []).append(_hz(times))
        meta_episodes.append(
            {"id": eid, "success": bool(ep["success"]), "task_index": 0}
        )
        label = "success" if ep["success"] else "failure"
        print(
            f"[openarm-dataset] episode {eid}: {len(ep['unix'])} frames "
            f"({label}, cams={present or 'none'})"
        )

    def mean_or(values: list[float], default: float = 30.0) -> float:
        return float(np.mean(values)) if values else default

    cameras_meta = {name: {} for name in sorted(camera_names)}
    frequencies = {
        "action": {
            "arms": {
                "left": mean_or(obs_hz_l),
                "right": mean_or(obs_hz_r),
            }
        },
        "obs": {
            "arms": {
                "left": mean_or(obs_hz_l),
                "right": mean_or(obs_hz_r),
            }
        },
        "cameras": {
            name: mean_or(values) for name, values in sorted(cam_hz.items())
        },
    }

    metadata = {
        "version": VERSION,
        "location": location,
        "operator": operator,
        "operation_type": "teleop",
        "tasks": [{"prompt": task, "description": task}],
        "episodes": meta_episodes,
        "equipment": {
            "id": "openarm_vr_teleop",
            "version": "1.0",
            "embodiments": {
                "arms": {
                    "id": "OpenArm",
                    "version": "2.0",
                }
            },
            "perceptions": {"cameras": cameras_meta},
            "leader": {
                "quest": {
                    "id": "MetaQuest",
                    "hardware_version": "3",
                }
            },
        },
        "frequencies": frequencies,
    }
    with (out / "metadata.yaml").open("w", encoding="utf-8") as handle:
        yaml.safe_dump(metadata, handle, sort_keys=False, allow_unicode=True)
    print(f"[openarm-dataset] wrote {out} ({len(meta_episodes)} episodes, v{VERSION})")
    return out


def convert_to_lerobot(
    openarm_root: Path,
    lerobot_out: Path,
    batch: Path,
    fps: int,
    task: str,
    repo_id: str,
    min_duration: float,
    format_name: str,
) -> bool:
    # Prefer the official package API.
    try:
        import openarm_dataset

        if lerobot_out.exists():
            shutil.rmtree(lerobot_out)
        dataset = openarm_dataset.Dataset(openarm_root)
        dataset.write(
            lerobot_out,
            format=format_name,
            fps=fps,
            smoothing_cutoff=1.0,
            train_split=0.8,
            success_only=False,
            state="qpos",
        )
        print(f"[openarm-dataset] lerobot via openarm_dataset → {lerobot_out}")
        return True
    except Exception as exc:  # noqa: BLE001 — fall through to CLI / local
        print(f"[openarm-dataset] openarm_dataset convert skipped: {exc}", file=sys.stderr)
        if lerobot_out.exists():
            shutil.rmtree(lerobot_out, ignore_errors=True)

    convert = shutil.which("openarm-dataset-convert")
    if convert:
        if lerobot_out.exists():
            shutil.rmtree(lerobot_out)
        cmd = [
            convert,
            str(openarm_root),
            str(lerobot_out),
            "--format",
            format_name,
            "--fps",
            str(fps),
        ]
        result = subprocess.run(cmd, check=False)
        if result.returncode == 0:
            print(f"[openarm-dataset] lerobot via {convert} → {lerobot_out}")
            return True
        print("[openarm-dataset] openarm-dataset-convert failed", file=sys.stderr)

    # Local CSV→LeRobot writer (same layout as before).
    collect = ROOT / "lerobot_collect.py"
    if not collect.is_file():
        print("[openarm-dataset] no LeRobot converter available", file=sys.stderr)
        return False
    cmd = [
        sys.executable,
        str(collect),
        "--batch",
        str(batch),
        "--task",
        task,
        "--fps",
        str(fps),
        "--repo-id",
        repo_id,
        "--min-duration",
        str(min_duration),
    ]
    result = subprocess.run(cmd, check=False)
    if result.returncode != 0:
        print("[openarm-dataset] lerobot_collect.py failed", file=sys.stderr)
        return False
    print(f"[openarm-dataset] lerobot via lerobot_collect.py → {batch / 'lerobot'}")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Export teleop batch to OpenArm Dataset (+ LeRobot)"
    )
    parser.add_argument("--batch", type=Path, required=True)
    parser.add_argument("--task", default="bimanual teleoperation")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--repo-id", default="local/openarm_vr_teleop")
    parser.add_argument("--min-duration", type=float, default=1.0)
    parser.add_argument("--operator", default="operator")
    parser.add_argument("--location", default="lab")
    parser.add_argument(
        "--lerobot-format",
        default="lerobot_v3.0",
        choices=["lerobot_v3.0", "lerobot_v2.1", "none"],
        help="LeRobot export format; 'none' skips conversion",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="OpenArm dataset root (default: <batch>/openarm_dataset)",
    )
    args = parser.parse_args()
    if args.fps <= 0:
        print("[openarm-dataset] fps must be positive", file=sys.stderr)
        return 2
    if not args.batch.is_dir():
        print(f"[openarm-dataset] batch not found: {args.batch}", file=sys.stderr)
        return 2

    out = args.out or (args.batch / "openarm_dataset")
    written = write_openarm_dataset(
        args.batch,
        out,
        args.task,
        args.operator,
        args.location,
        args.min_duration,
    )
    if written is None:
        # Keep a stale lerobot/ from being treated as current.
        stale = args.batch / "lerobot"
        if stale.exists():
            shutil.rmtree(stale)
        return 0

    if args.lerobot_format == "none":
        return 0
    ok = convert_to_lerobot(
        written,
        args.batch / "lerobot",
        args.batch,
        args.fps,
        args.task,
        args.repo_id,
        args.min_duration,
        args.lerobot_format,
    )
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
