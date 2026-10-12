#!/usr/bin/env python3
"""Turn Quest teleop segments into a LeRobot v3 dataset.

Prefer ``openarm_dataset_export.py`` (writes OpenArm Dataset v0.4, then converts).
This script remains the offline / no-``openarm_dataset`` fallback from CSV.

The control loop writes ``<segment>/lerobot_episode.csv`` while the record
panel is running (measured joint positions as observation.state, MIT position
commands as action). This script packs every segment in a batch into the
layout used by OpenArm's ``openarm-dataset-convert --format lerobot_v3.0``:

    <batch>/lerobot/
      meta/info.json
      meta/stats.json
      meta/tasks.parquet
      meta/episodes/chunk-000/file-000.parquet
      data/chunk-000/file-000.parquet

State is 16-D qpos, left arm then right arm, each ``joint1..joint7`` plus
``gripper``. That is the OpenArm bimanual vector (``robot_type=openarm_bimanual``).
Color from the three Gemini 335 cameras is stored as
``observation.images.{head,wrist_left,wrist_right}`` when those frames were
captured. When the ``lerobot`` package is installed, episodes are written with
``LeRobotDataset`` instead of the local parquet writer.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

FPS_DEFAULT = 30
ROBOT_TYPE = "openarm_bimanual"
CODEBASE_VERSION = "v3.0"
CHUNK_SIZE = 1000
STATE_NAMES = [
    f"{side}_{joint}.pos"
    for side in ("left", "right")
    for joint in (
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "joint7",
        "gripper",
    )
]
STATE_DIM = len(STATE_NAMES)
CAMERA_NAMES = ("head", "wrist_left", "wrist_right")
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480


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
        return empty, np.zeros((0, STATE_DIM), np.float32), np.zeros((0, STATE_DIM), np.float32), empty
    width = max(len(row) for row in rows)
    data = np.zeros((len(rows), width), dtype=np.float64)
    for i, row in enumerate(rows):
        data[i, : len(row)] = row
    times = data[:, 0]
    left_obs = data[:, 1:9]
    left_act = data[:, 9:17]
    right_obs = data[:, 17:25]
    right_act = data[:, 25:33]
    unix = data[:, 33] if width > 33 else np.array([])
    obs = np.concatenate([left_obs, right_obs], axis=1).astype(np.float32)
    act = np.concatenate([left_act, right_act], axis=1).astype(np.float32)
    return times, obs, act, unix


def _nearest_index(times: np.ndarray, targets: np.ndarray) -> np.ndarray:
    index = np.searchsorted(times, targets, side="left")
    index = np.clip(index, 0, len(times) - 1)
    previous = np.clip(index - 1, 0, len(times) - 1)
    choose_prev = np.abs(times[previous] - targets) <= np.abs(times[index] - targets)
    return np.where(choose_prev, previous, index)


def _resample(times, obs, act, unix, fps: int):
    if len(times) == 0:
        return obs, act, unix
    duration = float(times[-1] - times[0])
    count = int(np.floor(duration * fps)) + 1
    if count < 2:
        return obs[:1], act[:1], unix[:1] if len(unix) else unix
    targets = times[0] + np.arange(count, dtype=np.float64) / float(fps)
    index = _nearest_index(times, targets)
    unix_sel = unix[index] if len(unix) == len(times) else np.array([])
    return obs[index], act[index], unix_sel


def discover_episodes(batch: Path) -> list[Path]:
    files = sorted(batch.glob("*/lerobot_episode.csv"))
    if not files and (batch / "lerobot_episode.csv").is_file():
        files = [batch / "lerobot_episode.csv"]
    return files


def load_episodes(batch: Path, fps: int, min_duration: float):
    episodes = []
    for path in discover_episodes(batch):
        times, obs, act, unix = _load_episode_csv(path)
        if len(times) == 0:
            print(f"[lerobot] skip empty {path}", file=sys.stderr)
            continue
        duration = float(times[-1] - times[0])
        if duration < min_duration:
            print(
                f"[lerobot] skip {path.parent.name}: {duration:.2f}s < {min_duration:.2f}s",
                file=sys.stderr,
            )
            continue
        outcome_path = path.parent / "outcome.txt"
        if not outcome_path.is_file():
            print(f"[lerobot] skip unlabeled {path.parent.name}", file=sys.stderr)
            continue
        outcome = outcome_path.read_text(encoding="utf-8").strip().lower()
        if outcome in {"success", "1", "true"}:
            success = 1
        elif outcome in {"failure", "fail", "0", "false"}:
            success = 0
        else:
            print(f"[lerobot] skip {path.parent.name}: unknown outcome {outcome!r}", file=sys.stderr)
            continue
        obs_r, act_r, unix_r = _resample(times, obs, act, unix, fps)
        episodes.append((obs_r, act_r, unix_r, path.parent / "cameras", success))
        label = "success" if success else "failure"
        print(f"[lerobot] {path.parent.name}: {len(obs_r)} frames @ {fps} Hz ({duration:.2f}s, {label})")
    return episodes


def _camera_times(folder: Path) -> tuple[np.ndarray, list[Path]]:
    times_path = folder / "times.csv"
    frames = sorted(folder.glob("*.jpg"))
    if not times_path.is_file() or not frames:
        return np.array([]), []
    unix = []
    with times_path.open(encoding="utf-8") as handle:
        next(handle, None)
        for line in handle:
            parts = line.strip().split(",")
            if len(parts) >= 2:
                unix.append(float(parts[1]))
    count = min(len(unix), len(frames))
    return np.asarray(unix[:count], dtype=np.float64), frames[:count]


def _aligned_frames(camera_dir: Path, name: str, unix: np.ndarray) -> list[Path]:
    folder = camera_dir / name
    times, frames = _camera_times(folder)
    if len(times) == 0 or len(unix) == 0:
        return []
    index = _nearest_index(times, unix)
    return [frames[int(i)] for i in index]


def _ffmpeg_bin() -> str | None:
    override = os.environ.get("OPENARM_FFMPEG", "").strip()
    if override and Path(override).is_file():
        return override
    found = shutil.which("ffmpeg")
    if found:
        return found
    roots = [Path.home() / "miniconda3" / "envs", Path.home() / "anaconda3" / "envs"]
    preferred = []
    for root in roots:
        preferred.extend(sorted(root.glob("lerobot/bin/ffmpeg")))
        preferred.extend(sorted(root.glob("*/bin/ffmpeg")))
    for path in preferred:
        if path.is_file():
            return str(path)
    return None


def _encode_video(frames: list[Path], out_path: Path, fps: int, work: Path) -> bool:
    ffmpeg = _ffmpeg_bin()
    if ffmpeg is None or not frames:
        return False
    work.mkdir(parents=True, exist_ok=True)
    for index, src in enumerate(frames):
        dest = work / f"{index:06d}.jpg"
        if dest.exists() or dest.is_symlink():
            dest.unlink()
        dest.symlink_to(src.resolve())
    out_path.parent.mkdir(parents=True, exist_ok=True)
    command = [
        ffmpeg,
        "-y",
        "-loglevel",
        "error",
        "-framerate",
        str(fps),
        "-i",
        str(work / "%06d.jpg"),
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        str(out_path),
    ]
    result = subprocess.run(command, check=False)
    return result.returncode == 0 and out_path.is_file()


def _read_rgb(path: Path) -> np.ndarray | None:
    try:
        import cv2

        bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if bgr is not None:
            return bgr[:, :, ::-1]
    except ImportError:
        pass
    try:
        from PIL import Image

        return np.asarray(Image.open(path).convert("RGB"))
    except Exception:
        pass
    ffmpeg = _ffmpeg_bin()
    if ffmpeg is None:
        return None
    result = subprocess.run(
        [
            ffmpeg,
            "-loglevel",
            "error",
            "-i",
            str(path),
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-vframes",
            "1",
            "pipe:1",
        ],
        check=False,
        capture_output=True,
    )
    expected = VIDEO_WIDTH * VIDEO_HEIGHT * 3
    if result.returncode != 0 or len(result.stdout) != expected:
        return None
    return np.frombuffer(result.stdout, dtype=np.uint8).reshape(VIDEO_HEIGHT, VIDEO_WIDTH, 3)


def _image_stats(frames: list[Path]) -> dict | None:
    if not frames:
        return None
    sample = frames[:: max(1, len(frames) // 16)][:16]
    pixels = []
    for path in sample:
        image = _read_rgb(path)
        if image is None:
            continue
        pixels.append(image.reshape(-1, 3).astype(np.float64))
    if not pixels:
        return None
    data = np.concatenate(pixels, axis=0)
    minimum = data.min(axis=0) / 255.0
    maximum = data.max(axis=0) / 255.0
    mean = data.mean(axis=0) / 255.0
    std = data.std(axis=0) / 255.0

    def nest(values):
        return [[[float(v)]] for v in values]

    return {
        "min": nest(minimum),
        "max": nest(maximum),
        "mean": nest(mean),
        "std": nest(std),
        "count": [len(sample)],
    }


def _describe(values: np.ndarray) -> dict:
    data = np.asarray(values, dtype=np.float64)
    if data.ndim == 1:
        data = data.reshape(-1, 1)
    return {
        "min": data.min(axis=0),
        "max": data.max(axis=0),
        "mean": data.mean(axis=0),
        "std": data.std(axis=0),
        "count": np.array([data.shape[0]], dtype=np.int64),
    }


def _write_with_lerobot(out: Path, episodes, fps: int, task: str, repo_id: str) -> bool:
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError:
        return False
    if out.exists() and any(out.iterdir()):
        # Official writer creates a new dataset. Rebuild into a fresh directory
        # by removing the previous export; source CSVs stay in the batch.
        shutil.rmtree(out)
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        robot_type=ROBOT_TYPE,
        root=str(out),
        use_videos=False,
        features={
            "observation.state": {
                "dtype": "float32",
                "shape": (STATE_DIM,),
                "names": STATE_NAMES,
            },
            "action": {
                "dtype": "float32",
                "shape": (STATE_DIM,),
                "names": STATE_NAMES,
            },
        },
    )
    for episode in episodes:
        obs, act = episode[0], episode[1]
        for row_obs, row_act in zip(obs, act):
            dataset.add_frame(
                {
                    "observation.state": row_obs.astype(np.float32),
                    "action": row_act.astype(np.float32),
                    "task": task,
                }
            )
        dataset.save_episode()
    dataset.finalize()
    return True


def _json_stats(stats: dict) -> dict:
    out = {}
    for key, value in stats.items():
        out[key] = {
            name: (item.tolist() if isinstance(item, np.ndarray) else item)
            for name, item in value.items()
        }
    return out


def _write_pyarrow(out: Path, episodes, fps: int, task: str, repo_id: str) -> None:
    import pandas as pd

    if out.exists():
        shutil.rmtree(out)
    frames = []
    episode_rows = []
    stats_rows = []
    global_index = 0
    video_rows: list[dict] = []
    for episode_index, episode in enumerate(episodes):
        obs, act = episode[0], episode[1]
        success_flag = int(episode[4]) if len(episode) > 4 else 1
        count = len(obs)
        timestamps = np.arange(count, dtype=np.float64) / float(fps)
        frame_index = np.arange(count, dtype=np.int64)
        index = np.arange(global_index, global_index + count, dtype=np.int64)
        frames.append(
            pd.DataFrame(
                {
                    "action": list(act.astype(np.float32)),
                    "observation.state": list(obs.astype(np.float32)),
                    "timestamp": timestamps,
                    "frame_index": frame_index,
                    "episode_index": np.full(count, episode_index, dtype=np.int64),
                    "index": index,
                    "task_index": np.zeros(count, dtype=np.int64),
                    "success": np.full(count, success_flag, dtype=np.int64),
                    "last_frame_index": np.full(count, count - 1, dtype=np.int64),
                }
            )
        )
        ep_stats = {
            "action": _describe(act),
            "observation.state": _describe(obs),
            "timestamp": _describe(timestamps),
            "frame_index": _describe(frame_index.astype(np.float64)),
            "episode_index": _describe(np.full(count, episode_index, dtype=np.float64)),
            "index": _describe(index.astype(np.float64)),
            "task_index": _describe(np.zeros(count, dtype=np.float64)),
        }
        row = {
            "episode_index": episode_index,
            "tasks": [task],
            "length": count,
            "data/chunk_index": 0,
            "data/file_index": 0,
            "dataset_from_index": global_index,
            "dataset_to_index": global_index + count,
            "meta/episodes/chunk_index": 0,
            "meta/episodes/file_index": 0,
        }
        flat = {}
        for feature, desc in ep_stats.items():
            for stat_name, value in desc.items():
                flat[f"stats/{feature}/{stat_name}"] = value.tolist()
        row.update(flat)
        episode_rows.append(row)
        video_rows.append({})
        stats_rows.append(ep_stats)
        global_index += count

    present_cameras = []
    for name in CAMERA_NAMES:
        sequences = []
        ok = True
        for obs, _act, unix, camera_dir, *_rest in episodes:
            aligned = _aligned_frames(camera_dir, name, unix)
            if len(aligned) != len(obs):
                ok = False
                break
            sequences.append(aligned)
        if not ok:
            if any((episode[3] / name).is_dir() for episode in episodes):
                print(f"[lerobot] camera {name} not aligned; video omitted", file=sys.stderr)
            continue
        present_cameras.append((name, sequences))

    work = out / ".video_work"
    image_stats = {}
    for name, sequences in present_cameras:
        key = f"observation.images.{name}"
        packed: list[Path] = []
        offset = 0
        for row, camera_frames in zip(video_rows, sequences):
            count = len(camera_frames)
            row[f"videos/{key}/chunk_index"] = 0
            row[f"videos/{key}/file_index"] = 0
            row[f"videos/{key}/from_timestamp"] = offset / float(fps)
            row[f"videos/{key}/to_timestamp"] = (offset + count) / float(fps)
            packed.extend(camera_frames)
            offset += count
        video_path = out / "videos" / key / "chunk-000" / "file-000.mp4"
        if not _encode_video(packed, video_path, fps, work / name):
            print(f"[lerobot] ffmpeg failed for {name}; video omitted", file=sys.stderr)
            for row in video_rows:
                for suffix in ("chunk_index", "file_index", "from_timestamp", "to_timestamp"):
                    row.pop(f"videos/{key}/{suffix}", None)
            continue
        stats = _image_stats(packed)
        if stats is not None:
            image_stats[key] = stats
        print(f"[lerobot] video {key}: {len(packed)} frames")
    if work.exists():
        shutil.rmtree(work, ignore_errors=True)
    for row, extra in zip(episode_rows, video_rows):
        row.update(extra)

    data_path = out / "data/chunk-000/file-000.parquet"
    data_path.parent.mkdir(parents=True, exist_ok=True)
    pd.concat(frames, ignore_index=True).to_parquet(data_path, index=False)

    episodes_path = out / "meta/episodes/chunk-000/file-000.parquet"
    episodes_path.parent.mkdir(parents=True, exist_ok=True)
    episode_frame = pd.DataFrame(episode_rows)
    for column in episode_frame.columns:
        if str(column).endswith("/chunk_index") or str(column).endswith("/file_index"):
            episode_frame[column] = episode_frame[column].astype("int64")
    episode_frame.to_parquet(episodes_path, index=False)

    tasks = pd.DataFrame({"task_index": [0]}, index=pd.Index([task], name="task"))
    tasks_path = out / "meta/tasks.parquet"
    tasks_path.parent.mkdir(parents=True, exist_ok=True)
    tasks.to_parquet(tasks_path)

    overall = {}
    for feature in stats_rows[0]:
        per_ep = [row[feature] for row in stats_rows]
        counts = np.stack([item["count"] for item in per_ep]).astype(np.float64)
        means = np.stack([item["mean"] for item in per_ep])
        variances = np.stack([item["std"] for item in per_ep]) ** 2
        total = counts.sum(axis=0)
        while counts.ndim < means.ndim:
            counts = np.expand_dims(counts, axis=-1)
        total_mean = (means * counts).sum(axis=0) / total
        delta = means - total_mean
        total_var = ((variances + delta**2) * counts).sum(axis=0) / total
        overall[feature] = {
            "min": np.min(np.stack([item["min"] for item in per_ep]), axis=0),
            "max": np.max(np.stack([item["max"] for item in per_ep]), axis=0),
            "mean": total_mean,
            "std": np.sqrt(total_var),
            "count": total.reshape(-1),
        }
    meta = out / "meta"
    meta.mkdir(parents=True, exist_ok=True)
    stats_payload = _json_stats(overall)
    stats_payload.update(image_stats)
    (meta / "stats.json").write_text(
        json.dumps(stats_payload, indent=4),
        encoding="utf-8",
    )
    num_episodes = len(episodes)
    train_end = max(1, int(round(num_episodes * 0.8))) if num_episodes else 0
    splits = {"train": f"0:{train_end}"}
    if train_end < num_episodes:
        splits["val"] = f"{train_end}:{num_episodes}"
    info = {
        "codebase_version": CODEBASE_VERSION,
        "robot_type": ROBOT_TYPE,
        "total_episodes": num_episodes,
        "total_frames": global_index,
        "total_tasks": 1,
        "chunks_size": CHUNK_SIZE,
        "data_files_size_in_mb": 100,
        "video_files_size_in_mb": 200,
        "fps": fps,
        "splits": splits,
        "data_path": "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet",
        "video_path": "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4",
        "features": {
            "action": {
                "dtype": "float32",
                "names": STATE_NAMES,
                "shape": [STATE_DIM],
                "fps": fps,
            },
            "observation.state": {
                "dtype": "float32",
                "names": STATE_NAMES,
                "shape": [STATE_DIM],
                "fps": fps,
            },
            "timestamp": {"dtype": "float64", "shape": [1], "names": None, "fps": fps},
            "frame_index": {"dtype": "int64", "shape": [1], "names": None, "fps": fps},
            "episode_index": {"dtype": "int64", "shape": [1], "names": None, "fps": fps},
            "index": {"dtype": "int64", "shape": [1], "names": None, "fps": fps},
            "task_index": {"dtype": "int64", "shape": [1], "names": None, "fps": fps},
            "success": {"dtype": "int64", "shape": [1], "names": None, "fps": fps},
            "last_frame_index": {"dtype": "int64", "shape": [1], "names": None, "fps": fps},
        },
        "repo_id": repo_id,
    }
    for name, _sequences in present_cameras:
        key = f"observation.images.{name}"
        if key not in image_stats and not (out / "videos" / key / "chunk-000" / "file-000.mp4").is_file():
            continue
        info["features"][key] = {
            "dtype": "video",
            "shape": [VIDEO_HEIGHT, VIDEO_WIDTH, 3],
            "names": ["height", "width", "channels"],
            "fps": fps,
            "info": {
                "video.height": VIDEO_HEIGHT,
                "video.width": VIDEO_WIDTH,
                "video.codec": "h264",
                "video.pix_fmt": "yuv420p",
                "video.is_depth_map": False,
                "video.fps": fps,
                "video.channels": 3,
                "has_audio": False,
            },
        }
    (meta / "info.json").write_text(json.dumps(info, indent=4), encoding="utf-8")


def export_batch(batch: Path, fps: int, task: str, repo_id: str, min_duration: float) -> Path | None:
    episodes = load_episodes(batch, fps, min_duration)
    if not episodes:
        out = batch / "lerobot"
        if out.exists():
            shutil.rmtree(out)
        print(f"[lerobot] no labeled episodes under {batch}", file=sys.stderr)
        return out
    out = batch / "lerobot"
    has_cameras = any((episode[3] / name).is_dir() for episode in episodes for name in CAMERA_NAMES)
    if not has_cameras and _write_with_lerobot(out, episodes, fps, task, repo_id):
        print(f"[lerobot] wrote {out} via LeRobotDataset ({len(episodes)} episodes)")
    else:
        _write_pyarrow(out, episodes, fps, task, repo_id)
        print(f"[lerobot] wrote {out} ({len(episodes)} episodes, pyarrow v3)")
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description="Export Quest teleop segments to LeRobot v3")
    parser.add_argument("--batch", type=Path, required=True, help="recordings/sessions/<name>")
    parser.add_argument("--task", default="bimanual teleoperation")
    parser.add_argument("--fps", type=int, default=FPS_DEFAULT)
    parser.add_argument("--repo-id", default="local/openarm_vr_teleop")
    parser.add_argument("--min-duration", type=float, default=1.0)
    args = parser.parse_args()
    if args.fps <= 0:
        print("[lerobot] fps must be positive", file=sys.stderr)
        return 2
    if not args.batch.is_dir():
        print(f"[lerobot] batch not found: {args.batch}", file=sys.stderr)
        return 2
    result = export_batch(args.batch, args.fps, args.task, args.repo_id, args.min_duration)
    return 0 if result is not None else 1


if __name__ == "__main__":
    raise SystemExit(main())
