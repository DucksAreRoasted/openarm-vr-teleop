"""Filesystem helpers for teleop session / episode management."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Literal

import yaml

from .services import ROOT

SESSIONS_ROOT = ROOT / "recordings" / "sessions"
EXPORT_SCRIPT = ROOT / "script" / "openarm_dataset_export.py"
CAMERA_NAMES = ("head", "wrist_left", "wrist_right")
Outcome = Literal["success", "failure", "unlabeled"]


@dataclass
class EpisodeInfo:
    id: str  # folder name, e.g. "01"
    path: Path
    outcome: Outcome
    frames: int
    duration_s: float
    cameras_ok: int  # how many of CAMERA_NAMES have frames
    in_export: bool


@dataclass
class BatchInfo:
    name: str
    path: Path
    episodes: int
    success: int
    failure: int
    unlabeled: int
    size_bytes: int
    has_openarm: bool
    has_lerobot: bool


def ensure_sessions_root() -> Path:
    SESSIONS_ROOT.mkdir(parents=True, exist_ok=True)
    return SESSIONS_ROOT


def batch_path(name: str) -> Path:
    name = name.strip()
    if not name or "/" in name or name in {".", ".."}:
        raise ValueError(f"invalid batch name: {name!r}")
    return SESSIONS_ROOT / name


def _dir_size(path: Path) -> int:
    total = 0
    if not path.is_dir():
        return 0
    for item in path.rglob("*"):
        if item.is_file():
            try:
                total += item.stat().st_size
            except OSError:
                pass
    return total


def _read_outcome(path: Path) -> Outcome:
    outcome_path = path / "outcome.txt"
    if not outcome_path.is_file():
        return "unlabeled"
    text = outcome_path.read_text(encoding="utf-8").strip().lower()
    if text in {"success", "1", "true"}:
        return "success"
    if text in {"failure", "fail", "0", "false"}:
        return "failure"
    return "unlabeled"


def _csv_stats(path: Path) -> tuple[int, float]:
    csv_path = path / "lerobot_episode.csv"
    if not csv_path.is_file():
        return 0, 0.0
    frames = 0
    t0 = t1 = None
    with csv_path.open(encoding="utf-8") as handle:
        next(handle, None)
        for line in handle:
            line = line.strip()
            if not line:
                continue
            parts = line.split(",")
            if not parts:
                continue
            try:
                t = float(parts[0])
            except ValueError:
                continue
            if t0 is None:
                t0 = t
            t1 = t
            frames += 1
    if frames == 0 or t0 is None or t1 is None:
        return frames, 0.0
    return frames, max(0.0, float(t1 - t0))


def _camera_count(path: Path) -> int:
    root = path / "cameras"
    if not root.is_dir():
        return 0
    count = 0
    for name in CAMERA_NAMES:
        folder = root / name
        if folder.is_dir() and (
            any(folder.glob("*.jpg")) or any(folder.glob("*.jpeg"))
        ):
            count += 1
    return count


def _exported_ids(batch: Path) -> set[str]:
    meta = batch / "openarm_dataset" / "metadata.yaml"
    if not meta.is_file():
        return set()
    try:
        data = yaml.safe_load(meta.read_text(encoding="utf-8")) or {}
    except Exception:
        return set()
    ids: set[str] = set()
    for ep in data.get("episodes") or []:
        eid = str(ep.get("id", "")).strip()
        if eid:
            ids.add(eid)
            if eid.isdigit():
                ids.add(f"{int(eid):02d}")
                ids.add(str(int(eid)))
    return ids


def _is_episode_dir(path: Path) -> bool:
    if not path.is_dir():
        return False
    name = path.name
    if name in {"openarm_dataset", "lerobot"}:
        return False
    if (path / "lerobot_episode.csv").is_file():
        return True
    if name.isdigit():
        return True
    return False


def list_episode_dirs(batch: Path) -> list[Path]:
    if not batch.is_dir():
        return []
    dirs = [p for p in batch.iterdir() if _is_episode_dir(p)]

    def sort_key(p: Path):
        if p.name.isdigit():
            return (0, int(p.name))
        return (1, p.name)

    return sorted(dirs, key=sort_key)


def list_episodes(batch_name: str) -> list[EpisodeInfo]:
    batch = batch_path(batch_name)
    exported = _exported_ids(batch)
    out: list[EpisodeInfo] = []
    for path in list_episode_dirs(batch):
        frames, duration = _csv_stats(path)
        eid = path.name
        in_export = eid in exported or (
            eid.isdigit() and (str(int(eid)) in exported or f"{int(eid):02d}" in exported)
        )
        out.append(
            EpisodeInfo(
                id=eid,
                path=path,
                outcome=_read_outcome(path),
                frames=frames,
                duration_s=duration,
                cameras_ok=_camera_count(path),
                in_export=in_export,
            )
        )
    return out


def list_batches() -> list[BatchInfo]:
    ensure_sessions_root()
    batches: list[BatchInfo] = []
    for path in sorted(SESSIONS_ROOT.iterdir(), key=lambda p: p.name, reverse=True):
        if not path.is_dir():
            continue
        episodes = list_episodes(path.name)
        batches.append(
            BatchInfo(
                name=path.name,
                path=path,
                episodes=len(episodes),
                success=sum(1 for e in episodes if e.outcome == "success"),
                failure=sum(1 for e in episodes if e.outcome == "failure"),
                unlabeled=sum(1 for e in episodes if e.outcome == "unlabeled"),
                size_bytes=_dir_size(path),
                has_openarm=(path / "openarm_dataset" / "metadata.yaml").is_file(),
                has_lerobot=(path / "lerobot" / "meta" / "info.json").is_file()
                or (path / "lerobot").is_dir(),
            )
        )
    return batches


def episode_dir(batch_name: str, episode_id: str) -> Path:
    batch = batch_path(batch_name)
    eid = episode_id.strip()
    candidates = []
    if eid.isdigit():
        candidates.append(batch / f"{int(eid):02d}")
        candidates.append(batch / str(int(eid)))
    candidates.append(batch / eid)
    for path in candidates:
        if path.is_dir():
            return path
    raise FileNotFoundError(f"episode {episode_id!r} not found in {batch_name}")


def delete_episode(batch_name: str, episode_id: str, *, export: bool = True) -> None:
    path = episode_dir(batch_name, episode_id)
    shutil.rmtree(path)
    if export:
        export_batch(batch_name)


def delete_batch(batch_name: str) -> None:
    path = batch_path(batch_name)
    if not path.is_dir():
        raise FileNotFoundError(path)
    shutil.rmtree(path)


def set_outcome(
    batch_name: str,
    episode_id: str,
    outcome: Literal["success", "failure"],
    *,
    export: bool = True,
) -> None:
    path = episode_dir(batch_name, episode_id)
    (path / "outcome.txt").write_text(outcome + "\n", encoding="utf-8")
    if export:
        export_batch(batch_name)


def export_batch(
    batch_name: str,
    *,
    task: str | None = None,
    fps: int = 30,
    timeout_s: float = 600.0,
) -> subprocess.CompletedProcess[str]:
    batch = batch_path(batch_name)
    if not batch.is_dir():
        raise FileNotFoundError(batch)
    if not EXPORT_SCRIPT.is_file():
        raise FileNotFoundError(EXPORT_SCRIPT)
    py = sys.executable
    cmd = [
        py,
        str(EXPORT_SCRIPT),
        "--batch",
        str(batch),
        "--fps",
        str(fps),
    ]
    if task:
        cmd.extend(["--task", task])
    return subprocess.run(
        cmd,
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout_s,
        cwd=str(ROOT),
    )


def format_size(n: int) -> str:
    value = float(n)
    for unit in ("B", "KB", "MB", "GB"):
        if value < 1024 or unit == "GB":
            if unit == "B":
                return f"{int(value)} {unit}"
            return f"{value:.1f} {unit}"
        value /= 1024.0
    return f"{n} B"


def open_in_file_manager(path: Path) -> None:
    path = path if path.exists() else SESSIONS_ROOT
    subprocess.Popen(
        ["xdg-open", str(path)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
