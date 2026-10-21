"""Console helpers for the two-sided inertial identification (scripts/identify_gravity.py)."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys

from .services import ROOT

LOG_DIR = ROOT / "recordings" / "gravity"
SCRIPT = ROOT / "scripts" / "identify_gravity.py"
_SUMMARY_MARKERS = ("[left]", "[right]", "当前配置", "URDF 原值", "本次辨识", "link", "[warn]")


def latest_two_sided_log(log_dir: Path = LOG_DIR) -> Path | None:
    """Newest calibration log recorded with +δ/−δ approaches."""
    for path in sorted(log_dir.glob("gravity_*.csv"), reverse=True):
        try:
            with path.open() as handle:
                header = handle.readline().strip().split(",")
        except OSError:
            continue
        if {"approach", "tau_g1", "ff_gain"}.issubset(header):
            return path
    return None


def run_identification(log_path: Path, yaml_path: Path, write: bool) -> tuple[bool, str]:
    command = [sys.executable, str(SCRIPT), str(log_path), "--write", str(yaml_path)]
    if not write:
        command.append("--dry-run")
    result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=300, check=False)
    output = (result.stdout + result.stderr).strip()
    return result.returncode == 0, output or f"惯量标定退出码 {result.returncode}"


def summary(output: str) -> str:
    return "\n".join(line.strip() for line in output.splitlines()
                     if line.strip().startswith(_SUMMARY_MARKERS))


FRICTION_SCRIPT = ROOT / "scripts" / "identify_friction.py"


def friction_collect_command(yaml_path: Path, *, gripper: bool = False) -> list[str]:
    """Move the arms or the grippers, then preview the fit without writing."""
    command = [sys.executable, "-u", str(FRICTION_SCRIPT), "--collect", "--dry-run", "--write", str(yaml_path)]
    if gripper:
        command.append("--gripper")
    return command


def run_friction_write(log_path: Path, yaml_path: Path) -> tuple[bool, str]:
    command = [sys.executable, str(FRICTION_SCRIPT), str(log_path), "--write", str(yaml_path)]
    result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=120, check=False)
    output = (result.stdout + result.stderr).strip()
    return result.returncode == 0, output or f"摩擦辨识退出码 {result.returncode}"


def friction_log_path(output: str) -> Path | None:
    for line in reversed(output.splitlines()):
        if line.startswith("FRICTION_LOG_PATH="):
            return Path(line.split("=", 1)[1].strip())
    return None


def friction_summary(output: str) -> str:
    return "\n".join(line.strip() for line in output.splitlines()
                     if line.strip().startswith(("[left]", "[right]", "[warn] 没有", "夹爪写入")))
