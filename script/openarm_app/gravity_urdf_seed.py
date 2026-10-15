"""Generate and install the URDF gravity baseline without opening hardware."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import stat
import subprocess
import sys
import tempfile

from .gravity_knobs import load_params
from .services import ROOT


def install_urdf_gravity_seed(yaml_path: Path, urdf_path: Path) -> str:
    """Validate a staged seed, then replace the current YAML atomically."""
    yaml_path = yaml_path.resolve()
    urdf_path = urdf_path.resolve()
    if not yaml_path.is_file():
        raise FileNotFoundError(f"找不到当前重力配置：{yaml_path}")
    if not urdf_path.is_file():
        raise FileNotFoundError(f"找不到 URDF：{urdf_path}")

    with tempfile.TemporaryDirectory(prefix=".gravity_urdf_", dir=yaml_path.parent) as folder:
        staged = Path(folder) / yaml_path.name
        shutil.copy2(yaml_path, staged)
        result = subprocess.run(
            [sys.executable, str(ROOT / "scripts" / "seed_gravity_comp.py"),
             "--urdf", str(urdf_path), "--write", str(staged)],
            cwd=ROOT,
            capture_output=True,
            text=True,
            timeout=30,
            check=False,
        )
        if result.returncode != 0:
            raise RuntimeError((result.stderr or result.stdout).strip() or "URDF 初值计算失败")
        params = load_params(staged)
        for name, expected in (
            ("link_mass_scale", [1.0] * 7),
            ("left_link_mass_scale", [1.0] * 7),
            ("right_link_mass_scale", [1.0] * 7),
            ("left_link_com_delta", [0.0] * 21),
            ("right_link_com_delta", [0.0] * 21),
            ("left_gravity_scale", [1.0] * 7),
            ("right_gravity_scale", [1.0] * 7),
            ("left_gravity_bias", [0.0] * 7),
            ("right_gravity_bias", [0.0] * 7),
        ):
            if params.get(name) != expected:
                raise RuntimeError(f"生成的配置校验失败：{name}")
        if params.get("urdf_gravity_scale") != 1.0:
            raise RuntimeError("生成的 URDF 全局倍率不是 1")

        staged.chmod(stat.S_IMODE(yaml_path.stat().st_mode))
        os.replace(staged, yaml_path)
        return result.stdout.strip()
