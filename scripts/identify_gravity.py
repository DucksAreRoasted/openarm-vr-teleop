#!/usr/bin/python3
"""Quasi-static inertial identification (Pinocchio static regressor when available).

Fits link1..link7 (m, mx, my, mz) + EE point mass from soft-hold calib logs:

  τ* = g_model + kp_hold·e  ≈ Y_static(q) · φ

Default: distal→proximal (j7→j1) when log has traj bands 1xx/2xx/3xx.
Writes into gravity_comp.yaml (merge, does not wipe camera/distal knobs):
  left/right_link_mass_scale
  left/right_link_com_delta   (21 = 7×3 meters)
  ee_payload_kg
  resets joint affine scale/bias to identity (masses carry the model)

Usage:
  ./scripts/identify_gravity.py recordings/gravity/gravity_XXXX.csv
  ./scripts/identify_gravity.py --log-dir recordings/gravity
  ./scripts/identify_gravity.py --batch   # force whole-arm LS
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from lib.gravity_yaml_io import load_params, write_params  # noqa: E402
from lib.pinocchio_gravity_id import (  # noqa: E402
    FINGER_MASS,
    com_delta_to_list,
    has_staged_traj,
    identify_side,
    try_import_pinocchio,
)

DEFAULT_YAML = ROOT / "config" / "gravity_comp.yaml"
DEFAULT_LOG_DIR = ROOT / "recordings" / "gravity"
DEFAULT_URDF = ROOT / "assets" / "v1.urdf"


def load_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise SystemExit(f"空日志：{path}")
    skip = {"side"}
    keys = [key for key in rows[0].keys() if key not in skip]
    data = {key: np.array([float(row[key]) for row in rows], dtype=np.float64) for key in keys}
    data["_side"] = np.array([row["side"] for row in rows])
    return data


def pick_log(log_dir: Path) -> Path:
    files = sorted(log_dir.glob("gravity_*.csv"))
    # Prefer motion-calib logs (not pinocchio_zg_*)
    files = [p for p in files if p.name.startswith("gravity_")]
    if not files:
        raise SystemExit(f"{log_dir} 无 gravity_*.csv")
    return max(files[-8:], key=lambda p: p.stat().st_size)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", help="gravity_*.csv")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--write", type=Path, default=DEFAULT_YAML)
    parser.add_argument("--speed-max", type=float, default=0.12)
    parser.add_argument("--max-samples", type=int, default=300)
    parser.add_argument("--g-scale", type=float, default=0.92, help="写出后的运行时 g_scale")
    parser.add_argument("--skip-motion", action="store_true", help="使用 log-dir 里最近较大日志")
    parser.add_argument(
        "--batch",
        action="store_true",
        help="强制整臂一次最小二乘（默认：有 1xx/2xx/3xx traj 时 j7→j1 分阶段）",
    )
    parser.add_argument(
        "--sequential",
        action="store_true",
        help="强制分阶段辨识（日志无 traj 带时会回退整批）",
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    pin = try_import_pinocchio()
    print(
        "辨识后端：Pinocchio"
        if pin is not None
        else "辨识后端：解析静力学回归器（未找到 pinocchio；可 apt install ros-humble-pinocchio）"
    )

    if args.log:
        log_path = Path(args.log)
    else:
        log_path = pick_log(args.log_dir)
    _ = args.skip_motion  # documented alias; pick_log already used when log omitted
    print(f"读取 {log_path}")
    data = load_csv(log_path)
    urdf = str(args.urdf)

    if args.batch and args.sequential:
        raise SystemExit("不能同时 --batch 与 --sequential")
    if args.batch:
        sequential: bool | None = False
    elif args.sequential:
        sequential = True
    else:
        sequential = None  # auto: staged traj → j7→j1
    mode = (
        "j7→j1 分阶段"
        if (sequential is True or (sequential is None and has_staged_traj(data)))
        else "整批 LS"
    )
    print(f"辨识模式：{mode}")

    left = identify_side(
        data,
        "left",
        urdf,
        speed_max=args.speed_max,
        max_samples=args.max_samples,
        sequential=sequential,
    )
    right = identify_side(
        data,
        "right",
        urdf,
        speed_max=args.speed_max,
        max_samples=args.max_samples,
        sequential=sequential,
    )

    ee_payload = float(
        np.clip(0.5 * (left.ee_mass + right.ee_mass) - FINGER_MASS, 0.0, 0.30)
    )

    def dump(tag: str, r) -> None:
        print(f"\n[{tag}] backend={r.backend} n={r.n} rms={r.rms:.4f} Nm")
        for i, s in enumerate(r.mass_scales, start=1):
            d = r.com_delta[i - 1]
            print(
                f"  link{i}: scale={s:.3f}  m={s * r.m0[i-1]:.3f} kg  "
                f"Δcom=[{d[0]:+.4f},{d[1]:+.4f},{d[2]:+.4f}] m"
            )
        print(f"  ee_mass≈{r.ee_mass:.4f} kg")

    dump("left", left)
    dump("right", right)
    print(f"\nee_payload_kg (beyond fingers) ≈ {ee_payload:.4f}")

    if args.dry_run:
        print("(--dry-run，未写入)")
        return

    updates = {
        "urdf_gravity_scale": 1.0,
        "ee_payload_kg": ee_payload,
        "g_scale": float(args.g_scale),
        "ff_gain": float(args.g_scale),
        "left_link_mass_scale": [float(x) for x in left.mass_scales],
        "right_link_mass_scale": [float(x) for x in right.mass_scales],
        "left_link_com_delta": com_delta_to_list(left.com_delta),
        "right_link_com_delta": com_delta_to_list(right.com_delta),
        # Mass/CoM carry the model — clear joint affine so they don't double-count.
        "left_gravity_scale": [1.0] * 7,
        "right_gravity_scale": [1.0] * 7,
        "left_gravity_bias": [0.0] * 7,
        "right_gravity_bias": [0.0] * 7,
    }
    write_params(args.write, updates)
    print(f"\n已合并写入 {args.write}（保留相机/远端增益等原有字段）")
    print("零重力 / 镜像示教 / MoveIt 会读 link_mass_scale + link_com_delta。")
    _ = load_params(args.write)


if __name__ == "__main__":
    main()
