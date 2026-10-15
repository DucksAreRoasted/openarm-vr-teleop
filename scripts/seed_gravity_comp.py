#!/usr/bin/python3
"""Seed gravity_comp.yaml from URDF via Pinocchio (CAD initial G(q)).

Does NOT need the robot. Loads assets/v1.urdf, computes nominal gravity at the
calib home pose with Pinocchio (or analytical fallback), then writes identity
corrections so runtime uses pure CAD:

  link_mass_scale = 1, link_com_delta = 0, gravity_scale/bias = 1/0
  urdf_gravity_scale = 1  (CAD is the prior; ID later absorbs mass error)

Camera / distal / friction knobs already in yaml are preserved (merge write).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from lib.gravity_yaml_io import load_params, write_params  # noqa: E402
from lib.pinocchio_gravity_id import (  # noqa: E402
    FINGER_MASS,
    nominal_gravity_from_urdf,
    try_import_pinocchio,
)

DEFAULT_YAML = ROOT / "config" / "gravity_comp.yaml"
DEFAULT_URDF = ROOT / "assets" / "v1.urdf"


def _fmt_tau(g: np.ndarray) -> str:
    return " ".join(f"{v:+.3f}" for v in g)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", type=Path, default=DEFAULT_YAML)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument(
        "--urdf-scale",
        type=float,
        default=1.0,
        help="CAD 全局倍率；纯 URDF 初值用 1.0，已知偏轻时可 >1",
    )
    parser.add_argument(
        "--ee-payload",
        type=float,
        default=None,
        help="额外末端质量 (kg)，默认保留 YAML 中的值",
    )
    parser.add_argument(
        "--g-scale",
        type=float,
        default=0.95,
        help="运行时前馈增益（略 <1 避免上抬）",
    )
    parser.add_argument(
        "--reset-friction",
        action="store_true",
        help="同时把摩擦重置为保守默认（默认保留原有摩擦）",
    )
    parser.add_argument("--dry-run", action="store_true", help="只打印 URDF 初值，不写 yaml")
    args = parser.parse_args()

    if args.ee_payload is None:
        args.ee_payload = float(load_params(args.write).get("ee_payload_kg", 0.0))

    if not args.urdf.is_file():
        raise SystemExit(f"URDF 不存在：{args.urdf}")

    pin = try_import_pinocchio()
    print(
        "URDF 初值后端：Pinocchio"
        if pin is not None
        else "URDF 初值后端：解析模型（未找到 pinocchio；可 apt install ros-humble-pinocchio）"
    )
    print(f"URDF：{args.urdf}")

    params = load_params(args.write)
    for side in ("left", "right"):
        present = params.get(f"{side}_gripper_present", True)
        ee_mass = (FINGER_MASS + max(0.0, float(params.get("gripper_motor_kg", 0.12))) +
                   max(0.0, float(args.ee_payload))) if present else 0.0
        g, backend, info = nominal_gravity_from_urdf(
            str(args.urdf), side, ee_mass=ee_mass
        )
        g_scaled = float(args.urdf_scale) * g
        print(f"\n[{side}] backend={backend}  q_home={np.array2string(info['q'], precision=2)}")
        print(f"  CAD masses link1..7 = {np.array2string(info['masses_link1_7'], precision=3)}")
        print(f"  G_urdf(q)     = [{_fmt_tau(g)}] Nm")
        if abs(args.urdf_scale - 1.0) > 1e-9:
            print(f"  ×urdf_scale   = [{_fmt_tau(g_scaled)}] Nm  (scale={args.urdf_scale})")
        print(f"  gripper={'installed' if present else 'removed'} ee_mass = {ee_mass:.4f} kg")

    if args.dry_run:
        print("\n(--dry-run，未写入 yaml)")
        return

    # Identity corrections: runtime G(q) = urdf_scale * G_cad(q) (+ distal point masses).
    updates = {
        "urdf_gravity_scale": float(args.urdf_scale),
        "ee_payload_kg": float(args.ee_payload),
        "g_scale": float(args.g_scale),
        "ff_gain": float(args.g_scale),
        "link_mass_scale": [1.0] * 7,
        "left_link_mass_scale": [1.0] * 7,
        "right_link_mass_scale": [1.0] * 7,
        "left_link_com_delta": [0.0] * 21,
        "right_link_com_delta": [0.0] * 21,
        "left_gravity_scale": [1.0] * 7,
        "right_gravity_scale": [1.0] * 7,
        "left_gravity_bias": [0.0] * 7,
        "right_gravity_bias": [0.0] * 7,
    }
    if args.reset_friction:
        updates.update(
            {
                "left_friction_fc": [0.35, 0.35, 0.20, 0.20, 0.08, 0.08, 0.06],
                "left_friction_fv": [0.02, 0.02, 0.015, 0.015, 0.006, 0.006, 0.005],
                "left_friction_k": [20.0, 20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
                "right_friction_fc": [0.35, 0.35, 0.20, 0.20, 0.08, 0.08, 0.06],
                "right_friction_fv": [0.02, 0.02, 0.015, 0.015, 0.006, 0.006, 0.005],
                "right_friction_k": [20.0, 20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
            }
        )
    write_params(args.write, updates)
    print(
        f"\n已写入 URDF 初值 → {args.write}\n"
        f"  纯 CAD：mass_scale=1, com_delta=0, joint scale/bias=1/0\n"
        f"  urdf_gravity_scale={args.urdf_scale} ee_payload={args.ee_payload} "
        f"g_scale={args.g_scale}\n"
        f"  相机/远端增益等原有字段已保留。真机偏差再用「惯量标定」修正。"
    )


if __name__ == "__main__":
    main()
