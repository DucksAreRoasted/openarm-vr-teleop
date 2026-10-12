#!/usr/bin/python3
"""Gravity refinement from calibration logs.

--runtime-fit / --post-id use the exact C++ zero-g model, preserve physics,
friction and gain, and accept bounded affine changes only after whole-pose
validation. New logs must contain tau_g and ff_gain. --dry-run previews changes.
Other switches retain the older experimental joint/friction fitting path.
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

DEFAULT_YAML = ROOT / "config" / "gravity_comp.yaml"
DEFAULT_LOG_DIR = ROOT / "recordings" / "gravity"
DEFAULT_URDF = ROOT / "assets" / "v1.urdf"
KP = np.array([50.0, 50.0, 50.0, 50.0, 10.0, 10.0, 10.0])
KP_ZERO_G = np.array([5.0, 5.0, 5.0, 5.0, 1.5, 1.5, 1.5])
# Must match openarm_gravity_calib KP_SOFT.
KP_HOLD = np.array([2.5, 2.5, 2.0, 2.0, 1.0, 1.0, 1.0])
KD_HOLD = np.array([0.35, 0.35, 0.3, 0.3, 0.15, 0.12, 0.15])

DEFAULT_FC = [0.35, 0.35, 0.20, 0.20, 0.08, 0.08, 0.06]
DEFAULT_FV = [0.02, 0.02, 0.015, 0.015, 0.006, 0.006, 0.005]
DEFAULT_FK = [20.0, 20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
# No hidden extra derating; the explicit runtime gain controls compensation.
SAFE_SCALE = 1.0
BIAS_LIM_PROX = 0.45
BIAS_LIM_DIST = 0.30


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


def fit_affine(
    x: np.ndarray, y: np.ndarray, *, scale_lo: float, scale_hi: float, bias_lim: float
) -> tuple[float, float, int]:
    """Fit y ≈ scale*x + bias, but prefer small bias (large bias causes home lift)."""
    if len(x) < 20:
        return 1.0, 0.0, len(x)
    # Scale-first through origin, then tiny residual bias.
    denom = float(np.dot(x, x))
    if denom < 1e-3 or float(np.std(x)) < 0.05:
        bias = float(np.clip(np.mean(y - x), -bias_lim, bias_lim)) if len(y) >= 20 else 0.0
        return 1.0, bias, len(x)
    scale = float(np.clip(np.dot(x, y) / denom, scale_lo, scale_hi))
    bias = float(np.clip(np.mean(y - scale * x), -bias_lim, bias_lim))
    return scale, bias, len(x)


def fit_friction(
    dq: np.ndarray, residual: np.ndarray, *, k: float, fc_hi: float, fv_hi: float
) -> tuple[float, float, int]:
    """Fit residual ≈ Fc·tanh(k·dq) + Fv·dq on moving samples."""
    if len(dq) < 40:
        return 0.0, 0.0, len(dq)
    moving = np.abs(dq) > 0.05
    if int(moving.sum()) < 30:
        return 0.0, 0.0, int(moving.sum())
    x1 = np.tanh(k * dq[moving])
    x2 = dq[moving]
    y = residual[moving]
    a = np.column_stack([x1, x2])
    coef, _, _, _ = np.linalg.lstsq(a, y, rcond=None)
    fc = float(np.clip(coef[0], 0.0, fc_hi))
    fv = float(np.clip(coef[1], 0.0, fv_hi))
    return fc, fv, int(moving.sum())


def recompute_g_from_yaml(
    data: dict[str, np.ndarray],
    side: str,
    urdf_path: str,
    yaml_path: Path,
) -> np.ndarray:
    """G_new(q) from identified mass/CoM in yaml — for residual fit after identify."""
    from lib.runtime_gravity_fit import runtime_gravity
    gravity = runtime_gravity(data, Path(urdf_path), yaml_path)
    gravity[data["_side"] != side] = 0.0
    return gravity


def tune_side(
    data: dict[str, np.ndarray],
    side: str,
    speed_max: float,
    *,
    droop: bool,
    scale_lo: float,
    scale_hi: float,
    bias_lim: float,
    g_override: np.ndarray | None = None,
) -> tuple[list[float], list[float], list[float], list[float], list[float]]:
    mask = data["_side"] == side
    if not np.any(mask):
        return [1.0] * 7, [0.0] * 7, DEFAULT_FC[:], DEFAULT_FV[:], DEFAULT_FK[:]
    speed = np.zeros(np.count_nonzero(mask))
    for index in range(1, 8):
        speed = np.maximum(speed, np.abs(data[f"dq{index}"][mask]))
        speed = np.maximum(speed, np.abs(data[f"dq_meas{index}"][mask]))
    quiet = speed < speed_max
    scales: list[float] = []
    biases: list[float] = []
    fcs: list[float] = []
    fvs: list[float] = []
    fks: list[float] = []
    mode = "post-id" if g_override is not None else ("droop" if droop else "legacy")
    print(f"\n[{side}] 准静态 {int(quiet.sum())}/{int(mask.sum())} (speed<{speed_max}, {mode})")
    for joint in range(1, 8):
        if g_override is not None:
            g = g_override[mask][quiet][:, joint - 1]
        else:
            g = data[f"g_model{joint}"][mask][quiet]
        mtau = data[f"mtau{joint}"][mask][quiet]
        tau_ff = data[f"tau_ff{joint}"][mask][quiet]
        err = data[f"e{joint}"][mask][quiet]
        if droop or g_override is not None:
            # τ* = τ_g + kp·e + kd·ė. τ_g = scale·g+bias. Friction stays out of this target.
            edot = data[f"dq{joint}"][mask][quiet] - data[f"dq_meas{joint}"][mask][quiet]
            pd = KP_HOLD[joint - 1] * err + KD_HOLD[joint - 1] * edot
            if f"tau_g{joint}" in data:
                target = data[f"tau_g{joint}"][mask][quiet] + pd
                method = f"τ*=τ_g+{KP_HOLD[joint-1]:.1f}*e+{KD_HOLD[joint-1]:.2f}*ė"
            elif f"g_model{joint}" in data:
                target = data[f"g_model{joint}"][mask][quiet] + pd
                method = f"τ*=g_model+{KP_HOLD[joint-1]:.1f}*e+{KD_HOLD[joint-1]:.2f}*ė"
            else:
                target = tau_ff + pd
                method = f"droop τff+{KP_HOLD[joint-1]:.1f}*e+{KD_HOLD[joint-1]:.2f}*ė"
        else:
            use_motor = np.std(mtau) > 0.05 and np.mean(np.abs(mtau)) > 0.05
            soft = float(np.mean(np.abs(err))) < 0.02
            kp = KP_ZERO_G[joint - 1] if soft else KP[joint - 1]
            target = mtau if use_motor else (tau_ff + kp * err)
            method = "电机力矩" if use_motor else ("soft*e+tau_ff" if soft else "kp*e+tau_ff")

        # j1 侧平举主导：优先用 |q1| 大的样本，避免矢状面近零激励把 scale 拉回 1
        if joint == 1:
            q1 = data[f"q_meas{joint}"][mask][quiet]
            lat = np.abs(q1) > 0.55
            if int(lat.sum()) >= 40:
                g = g[lat]
                target = target[lat]
                err = err[lat]
                method += f"+lat(|q1|>0.55,n={int(lat.sum())})"
            else:
                print(
                    f"  j1: 侧平举样本不足 n_lat={int(lat.sum())}，用全部准静态点"
                )

        scale, bias, count = fit_affine(
            g, target, scale_lo=scale_lo, scale_hi=scale_hi, bias_lim=bias_lim
        )
        # Prefer under-compensation: derate scale and tighten proximal bias.
        scale = float(np.clip(scale * SAFE_SCALE, scale_lo, scale_hi))
        bias_cap = BIAS_LIM_PROX if joint <= 4 else BIAS_LIM_DIST
        if g_override is not None:
            # Post-ID: keep corrections mild — mass already carries most of the model.
            bias_cap = min(bias_cap, 0.28 if joint <= 4 else 0.18)
        bias = float(np.clip(bias, -bias_cap, bias_cap))
        rms_e = float(np.sqrt(np.mean(err**2))) if len(err) else 0.0
        mean_e = float(np.mean(err)) if len(err) else 0.0
        resid_rms = float(np.sqrt(np.mean((target - (scale * g + bias)) ** 2))) if count else 0.0
        print(
            f"  j{joint}: scale={scale:.3f} bias={bias:+.3f} n={count} "
            f"via {method}×{SAFE_SCALE:.2f}  rms_e={rms_e:.4f} mean_e={mean_e:+.4f} "
            f"resid={resid_rms:.4f}"
        )
        scales.append(scale)
        biases.append(bias)

        # Friction from all samples of this side (after removing new gravity).
        if g_override is not None:
            g_all = g_override[mask][:, joint - 1]
        else:
            g_all = data[f"g_model{joint}"][mask]
        dq_all = data[f"dq_meas{joint}"][mask]
        if droop or g_override is not None:
            edot_all = data[f"dq{joint}"][mask] - data[f"dq_meas{joint}"][mask]
            pd_all = KP_HOLD[joint - 1] * data[f"e{joint}"][mask] + KD_HOLD[joint - 1] * edot_all
            if f"tau_g{joint}" in data:
                resid = data[f"tau_g{joint}"][mask] + pd_all
            elif f"g_model{joint}" in data:
                resid = data[f"g_model{joint}"][mask] + pd_all
            else:
                resid = data[f"tau_ff{joint}"][mask] + pd_all
            resid = resid - (scale * g_all + bias)
        else:
            resid = data[f"mtau{joint}"][mask] - (scale * g_all + bias)
        fk = DEFAULT_FK[joint - 1]
        fc_hi = 1.2 if joint <= 2 else (0.8 if joint <= 4 else 0.35)
        fv_hi = 0.08 if joint <= 2 else (0.05 if joint <= 4 else 0.03)
        fc, fv, n_fr = fit_friction(dq_all, resid, k=fk, fc_hi=fc_hi, fv_hi=fv_hi)
        if n_fr < 30:
            fc, fv = DEFAULT_FC[joint - 1], DEFAULT_FV[joint - 1]
            print(f"       friction: keep default Fc={fc:.3f} Fv={fv:.3f} (n_move={n_fr})")
        else:
            # Cap friction; large Fv at low speed contributes to drift/lift feel.
            fc = float(np.clip(fc, 0.0, fc_hi))
            fv = float(np.clip(fv, 0.0, fv_hi))
            print(f"       friction: Fc={fc:.3f} Fv={fv:.3f} k={fk:.0f} n_move={n_fr}")
        fcs.append(fc)
        fvs.append(fv)
        fks.append(fk)
    return scales, biases, fcs, fvs, fks


def fold_scale_into_urdf(
    left_s: list[float], right_s: list[float], urdf_scale: float
) -> tuple[list[float], list[float], float]:
    """If joint scales sit high (clipped under-mass), fold median into urdf_gravity_scale."""
    pooled = np.array(left_s + right_s, dtype=np.float64)
    proximal = np.array(left_s[:4] + right_s[:4], dtype=np.float64)
    med = float(np.median(proximal if len(proximal) else pooled))
    if med < 1.08:
        print(f"\n[fold] 近端 median scale={med:.3f}，不折入 urdf_scale")
        return left_s, right_s, urdf_scale
    factor = float(np.clip(med, 1.0, 1.6))
    new_urdf = float(np.clip(urdf_scale * factor, 0.5, 2.5))
    left_out = [float(np.clip(s / factor, 0.75, 1.35)) for s in left_s]
    right_out = [float(np.clip(s / factor, 0.75, 1.35)) for s in right_s]
    print(
        f"\n[fold] 近端 median scale={med:.3f} → urdf_gravity_scale "
        f"{urdf_scale:.3f}×{factor:.3f}={new_urdf:.3f}，关节 scale 回缩"
    )
    return left_out, right_out, new_urdf


def read_model_scalars(path: Path) -> tuple[float, float, float]:
    urdf_scale, ee_payload, ff_gain = 1.0, 0.0, 0.90
    if not path.is_file():
        return urdf_scale, ee_payload, ff_gain
    for line in path.read_text().splitlines():
        line = line.strip()
        if line.startswith("urdf_gravity_scale:"):
            urdf_scale = float(line.split(":", 1)[1].strip())
        elif line.startswith("ee_payload_kg:"):
            ee_payload = float(line.split(":", 1)[1].strip())
        elif line.startswith("ff_gain:") or line.startswith("g_scale:"):
            ff_gain = float(line.split(":", 1)[1].strip())
    return urdf_scale, ee_payload, ff_gain


def _parse_array7(line: str) -> list[float] | None:
    raw = line.split(":", 1)[1].strip().strip("[]")
    values = [float(x.strip()) for x in raw.split(",") if x.strip()]
    if len(values) < 7:
        return None
    return values[:7]


def read_existing_yaml(path: Path) -> dict[str, list[float]]:
    out = {
        "left_s": [1.0] * 7,
        "left_b": [0.0] * 7,
        "right_s": [1.0] * 7,
        "right_b": [0.0] * 7,
        "left_fc": DEFAULT_FC[:],
        "left_fv": DEFAULT_FV[:],
        "left_fk": DEFAULT_FK[:],
        "right_fc": DEFAULT_FC[:],
        "right_fv": DEFAULT_FV[:],
        "right_fk": DEFAULT_FK[:],
        "left_link_mass_scale": [1.0] * 7,
        "right_link_mass_scale": [1.0] * 7,
    }
    if not path.is_file():
        return out
    key_map = {
        "left_gravity_scale:": "left_s",
        "left_gravity_bias:": "left_b",
        "right_gravity_scale:": "right_s",
        "right_gravity_bias:": "right_b",
        "left_friction_fc:": "left_fc",
        "left_friction_fv:": "left_fv",
        "left_friction_k:": "left_fk",
        "right_friction_fc:": "right_fc",
        "right_friction_fv:": "right_fv",
        "right_friction_k:": "right_fk",
        "left_link_mass_scale:": "left_link_mass_scale",
        "right_link_mass_scale:": "right_link_mass_scale",
    }
    for line in path.read_text().splitlines():
        line = line.strip()
        for prefix, dest in key_map.items():
            if line.startswith(prefix):
                parsed = _parse_array7(line)
                if parsed is not None:
                    out[dest] = parsed
    return out


def write_yaml(
    path: Path,
    left_s,
    left_b,
    right_s,
    right_b,
    *,
    left_fc,
    left_fv,
    left_fk,
    right_fc,
    right_fv,
    right_fk,
    urdf_scale: float,
    ee_payload: float,
    ff_gain: float = 0.90,
    left_link_mass_scale: list[float] | None = None,
    right_link_mass_scale: list[float] | None = None,
) -> None:
    ones = [1.0] * 7
    updates = {
        "urdf_gravity_scale": float(urdf_scale),
        "ee_payload_kg": float(ee_payload),
        "g_scale": float(ff_gain),
        "ff_gain": float(ff_gain),
        "left_link_mass_scale": list(left_link_mass_scale or ones),
        "right_link_mass_scale": list(right_link_mass_scale or ones),
        "left_gravity_scale": list(left_s),
        "left_gravity_bias": list(left_b),
        "left_friction_fc": list(left_fc),
        "left_friction_fv": list(left_fv),
        "left_friction_k": list(left_fk),
        "right_gravity_scale": list(right_s),
        "right_gravity_bias": list(right_b),
        "right_friction_fc": list(right_fc),
        "right_friction_fv": list(right_fv),
        "right_friction_k": list(right_fk),
    }
    try:
        write_params(path, updates)
    except PermissionError as error:
        raise SystemExit(
            f"写不进 {path}（权限不够）。请执行：\n  chmod a+rw {path}\n原始错误：{error}"
        ) from error


def pick_best_log(log_dir: Path) -> Path:
    files = sorted(log_dir.glob("gravity_*.csv"))
    if not files:
        raise SystemExit(f"在 {log_dir} 没找到 gravity_*.csv，先跑零重力。")
    return max(files[-8:], key=lambda path: path.stat().st_size)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", help="gravity_*.csv；默认选最近较大的一份")
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--write", type=Path, default=DEFAULT_YAML, help="写出 gravity_comp.yaml")
    parser.add_argument("--speed-max", type=float, default=0.15, help="准静态速度阈值 rad/s")
    parser.add_argument(
        "--side",
        choices=("both", "left", "right"),
        default="both",
        help="只更新某一侧，另一侧保留 yaml 原值",
    )
    parser.add_argument(
        "--full",
        action="store_true",
        help="放开 scale 到 0.5~2.0（不推荐，优先 --droop --fold-scale）",
    )
    parser.add_argument(
        "--droop",
        action="store_true",
        help="用软持下垂：τ*=τ_ff+kp_hold·e（对齐自动标定）",
    )
    parser.add_argument(
        "--fold-scale",
        action="store_true",
        help="把偏高的关节 scale 折进 urdf_gravity_scale",
    )
    parser.add_argument(
        "--friction-only",
        action="store_true",
        help="只更新摩擦；保留 yaml 里的重力 scale/bias 与质量辨识结果",
    )
    parser.add_argument(
        "--post-id",
        action="store_true",
        help="惯量辨识后的二次精修：用 yaml 质量重算 G(q)，再拟合关节 scale/bias+摩擦",
    )
    parser.add_argument("--runtime-fit", action="store_true",
                        help="保留现有物理/摩擦参数，按实际 KDL 模型拟合并验证关节补偿")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--dry-run", action="store_true", help="只打印，不写文件")
    args = parser.parse_args()

    if args.runtime_fit or args.post_id:
        from lib.runtime_gravity_fit import refine_file
        log_path = Path(args.log) if args.log else pick_best_log(args.log_dir)
        refine_file(load_csv(log_path), args.write, args.urdf, args.speed_max, args.dry_run, args.side)
        return

    if args.droop:
        scale_lo, scale_hi, bias_lim = 0.70, 1.50, 0.45
    elif args.full:
        scale_lo, scale_hi, bias_lim = 0.50, 2.00, 1.0
    else:
        scale_lo, scale_hi, bias_lim = 0.85, 1.25, 0.45

    log_path = Path(args.log) if args.log else pick_best_log(args.log_dir)
    print(f"读取 {log_path}")
    data = load_csv(log_path)
    existing = read_existing_yaml(args.write)
    left_s, left_b = existing["left_s"], existing["left_b"]
    right_s, right_b = existing["right_s"], existing["right_b"]
    left_fc, left_fv, left_fk = existing["left_fc"], existing["left_fv"], existing["left_fk"]
    right_fc, right_fv, right_fk = existing["right_fc"], existing["right_fv"], existing["right_fk"]
    urdf_scale, ee_payload, ff_gain = read_model_scalars(args.write)
    ff_gain = float(np.clip(ff_gain if ff_gain > 0 else 0.90, 0.80, 1.10))
    g_left = g_right = None

    if args.side in ("both", "left"):
        new_ls, new_lb, left_fc, left_fv, left_fk = tune_side(
            data,
            "left",
            args.speed_max,
            droop=args.droop or args.friction_only or args.post_id,
            scale_lo=scale_lo,
            scale_hi=scale_hi,
            bias_lim=bias_lim,
            g_override=g_left,
        )
        if not args.friction_only:
            left_s, left_b = new_ls, new_lb
    if args.side in ("both", "right"):
        new_rs, new_rb, right_fc, right_fv, right_fk = tune_side(
            data,
            "right",
            args.speed_max,
            droop=args.droop or args.friction_only or args.post_id,
            scale_lo=scale_lo,
            scale_hi=scale_hi,
            bias_lim=bias_lim,
            g_override=g_right,
        )
        if not args.friction_only:
            right_s, right_b = new_rs, new_rb
    if args.fold_scale and not args.friction_only and not args.post_id:
        left_s, right_s, urdf_scale = fold_scale_into_urdf(left_s, right_s, urdf_scale)
    if args.dry_run:
        print("\n(--dry-run，未写入)")
        return
    write_yaml(
        args.write,
        left_s,
        left_b,
        right_s,
        right_b,
        left_fc=left_fc,
        left_fv=left_fv,
        left_fk=left_fk,
        right_fc=right_fc,
        right_fv=right_fv,
        right_fk=right_fk,
        urdf_scale=urdf_scale,
        ee_payload=ee_payload,
        ff_gain=ff_gain,
        left_link_mass_scale=existing["left_link_mass_scale"],
        right_link_mass_scale=existing["right_link_mass_scale"],
    )
    print(
        f"\n已合并写入 {args.write}（side={args.side}, droop={args.droop}, "
        f"post_id={args.post_id}, fold={args.fold_scale}, friction_only={args.friction_only}）"
    )
    print(
        f"模型：urdf_gravity_scale={urdf_scale} ee_payload_kg={ee_payload} "
        f"ff_gain={ff_gain} + friction Fc/Fv"
    )
    print("零重力运行中点精调会热加载；否则重启生效。")


if __name__ == "__main__":
    main()
