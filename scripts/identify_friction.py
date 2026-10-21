#!/usr/bin/python3
"""Joint friction identification from constant-speed forward/backward sweeps.

The sweep (openarm_gravity_calib --friction) hard-tracks one joint at a time at
0.05 / 0.15 / 0.30 rad/s in both directions with gravity-only feedforward. At the
same position, gravity and model error are the same in both directions, so

    (tau(+v) - tau(-v)) / 2 = Fc*tanh(k*v) + Fv*v

Fc/Fv are fit per joint with the same crossover used at runtime (k=8), then
derated by 0.85. A joint whose derated value would exceed its cap is left
unchanged. Accepted joints also get friction_k=8, so the zero-speed slope
matches the hand-tuned result and does not chatter. Masses, scale and bias stay.

Usage:
  ./scripts/identify_friction.py --collect --dry-run       # move arms, preview
  ./scripts/identify_friction.py recordings/gravity/friction_X.csv --write config/gravity_comp.yaml
"""

from __future__ import annotations

import argparse
import csv
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from lib.gravity_yaml_io import load_params, write_params  # noqa: E402

DEFAULT_YAML = ROOT / "config" / "gravity_comp.yaml"
DEFAULT_LOG_DIR = ROOT / "recordings" / "gravity"
RUN_CALIB = ROOT / "scripts" / "run_gravity_calib.sh"

DERATE = 0.85
# Gentle zero-crossing. k=20 applies most of Fc on encoder noise and chatters.
WRITE_K = 8.0
FC_CAP = [1.2, 1.2, 0.8, 0.8, 0.35, 0.35, 0.35]
FV_CAP = [0.08, 0.08, 0.05, 0.05, 0.03, 0.03, 0.03]
BIN_WIDTH = 0.02
MIN_BIN_SAMPLES = 3
MIN_BINS = 8
SIDES = ("left", "right")
GRIPPER_JOINT = 8
# DM4310. The measured plateau is several tenths of a Nm, above the old 0.08 default.
GRIPPER_FC_CAP = 1.0
GRIPPER_FV_CAP = 0.08
# Runtime crossover. The sweep is fit as a plateau, independent of this value.
# k=25 applies the full Coulomb torque on encoder noise and the gripper chatters.
GRIPPER_K = 8.0


def latest_friction_log(log_dir: Path = DEFAULT_LOG_DIR) -> Path | None:
    logs = sorted(log_dir.glob("friction_*.csv"), reverse=True)
    return logs[0] if logs else None


def load_log(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"{path.name} 没有采样")
    required = {"side", "joint", "speed", "dir", "q", "dq", "tau_g", "tau_cmd"}
    if not required.issubset(rows[0]):
        raise ValueError(f"{path.name} 不是摩擦扫描日志")
    data = {"side": np.array([row["side"] for row in rows])}
    for key in ("joint", "speed", "dir", "q", "dq", "tau_g", "tau_cmd", "mtau"):
        if key in rows[0]:
            data[key] = np.array([float(row[key]) for row in rows])
    return data


def speed_points(q, dq, direction, tau, tau_g, speed):
    """Per sweep speed: (|v|, median half-difference, bin spread, gravity residual, bins)."""
    points = []
    for nominal in sorted(set(np.round(speed, 4))):
        at = np.isclose(speed, nominal)
        bins = np.floor(q[at] / BIN_WIDTH).astype(int)
        halves, residuals = [], []
        for index in np.unique(bins):
            in_bin = bins == index
            forward = in_bin & (direction[at] > 0)
            backward = in_bin & (direction[at] < 0)
            if forward.sum() < MIN_BIN_SAMPLES or backward.sum() < MIN_BIN_SAMPLES:
                continue
            tau_f = np.median(tau[at][forward])
            tau_b = np.median(tau[at][backward])
            halves.append(0.5 * (tau_f - tau_b))
            residuals.append(0.5 * (tau_f + tau_b) - np.median(tau_g[at][in_bin]))
        if len(halves) < MIN_BINS:
            continue
        halves_arr = np.asarray(halves)
        q25, q75 = np.percentile(halves_arr, [25, 75])
        points.append(dict(
            v=float(np.median(np.abs(dq[at]))) if np.any(np.abs(dq[at]) > 0) else float(nominal),
            y=float(np.median(halves_arr)),
            spread=float(q75 - q25),
            residual=float(np.median(residuals)),
            bins=len(halves),
        ))
    return points


def fit_plateau(points):
    """Gripper speeds sit on the Coulomb plateau: y = Fc + Fv·v."""
    v = np.array([p["v"] for p in points])
    y = np.array([p["y"] for p in points])
    if len(v) < 2 or float(np.std(v)) < 1e-3:
        fc = float(max(np.mean(y), 0.0))
        return fc, 0.0, float(np.std(y))
    slope, intercept = np.polyfit(v, y, 1)
    fv = float(slope)
    fc = float(intercept)
    if fv < 0.0:
        fv = 0.0
        fc = float(np.mean(y))
    if fc < 0.0:
        fc = 0.0
    predicted = fc + fv * v
    rms = float(np.sqrt(np.mean((y - predicted) ** 2)))
    return fc, fv, rms


def fit_points(points, k):
    """Non-negative least squares for y = Fc*tanh(k*v) + Fv*v."""
    v = np.array([p["v"] for p in points])
    y = np.array([p["y"] for p in points])
    a = np.column_stack([np.tanh(k * v), v])
    best = None
    for active in ((0, 1), (0,), (1,)):
        coef = np.zeros(2)
        sol, *_ = np.linalg.lstsq(a[:, active], y, rcond=None)
        if np.any(sol < 0):
            continue
        coef[list(active)] = sol
        err = float(np.sqrt(np.mean((y - a @ coef) ** 2)))
        if best is None or err < best[2]:
            best = (float(coef[0]), float(coef[1]), err)
    return best if best is not None else (0.0, 0.0, float(np.sqrt(np.mean(y ** 2))))


def _fit_channel(data, params, side, joint, fc_cap, fv_cap, fc_key, fv_key, *, plateau=False):
    fc = float(params[fc_key]) if not isinstance(params[fc_key], list) else float(params[fc_key][joint - 1])
    fv = float(params[fv_key]) if not isinstance(params[fv_key], list) else float(params[fv_key][joint - 1])
    row = dict(side=side, joint=joint, accepted=False, fc_old=fc, fv_old=fv, fc_new=fc, fv_new=fv, reason="")
    mask = (data["side"] == side) & (data["joint"] == joint)
    if not np.any(mask):
        row["reason"] = "没有扫描数据"
        return row
    points = speed_points(data["q"][mask], data["dq"][mask], data["dir"][mask],
                          data["tau_cmd"][mask], data["tau_g"][mask], data["speed"][mask])
    row["points"] = points
    if len(points) < 2:
        row["reason"] = "正反向重叠位置不足"
        return row
    raw_fc, raw_fv, rms = fit_plateau(points) if plateau else fit_points(points, WRITE_K)
    mean_y = float(np.mean([abs(p["y"]) for p in points]))
    worst_spread = max(p["spread"] for p in points)
    scaled_fc, scaled_fv = DERATE * raw_fc, DERATE * raw_fv
    row.update(fc_raw=raw_fc, fv_raw=raw_fv, rms=rms, spread=worst_spread,
               residual=float(np.median([p["residual"] for p in points])))
    if worst_spread > max(0.10, 0.6 * mean_y):
        row["reason"] = f"同速度不同位置差异 {worst_spread:.3f} Nm 过大，疑似重力残差"
    elif rms > max(0.05, 0.25 * mean_y):
        row["reason"] = f"三档速度拟合残差 {rms:.3f} Nm 过大"
    elif scaled_fc > fc_cap or (not plateau and scaled_fv > fv_cap):
        row["reason"] = (f"拟合 Fc {scaled_fc:.3f} / Fv {scaled_fv:.4f} 超过上限 "
                         f"{fc_cap:.2f}/{fv_cap:.2f}，保留原值")
    else:
        # Two gripper speeds make the viscous slope noisy. Keep Fc and clip Fv.
        if scaled_fv > fv_cap:
            scaled_fv = fv_cap
        row.update(accepted=True, fc_new=scaled_fc, fv_new=scaled_fv)
    return row


def fit_gripper(data: dict[str, np.ndarray], params: dict) -> tuple[dict, list[dict]]:
    """One shared gripper parameter. Write the smaller accepted side."""
    reports = [_fit_channel(data, params, side, GRIPPER_JOINT, GRIPPER_FC_CAP, GRIPPER_FV_CAP,
                            "gripper_friction_fc", "gripper_friction_fv", plateau=True)
               for side in SIDES]
    accepted = [row for row in reports if row["accepted"]]
    updates: dict = {}
    if accepted:
        updates = {
            "gripper_friction_fc": min(row["fc_new"] for row in accepted),
            "gripper_friction_fv": min(row["fv_new"] for row in accepted),
            "gripper_friction_k": GRIPPER_K,
        }
        for row in reports:
            row["written_fc"] = updates["gripper_friction_fc"]
            row["written_fv"] = updates["gripper_friction_fv"]
    return updates, reports


def identify(data: dict[str, np.ndarray], params: dict) -> tuple[dict, list[dict]]:
    joints = {int(value) for value in data["joint"]}
    updates: dict = {}
    reports: list[dict] = []
    if GRIPPER_JOINT in joints:
        grip_updates, grip_reports = fit_gripper(data, params)
        updates.update(grip_updates)
        reports.extend(grip_reports)
    if not (joints & set(range(1, 8))):
        return updates, reports
    for side in SIDES:
        fc = [float(x) for x in params[f"{side}_friction_fc"]]
        fv = [float(x) for x in params[f"{side}_friction_fv"]]
        new_fc, new_fv = fc[:], fv[:]
        new_fk = [float(x) for x in params[f"{side}_friction_k"]]
        for joint in range(7):
            mask = (data["side"] == side) & (data["joint"] == joint + 1)
            row = dict(side=side, joint=joint + 1, accepted=False, fc_old=fc[joint], fv_old=fv[joint],
                       fc_new=fc[joint], fv_new=fv[joint], reason="")
            if not np.any(mask):
                row["reason"] = "没有扫描数据"
                reports.append(row)
                continue
            points = speed_points(data["q"][mask], data["dq"][mask], data["dir"][mask],
                                  data["tau_cmd"][mask], data["tau_g"][mask], data["speed"][mask])
            row["points"] = points
            if len(points) < 2:
                row["reason"] = "正反向重叠位置不足"
                reports.append(row)
                continue
            raw_fc, raw_fv, rms = fit_points(points, WRITE_K)
            mean_y = float(np.mean([abs(p["y"]) for p in points]))
            worst_spread = max(p["spread"] for p in points)
            scaled_fc, scaled_fv = DERATE * raw_fc, DERATE * raw_fv
            row.update(fc_raw=raw_fc, fv_raw=raw_fv, rms=rms, spread=worst_spread,
                       residual=float(np.median([p["residual"] for p in points])))
            if worst_spread > max(0.10, 0.6 * mean_y):
                row["reason"] = f"同速度不同位置差异 {worst_spread:.3f} Nm 过大，疑似重力残差"
            elif rms > max(0.05, 0.25 * mean_y):
                row["reason"] = f"三档速度拟合残差 {rms:.3f} Nm 过大"
            elif scaled_fc > FC_CAP[joint] or scaled_fv > FV_CAP[joint]:
                row["reason"] = (f"拟合 Fc {scaled_fc:.3f} / Fv {scaled_fv:.4f} 超过上限 "
                                 f"{FC_CAP[joint]:.2f}/{FV_CAP[joint]:.2f}，保留原值")
            else:
                new_fc[joint] = scaled_fc
                new_fv[joint] = scaled_fv
                new_fk[joint] = WRITE_K
                row.update(accepted=True, fc_new=new_fc[joint], fv_new=new_fv[joint])
            reports.append(row)
        updates[f"{side}_friction_fc"] = new_fc
        updates[f"{side}_friction_fv"] = new_fv
        updates[f"{side}_friction_k"] = new_fk
    return updates, reports


def format_report(reports: list[dict]) -> str:
    lines = [f"摩擦辨识：写入值 = 拟合 × {DERATE}；通过的关节换向陡度写成 {WRITE_K:.0f}；超过上限的关节保留"]
    for row in reports:
        name = "夹爪" if row["joint"] == GRIPPER_JOINT else f"J{row['joint']}"
        head = f"[{row['side']}] {name}: "
        if row["accepted"]:
            lines.append(
                head + f"Fc {row['fc_old']:.3f} → {row['fc_new']:.3f}  "
                f"Fv {row['fv_old']:.4f} → {row['fv_new']:.4f}  "
                f"(拟合 Fc {row['fc_raw']:.3f} Fv {row['fv_raw']:.4f}，残差 {row['rms']:.3f}，"
                f"正反平均偏离重力 {row['residual']:+.3f} Nm)")
        else:
            lines.append(head + f"保留 Fc {row['fc_old']:.3f} Fv {row['fv_old']:.4f}  [warn] {row['reason']}")
    written = next((row for row in reports if "written_fc" in row), None)
    if written is not None:
        lines.append(
            f"夹爪写入 Fc {written['written_fc']:.3f} Fv {written['written_fv']:.4f}"
            f"（通过的侧里取较小值），换向陡度 {GRIPPER_K:.0f}")
    return "\n".join(lines)


def write_atomic(target: Path, updates: dict) -> None:
    base = load_params(target)
    handle, temp = tempfile.mkstemp(prefix=".gravity_comp_", suffix=".yaml", dir=target.parent)
    os.close(handle)
    try:
        write_params(Path(temp), updates, base=base)
        os.replace(temp, target)
    finally:
        if os.path.exists(temp):
            os.unlink(temp)


def collect(*, gripper: bool = False) -> Path:
    flag = "--friction-gripper" if gripper else "--friction"
    print(f"+ bash scripts/run_gravity_calib.sh {flag}", flush=True)
    env = os.environ.copy()
    env.setdefault("OPENARM_ROOT", str(ROOT))
    log_path = None
    with subprocess.Popen(["bash", str(RUN_CALIB), flag], cwd=ROOT, env=env,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                          bufsize=1) as process:
        assert process.stdout is not None
        for line in process.stdout:
            sys.stdout.write(line)
            sys.stdout.flush()
            match = re.search(r"FRICTION_LOG_PATH=(\S+)", line)
            if match:
                log_path = Path(match.group(1))
        code = process.wait()
    if code != 0:
        raise RuntimeError(f"摩擦扫描未完成（退出码 {code}），不写入")
    if log_path is None or not log_path.is_file():
        raise RuntimeError("摩擦扫描没有生成日志")
    return log_path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("log", nargs="?", type=Path, help="friction_*.csv（缺省用最新）")
    parser.add_argument("--collect", action="store_true", help="先驱动扫描采数")
    parser.add_argument("--gripper", action="store_true", help="采夹爪摩擦，不扫手臂关节")
    parser.add_argument("--write", type=Path, default=Path(os.environ.get("OPENARM_GRAVITY_YAML", DEFAULT_YAML)))
    parser.add_argument("--dry-run", action="store_true", help="只预览，不写入")
    args = parser.parse_args(argv)
    try:
        log_path = collect(gripper=args.gripper) if args.collect else (args.log or latest_friction_log())
        if log_path is None:
            raise ValueError("没有摩擦扫描日志，请先运行摩擦辨识采数")
        target = args.write.resolve()
        if not target.is_file():
            raise ValueError(f"找不到重力配置 {target}")
        updates, reports = identify(load_log(log_path), load_params(target))
    except (OSError, ValueError, RuntimeError) as error:
        print(f"[error] {error}", file=sys.stderr)
        return 1
    print(f"FRICTION_LOG_PATH={log_path}")
    print(format_report(reports))
    accepted = sum(row["accepted"] for row in reports)
    if accepted == 0:
        print("[warn] 没有关节通过检查，配置未改动")
        return 0
    if args.dry_run:
        print(f"预览完成：{accepted} 项可更新，未写入")
        return 0
    write_atomic(target, updates)
    print(f"已写入 {target}：{accepted} 项摩擦更新，下次启动生效")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
