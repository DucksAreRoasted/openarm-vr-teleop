"""Conservative affine refinement against the actual zero-g KDL model.

Uses held-out whole poses, preserves physical parameters/friction, and updates
only joints whose independent validation error improves.

Two-sided logs (``approach`` = +1 / −1) hold each pose after releasing it from
goal+δ and goal−δ. Static friction opposes the final motion, so the two holds
bracket gravity: the mean is the target and half the difference estimates
friction. Poses that have only one side in such a log are dropped, because a
one-sided hold is biased by friction.
"""
from __future__ import annotations

import subprocess
from pathlib import Path
import numpy as np

from lib.gravity_yaml_io import load_params, write_params
from lib.calibration_coverage import validation_mask, require_vertical_pairs, VERTICAL_IDS
from lib.pinocchio_gravity_id import hold_pd_torque

ROOT = Path(__file__).resolve().parents[2]
# The calibrator offsets the left goal and mirrors it, so on the right arm's
# j1/j2/j5–j7 the logged approach sign is opposite to the joint's direction.
APPROACH_JOINT_SIGN = {"left": np.ones(7), "right": np.array([-1, -1, 1, 1, -1, -1, -1.0])}


def evaluate_gravity(sides, q: np.ndarray, urdf: Path, yaml_path: Path) -> np.ndarray:
    """Runtime G(q) (before scale/bias) for each (side, q) row."""
    binary = ROOT / "build" / "openarm_gravity_model"
    # Build even when present, so source changes cannot leave a stale evaluator.
    subprocess.run(["cmake", "--build", str(ROOT / "build"), "--target",
                    "openarm_gravity_model", "-j2"], check=True, stdout=subprocess.DEVNULL)
    rows = [str(side) + "," + ",".join(format(float(x), ".17g") for x in angles)
            for side, angles in zip(sides, q)]
    result = subprocess.run([str(binary), str(urdf), str(yaml_path)],
                            input="\n".join(rows) + "\n", text=True,
                            capture_output=True, check=True)
    gravity = np.array([[float(x) for x in line.split(",")]
                        for line in result.stdout.splitlines()])
    if gravity.shape != (len(rows), 7) or not np.isfinite(gravity).all():
        raise ValueError("Invalid runtime gravity output")
    return gravity


def runtime_gravity(data: dict, urdf: Path, yaml_path: Path) -> np.ndarray:
    q = np.column_stack([data[f"q_meas{j}"] for j in range(1, 8)])
    return evaluate_gravity(data["_side"], q, urdf, yaml_path)


def check_calibration_log(data: dict) -> None:
    if not all(f"tau_g{j}" in data for j in range(1, 8)):
        raise ValueError("日志缺少 tau_g，请重新采集标定数据；旧零重力日志不可用于自动精修")
    if "ff_gain" not in data:
        raise ValueError("日志未记录标定前馈增益 ff_gain，请用新版标定器重新采数")


def collect_holds(data: dict, side: str, speed_max: float = 0.05) -> dict[tuple, dict[int, np.ndarray]]:
    """Quiet soft-hold rows per pose, keyed by pose and approach sign."""
    idx = np.flatnonzero(data["_side"] == side)
    if len(idx) < 60:
        raise ValueError(f"{side}: 样本不足")
    if not np.isfinite(data["ff_gain"][idx]).all():
        raise ValueError("Invalid logged gain")
    # tau_g is always the actual gravity command recorded during collection.
    q = np.column_stack([data[f"q_meas{j}"][idx] for j in range(1, 8)])
    commands = np.column_stack([data[f"q_cmd{j}"][idx] for j in range(1, 8)])
    velocity = np.column_stack([data[f"dq{j}"][idx] for j in range(1, 8)])
    # Position-derived velocity avoids relying exclusively on noisy motor dq.
    t = data["t"][idx]
    dt = np.diff(t)
    speed = np.full(len(idx), np.inf)
    good_dt = (dt > 1e-4) & (dt < 0.1)
    speed[1:] = np.where(good_dt, np.max(np.abs(np.diff(q, axis=0)) /
                         np.maximum(dt[:, None], 1e-6), axis=1), np.inf)
    quiet = (speed < min(speed_max, 0.05)) & (np.max(np.abs(velocity), axis=1) < 1e-6)
    # Do not infer gravity from samples whose feedforward was clipped.
    if all(f"tau_ff{j}" in data for j in range(1, 8)):
        tau_ff = np.column_stack([data[f"tau_ff{j}"][idx] for j in range(1, 8)])
        quiet &= np.all(np.abs(tau_ff) < np.array([20,20,7,7,2,2,2])-0.001, axis=1)
    approach = (np.sign(data["approach"][idx]).astype(int) if "approach" in data
                else np.zeros(len(idx), dtype=int))
    # A whole held pose is the validation unit, never random adjacent samples.
    boundaries = np.r_[True, (np.max(np.abs(np.diff(commands, axis=0)), axis=1) > 1e-4)
                       | (np.diff(approach) != 0)]
    groups = np.cumsum(boundaries) - 1
    valid_groups = [group for group in np.unique(groups)
                    if np.count_nonzero(quiet & (groups == group)) >= 15]
    # Pair the +δ and −δ holds of one pose by waypoint id and commanded pose.
    holds: dict[tuple, dict[int, np.ndarray]] = {}
    for group in valid_groups:
        local = quiet & (groups == group)
        selected = idx[local]
        sign = int(approach[local][0])
        # Single-sided holds stay separate poses, exactly as before.
        key = (int(data["traj"][selected[0]]),
               tuple(np.round(np.median(commands[local], axis=0), 3)) if sign else int(group))
        holds.setdefault(key, {})[sign] = selected
    return holds


def hold_target(data: dict, rows: np.ndarray) -> np.ndarray:
    """Median physical gravity torque of one hold; medians stop long dwells dominating."""
    return np.median(np.array([
        np.array([data[f"tau_g{j}"][r] for j in range(1, 8)]) + hold_pd_torque(data, r)
        for r in rows]), axis=0)


def fit_runtime(data: dict, gravity: np.ndarray, params: dict,
                speed_max: float = 0.05, sides: tuple[str, ...] = ("left", "right")) -> tuple[dict, list[dict]]:
    check_calibration_log(data)
    if (np.isin(data["traj"], list(VERTICAL_IDS)).any() and
            "approach" in data and np.any(data["approach"] != 0)):
        require_vertical_pairs(data, sides)
    logged = np.column_stack([data[f"g_model{j}"] for j in range(1, 8)])
    if not np.allclose(gravity, logged, rtol=2e-4, atol=0.002):
        raise ValueError("当前物理模型与采数时不一致，拒绝混用旧日志")
    updates, reports = {}, []
    gain = float(params.get("g_scale", params.get("ff_gain", 0.90)))
    if not 0.2 <= gain <= 1.5:
        raise ValueError("Invalid zero-g gain")
    for side in sides:
        holds = {key: {sign: (np.median(gravity[rows], axis=0), hold_target(data, rows))
                       for sign, rows in sides_held.items()}
                 for key, sides_held in collect_holds(data, side, speed_max).items()}
        two_sided = any(sign != 0 for sides_held in holds.values() for sign in sides_held)
        scales = np.array(params[side + "_gravity_scale"], dtype=float)
        biases = np.array(params[side + "_gravity_bias"], dtype=float)
        gx, targets, traj_ids, frictions = [], [], [], []
        dropped = 0
        for key, sides_held in holds.items():
            if two_sided:
                if 1 not in sides_held or -1 not in sides_held:
                    dropped += 1
                    continue
                (g_plus, t_plus), (g_minus, t_minus) = sides_held[1], sides_held[-1]
                gx.append(0.5 * (g_plus + g_minus))
                targets.append(0.5 * (t_plus + t_minus))
                # A sticky joint stops near its release point, so the two holds sit at
                # different q; remove the model's gravity change before differencing.
                frictions.append(0.5 * ((t_minus - gain * (scales * g_minus + biases))
                                        - (t_plus - gain * (scales * g_plus + biases))))
            else:
                g_one, t_one = next(iter(sides_held.values()))
                gx.append(g_one)
                targets.append(t_one)
            traj_ids.append(key[0])
        stage_ids = [traj // 100 for traj in traj_ids]
        if not {1, 2, 3}.issubset(stage_ids) or len(gx) < 9:
            raise ValueError(f"{side}: 缺少完整腕/肘/肩停稳姿态，拒绝覆盖配置"
                             + (f"（{dropped} 个姿态只停稳了一侧）" if dropped else ""))
        x, target = np.asarray(gx), np.asarray(targets)
        if not np.isfinite(x).all() or not np.isfinite(target).all():
            raise ValueError("Non-finite calibration data")
        friction = (APPROACH_JOINT_SIGN[side] * np.median(np.asarray(frictions), axis=0)
                    if frictions else None)
        # Alternate held-out poses within each stage to cover wrist/elbow/shoulder.
        validate = validation_mask(np.array(traj_ids))
        for stage in (1, 2, 3):
            stage_rows = np.flatnonzero(np.array(stage_ids) == stage)
            if len(stage_rows) < 3:
                raise ValueError(f"{side}: stage {stage} 独立姿态不足")
        train = ~validate
        # Targets represent gravity; divide by retained gain when storing affine knobs.
        for j in range(7):
            prior = np.array([scales[j], biases[j]])
            a = np.column_stack([gain*x[:, j], np.full(len(x), gain)])
            old_error = target[:, j] - a @ prior
            accepted = False
            candidate = prior.copy()
            if np.std(x[train, j]) >= 0.05:
                # Fit bounded increments around the known working configuration.
                reg = np.diag([0.5, 5.0])
                delta = np.linalg.solve(a[train].T @ a[train] + reg,
                                       a[train].T @ old_error[train])
                delta[0] = np.clip(delta[0], -0.2*abs(prior[0]), 0.2*abs(prior[0]))
                delta[1] = np.clip(delta[1], -0.08, 0.08)
                candidate += delta
                before = np.sqrt(np.mean(old_error[validate]**2))
                after = np.sqrt(np.mean((target[validate,j]-a[validate]@candidate)**2))
                # Must improve held-out RMS and not worsen any held-out pose materially.
                new_error = target[:,j]-a@candidate
                accepted = (before-after > max(0.002, 0.02*before) and
                            np.all(np.abs(new_error[validate]) <= np.abs(old_error[validate])+0.02))
            else:
                before = after = np.sqrt(np.mean(old_error[validate]**2))
            if accepted:
                scales[j], biases[j] = candidate
            reports.append(dict(side=side, joint=j+1, accepted=bool(accepted),
                                validation_before=float(before), validation_candidate=float(after),
                                poses=len(x), unpaired_poses=dropped,
                                friction_estimate=None if friction is None else float(friction[j])))
        updates[side+"_gravity_scale"] = scales.tolist()
        updates[side+"_gravity_bias"] = biases.tolist()
    return updates, reports


def refine_file(data: dict, yaml_path: Path, urdf: Path,
                speed_max: float = 0.05, dry_run: bool = False, side: str = "both") -> list[dict]:
    params = load_params(yaml_path)
    gravity = runtime_gravity(data, urdf, yaml_path)
    sides = ("left", "right") if side == "both" else (side,)
    updates, reports = fit_runtime(data, gravity, params, speed_max, sides)
    for row in reports:
        friction = row.get("friction_estimate")
        print(f"[{row['side']}] j{row['joint']}: "
              f"{'更新' if row['accepted'] else '保留'} "
              f"验证 RMS 原值 {row['validation_before']:.4f} / 候选 {row['validation_candidate']:.4f} Nm"
              + ("" if friction is None else f"  静摩擦估计 {friction:+.3f} Nm（仅报告，不写入）"))
    unpaired = {row["side"]: row.get("unpaired_poses", 0) for row in reports}
    for name, count in unpaired.items():
        if count:
            print(f"[warn] {name}: {count} 个姿态只停稳了一侧，已从拟合中剔除")
    if not dry_run and any(row['accepted'] for row in reports):
        write_params(yaml_path, updates)
    return reports
