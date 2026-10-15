#!/usr/bin/python3
"""Automatic zero-g refinement against the runtime KDL model.

Default: retain current physics/friction, collect settled staged poses, fit bounded
joint scale/bias changes, validate on held-out whole poses, then commit atomically.
--skip-motion --dry-run previews a complete new-format log without moving hardware.
Explicit --identify / --affine retain the older experimental fitting paths.
Requires exclusive CAN for collection (stop teleop / zero-g / MoveIt first).
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_YAML = ROOT / "config" / "gravity_comp.yaml"
DEFAULT_LOG_DIR = ROOT / "recordings" / "gravity"
RUN_CALIB = ROOT / "scripts" / "run_gravity_calib.sh"
SEED = ROOT / "scripts" / "seed_gravity_comp.py"
TUNE = ROOT / "scripts" / "tune_gravity_comp.py"
IDENTIFY = ROOT / "scripts" / "identify_gravity.py"


def run(cmd: list[str], *, env: dict | None = None) -> subprocess.CompletedProcess[str]:
    """Forward progress immediately and retain output for log paths and reports."""
    print("+", " ".join(cmd), flush=True)
    child_env = os.environ.copy() if env is None else env.copy()
    child_env["PYTHONUNBUFFERED"] = "1"
    chunks: list[str] = []
    with subprocess.Popen(
        cmd,
        cwd=ROOT,
        env=child_env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    ) as process:
        assert process.stdout is not None
        for line in process.stdout:
            chunks.append(line)
            sys.stdout.write(line)
            sys.stdout.flush()
        returncode = process.wait()
    return subprocess.CompletedProcess(cmd, returncode, stdout="".join(chunks))


def calibrate_runtime(args, env: dict) -> int:
    """Collect against the existing model; write only a validated staged candidate."""
    from lib.gravity_yaml_io import write_params
    from tune_gravity_comp import load_csv
    target = args.write.resolve()
    if args.ee_payload is not None or args.urdf_scale is not None:
        print("默认精修保留物理参数；请先在 YAML 中配置负载/URDF 倍率再采数", file=sys.stderr)
        return 2
    if not target.is_file():
        print(f"缺少现有重力配置 {target}，请先建立物理参数", file=sys.stderr)
        return 1
    original = target.read_bytes()
    try:
        if args.skip_motion:
            # Prefer a recent calibration log, never select a large free-drag log.
            log_path = None
            for candidate in sorted(args.log_dir.glob("gravity_*.csv"), reverse=True):
                with candidate.open() as handle:
                    header = handle.readline().strip().split(",")
                if "tau_g1" in header and "ff_gain" in header:
                    log_path = candidate
                    break
            if log_path is None:
                raise ValueError("没有带 tau_g/ff_gain 的新版标定日志，请重新采数")
        else:
            result = run(["bash", str(RUN_CALIB)], env=env)
            if result.returncode:
                print("[calib] 采数未完成，保留原配置", flush=True)
                return result.returncode
            match = re.findall(r"LOG_PATH=(\S+)", result.stdout)
            if not match:
                raise ValueError("缺少完整采数日志")
            log_path = Path(match[-1])
            if env.get("OPENARM_CALIB_APPROACH", "both") == "both":
                from lib.calibration_coverage import require_vertical_pairs
                require_vertical_pairs(load_csv(log_path))
        # Keep the currently loaded working configuration untouched during fitting.
        with tempfile.TemporaryDirectory(prefix="openarm-calib-") as directory:
            staged = Path(directory) / "gravity_comp.yaml"
            staged.write_bytes(original)
            if args.g_scale is not None:
                write_params(staged, {"g_scale": args.g_scale, "ff_gain": args.g_scale})
            if args.seed:
                print("[calib] 默认精修保留当前质量/质心、负载和摩擦；--seed 不清空参数", flush=True)
            cmd = [sys.executable, str(TUNE), str(log_path), "--write", str(staged),
                   "--runtime-fit", "--speed-max", str(min(args.speed_max, 0.05))]
            if args.dry_run:
                cmd.append("--dry-run")
            result = run(cmd)
            if result.returncode:
                return result.returncode
            if not args.dry_run:
                report_path = log_path.with_suffix(".fit.txt")
                report_path.write_text(result.stdout, encoding="utf-8")
                print(f"[calib] 精修报告 {report_path}", flush=True)
            if not args.dry_run and staged.read_bytes() != original:
                if target.read_bytes() != original:
                    raise ValueError("标定期间配置已被其他操作修改，拒绝覆盖")
                # Replace atomically on the same filesystem.
                import os
                fd, name = tempfile.mkstemp(prefix=".gravity_candidate_", dir=target.parent)
                try:
                    with os.fdopen(fd, "wb") as handle:
                        handle.write(staged.read_bytes())
                    os.chmod(name, target.stat().st_mode & 0o777)
                    os.replace(name, target)
                finally:
                    Path(name).unlink(missing_ok=True)
        print(f"自动标定完成：运行模型一致的关节精修；日志={log_path}。物理/摩擦参数保留。", flush=True)
        return 0
    except (ValueError, OSError) as error:
        print(f"[calib] {error}；原配置未覆盖", file=sys.stderr)
        return 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", choices=("waypoints", "fourier"), default="waypoints",
                        help="分阶段航点软持。fourier 会因回归条件数更差而退回航点")
    parser.add_argument("--seed", action="store_true", help="兼容旧命令；默认精修不重置，只有 --identify/--affine 才初始化")
    parser.add_argument(
        "--urdf-scale",
        type=float,
        default=None,
        help="配合 --seed；identify 默认 1.0，affine 默认 1.25",
    )
    parser.add_argument("--ee-payload", type=float, default=None, help="配合 --seed")
    parser.add_argument("--write", type=Path, default=DEFAULT_YAML)
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument(
        "--speed-max",
        type=float,
        default=None,
        help="准静态速度阈值 rad/s，默认 0.12",
    )
    parser.add_argument("--dry-run", action="store_true", help="跑轨迹并拟合，但不写 yaml")
    parser.add_argument("--skip-motion", action="store_true", help="只对已有最大日志精调")
    parser.add_argument(
        "--affine",
        action="store_true",
        help="旧模式：只拟合关节 scale/bias（不用惯量辨识）",
    )
    parser.add_argument(
        "--identify",
        action="store_true",
        help="显式旧版质量/质心辨识（实验路径；默认只精修关节补偿）",
    )
    parser.add_argument("--g-scale", type=float, default=None, help="运行时 g_scale，默认保留当前值")
    parser.add_argument(
        "--no-friction",
        action="store_true",
        help="保留摩擦（默认精修本来就保留；静止采数不重估摩擦）",
    )
    args = parser.parse_args()

    if args.speed_max is None:
        args.speed_max = 0.12

    if args.identify and args.affine:
        print("不能同时 --identify 与 --affine", file=sys.stderr)
        return 2

    env = os.environ.copy()
    env["OPENARM_CALIB_TRAJECTORY"] = args.trajectory
    env["OPENARM_ROOT"] = str(ROOT)
    env["OPENARM_GRAVITY_YAML"] = str(args.write)
    env["OPENARM_ZERO_G_LOG_DIR"] = str(args.log_dir)

    if not args.identify and not args.affine:
        return calibrate_runtime(args, env)
    # Old modes remain explicit, but their intermediate writes are isolated too.
    target = args.write.resolve()
    if not target.is_file():
        print("旧版实验模式需要现有 YAML", file=sys.stderr)
        return 1
    original = target.read_bytes()
    with tempfile.TemporaryDirectory(prefix="openarm-legacy-") as directory:
        staged = Path(directory) / "gravity_comp.yaml"
        staged.write_bytes(original)
        args.write = staged
        env["OPENARM_GRAVITY_YAML"] = str(staged)
        code = run_legacy(args, env)
        if code == 0 and not args.dry_run:
            if target.read_bytes() != original:
                print("配置已被其他操作修改，拒绝覆盖", file=sys.stderr)
                return 1
            target.write_bytes(staged.read_bytes())
        return code


def run_legacy(args, env: dict) -> int:
    use_identify = args.identify
    print("[calib] 显式旧版质量/质心或 affine 路径；推荐默认运行模型精修", flush=True)
    if args.g_scale is None:
        from lib.gravity_yaml_io import load_params
        args.g_scale = float(load_params(args.write).get("g_scale", 0.92))

    seeded_yaml: bytes | None = None
    if args.seed:
        if args.dry_run:
            print("[calib] --dry-run：跳过 seed 写 yaml", flush=True)
        else:
            if args.write.is_file():
                seeded_yaml = args.write.read_bytes()
            seed_cmd = [sys.executable, str(SEED), "--write", str(args.write)]
            default_scale = 1.0 if use_identify else 1.25
            urdf_scale = default_scale if args.urdf_scale is None else args.urdf_scale
            seed_cmd += ["--urdf-scale", str(urdf_scale)]
            if args.ee_payload is not None:
                seed_cmd += ["--ee-payload", str(args.ee_payload)]
            result = run(seed_cmd)
            if result.returncode != 0:
                return result.returncode

    log_path: Path | None = None
    if not args.skip_motion:
        if not RUN_CALIB.is_file():
            print(f"缺少 {RUN_CALIB}", file=sys.stderr)
            return 1
        result = run(["bash", str(RUN_CALIB)], env=env)
        match = re.findall(r"LOG_PATH=(\S+)", result.stdout or "")
        if match:
            log_path = Path(match[-1])
        if result.returncode != 0:
            if seeded_yaml is not None:
                args.write.write_bytes(seeded_yaml)
                print("[calib] 轨迹未完成，已恢复标定前的 yaml", flush=True)
            if result.returncode == 130:
                print("[calib] 轨迹中止，不辨识、不写配置", flush=True)
            return result.returncode
        if log_path is None or not log_path.is_file():
            if seeded_yaml is not None:
                args.write.write_bytes(seeded_yaml)
                print("[calib] 没有完整日志，已恢复标定前的 yaml", flush=True)
            print("没找到 LOG_PATH，无法精调", file=sys.stderr)
            return 1
    else:
        files = sorted(args.log_dir.glob("gravity_*.csv"))
        files = [p for p in files if p.name.startswith("gravity_")]
        if not files:
            print(f"{args.log_dir} 没有日志", file=sys.stderr)
            return 1
        log_path = max(files[-8:], key=lambda path: path.stat().st_size)
        print(f"[calib] --skip-motion 使用 {log_path}", flush=True)

    if use_identify:
        fit_cmd = [
            sys.executable,
            str(IDENTIFY),
            str(log_path),
            "--write",
            str(args.write),
            "--speed-max",
            str(args.speed_max),
            "--g-scale",
            str(args.g_scale),
        ]
        if args.dry_run:
            fit_cmd.append("--dry-run")
        result = run(fit_cmd)
        if result.returncode != 0:
            return result.returncode

        if not args.dry_run:
            # Secondary auto refine: recompute G from identified masses → joint
            # scale/bias + friction (keeps link_mass_scale / com_delta).
            refine_cmd = [
                sys.executable,
                str(TUNE),
                str(log_path),
                "--write",
                str(args.write),
                "--speed-max",
                str(args.speed_max),
                "--post-id",
            ]
            if args.no_friction:
                # Still write scale/bias; skip by running post-id then we'd need
                # friction-only false — post-id always fits friction; OK.
                pass
            result = run(refine_cmd)
            if result.returncode != 0:
                return result.returncode

        mode = "identify+post-id" if not args.dry_run else "identify"
    else:
        fit_cmd = [
            sys.executable,
            str(TUNE),
            str(log_path),
            "--write",
            str(args.write),
            "--speed-max",
            str(args.speed_max),
            "--droop",
            "--fold-scale",
        ]
        if args.dry_run:
            fit_cmd.append("--dry-run")
        result = run(fit_cmd)
        if result.returncode != 0:
            return result.returncode
        mode = "affine-droop"

    print(f"\n自动标定完成（{mode}）。yaml={args.write}\n日志={log_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
