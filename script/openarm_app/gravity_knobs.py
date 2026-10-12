"""Read/write distal gains, joint scale/bias, friction knobs in gravity_comp.yaml."""

from __future__ import annotations

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
from lib.gravity_yaml_io import load_params, write_params  # noqa: E402

DEFAULT_EE = [0.24, 0.255, 0.45, 0.15, 0.60, 1.0, 1.1]
DEFAULT_CAM = [0.30, 0.32, 0.50, 0.30, 1.20, 0.75, 0.45]
DEFAULT_FC = [0.0, 0.0, 0.22, 0.32, 0.06, 0.05, 0.05]
DEFAULT_FV = [0.0, 0.0, 0.016, 0.022, 0.004, 0.0035, 0.0035]
DEFAULT_SCALE = [1.0] * 7
DEFAULT_BIAS = [0.0] * 7
DEFAULT_MOTOR = 0.12
DEFAULT_CAM_KG = 0.12
DEFAULT_EE_PAYLOAD = 0.0
DEFAULT_GFC = 0.08
DEFAULT_GFV = 0.005
DEFAULT_G_SCALE = 0.92

JOINT_NAMES = [
    "j1 肩旋转",
    "j2 肩俯仰",
    "j3 上臂滚",
    "j4 肘",
    "j5 前臂滚",
    "j6 腕偏航",
    "j7 腕俯仰",
]


def _vec7(params: dict, key: str, default: list[float]) -> list[float]:
    v = params.get(key, default)
    arr = [float(x) for x in list(v)[:7]]
    while len(arr) < 7:
        arr.append(float(default[len(arr)]))
    return arr


def read_knobs(path: Path) -> dict:
    params = load_params(path) if path.is_file() else {}
    return {
        "ee": _vec7(params, "distal_ee_gain", DEFAULT_EE),
        "cam": _vec7(params, "distal_cam_gain", DEFAULT_CAM),
        "fc_l": _vec7(params, "left_friction_fc", DEFAULT_FC),
        "fv_l": _vec7(params, "left_friction_fv", DEFAULT_FV),
        "fc_r": _vec7(params, "right_friction_fc", DEFAULT_FC),
        "fv_r": _vec7(params, "right_friction_fv", DEFAULT_FV),
        "scale_l": _vec7(params, "left_gravity_scale", DEFAULT_SCALE),
        "bias_l": _vec7(params, "left_gravity_bias", DEFAULT_BIAS),
        "scale_r": _vec7(params, "right_gravity_scale", DEFAULT_SCALE),
        "bias_r": _vec7(params, "right_gravity_bias", DEFAULT_BIAS),
        "motor": float(params.get("gripper_motor_kg", DEFAULT_MOTOR)),
        "cam_kg": float(params.get("wrist_camera_kg", DEFAULT_CAM_KG)),
        "ee_payload": float(params.get("ee_payload_kg", DEFAULT_EE_PAYLOAD)),
        "g_scale": float(params.get("g_scale", params.get("ff_gain", DEFAULT_G_SCALE))),
        "gfc": float(params.get("gripper_friction_fc", DEFAULT_GFC)),
        "gfv": float(params.get("gripper_friction_fv", DEFAULT_GFV)),
    }


def write_knobs(path: Path, knobs: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    updates = {
        "distal_ee_gain": [float(x) for x in knobs["ee"]],
        "distal_cam_gain": [float(x) for x in knobs["cam"]],
        "left_friction_fc": [float(x) for x in knobs["fc_l"]],
        "left_friction_fv": [float(x) for x in knobs["fv_l"]],
        "right_friction_fc": [float(x) for x in knobs["fc_r"]],
        "right_friction_fv": [float(x) for x in knobs["fv_r"]],
        "left_gravity_scale": [float(x) for x in knobs["scale_l"]],
        "left_gravity_bias": [float(x) for x in knobs["bias_l"]],
        "right_gravity_scale": [float(x) for x in knobs["scale_r"]],
        "right_gravity_bias": [float(x) for x in knobs["bias_r"]],
        "gripper_motor_kg": float(knobs["motor"]),
        "wrist_camera_kg": float(knobs["cam_kg"]),
        "ee_payload_kg": float(knobs["ee_payload"]),
        "g_scale": float(knobs["g_scale"]),
        "ff_gain": float(knobs["g_scale"]),
        "gripper_friction_fc": float(knobs["gfc"]),
        "gripper_friction_fv": float(knobs["gfv"]),
    }
    try:
        write_params(path, updates)
    except PermissionError as error:
        raise RuntimeError(
            f"写不进 {path}（权限不够）。请 chmod a+rw 后重试。\n{error}"
        ) from error
    try:
        path.chmod(0o666)
    except OSError:
        pass
    # Keep legacy verify that distal keys survived merge.
    text = path.read_text()
    if "distal_ee_gain:" not in text or "left_gravity_scale:" not in text:
        raise RuntimeError(f"write verify failed for {path}")
