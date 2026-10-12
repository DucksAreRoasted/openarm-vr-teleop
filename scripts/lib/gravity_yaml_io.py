"""Read/merge/write config/gravity_comp.yaml without wiping unrelated knobs.

Calibration used to rewrite a short yaml and drop camera / distal / tracking
keys — that made post-calib zero-g feel broken.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any

# Keys we always keep if present when a writer only updates a subset.
PRESERVE_DEFAULTS: dict[str, Any] = {
    "log_errors": True,
    "urdf_gravity_scale": 1.25,
    "ee_payload_kg": 0.0,
    "g_scale": 0.95,
    "ff_gain": 0.95,
    "tracking_g_scale": 1.08,
    "moveit_g_scale": 1.08,
    "gripper_motor_kg": 0.12,
    "wrist_camera_kg": 0.12,
    "wrist_camera_offset_left": [0.0, 0.08, 0.04],
    "wrist_camera_offset_right": [0.0, -0.08, 0.04],
    "wrist_zero_g_kd": 0.22,
    "gripper_friction_fc": 0.08,
    "gripper_friction_fv": 0.005,
    "gripper_friction_k": 25.0,
    "distal_ee_gain": [0.24, 0.255, 0.45, 0.15, 0.60, 1.0, 1.1],
    "distal_cam_gain": [0.30, 0.32, 0.50, 0.30, 1.20, 0.75, 0.45],
    "link_mass_scale": [1.0] * 7,
    "left_link_mass_scale": [1.0] * 7,
    "right_link_mass_scale": [1.0] * 7,
    "left_link_com_delta": [0.0] * 21,  # link1..7 × (dx,dy,dz) in link frame [m]
    "right_link_com_delta": [0.0] * 21,
    "left_gravity_scale": [1.0] * 7,
    "left_gravity_bias": [0.0] * 7,
    "right_gravity_scale": [1.0] * 7,
    "right_gravity_bias": [0.0] * 7,
    "left_friction_fc": [0.35, 0.35, 0.20, 0.20, 0.08, 0.08, 0.06],
    "left_friction_fv": [0.02, 0.02, 0.015, 0.015, 0.006, 0.006, 0.005],
    "left_friction_k": [20.0, 20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
    "right_friction_fc": [0.35, 0.35, 0.20, 0.20, 0.08, 0.08, 0.06],
    "right_friction_fv": [0.02, 0.02, 0.015, 0.015, 0.006, 0.006, 0.005],
    "right_friction_k": [20.0, 20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
}

_ARRAY_RE = re.compile(r"^([A-Za-z0-9_]+):\s*\[(.*)\]\s*$")
_SCALAR_RE = re.compile(r"^([A-Za-z0-9_]+):\s*(.+)\s*$")


def _parse_value(raw: str) -> Any:
    raw = raw.strip()
    if raw in ("true", "True"):
        return True
    if raw in ("false", "False"):
        return False
    try:
        if "." in raw or "e" in raw.lower():
            return float(raw)
        return int(raw)
    except ValueError:
        return raw


def load_params(path: Path) -> dict[str, Any]:
    params = dict(PRESERVE_DEFAULTS)
    if not path.is_file():
        return params
    for line in path.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        m = _ARRAY_RE.match(line)
        if m:
            key = m.group(1)
            vals = [float(x.strip()) for x in m.group(2).split(",") if x.strip()]
            params[key] = vals
            continue
        m = _SCALAR_RE.match(line)
        if m and m.group(1) not in ("ros__parameters", "/**"):
            key = m.group(1)
            if key.endswith(":"):
                continue
            params[key] = _parse_value(m.group(2))
    return params


def _fmt_value(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(f"{float(v):.6f}" for v in value) + "]"
    if isinstance(value, float):
        return f"{value:.6f}"
    return str(value)


# Stable write order so diffs stay readable.
_WRITE_ORDER = [
    "log_errors",
    "urdf_gravity_scale",
    "ee_payload_kg",
    "g_scale",
    "ff_gain",
    "tracking_g_scale",
    "moveit_g_scale",
    "gripper_motor_kg",
    "wrist_camera_kg",
    "wrist_camera_offset_left",
    "wrist_camera_offset_right",
    "wrist_zero_g_kd",
    "gripper_friction_fc",
    "gripper_friction_fv",
    "gripper_friction_k",
    "distal_ee_gain",
    "distal_cam_gain",
    "link_mass_scale",
    "left_link_mass_scale",
    "right_link_mass_scale",
    "left_link_com_delta",
    "right_link_com_delta",
    "left_gravity_scale",
    "left_gravity_bias",
    "left_friction_fc",
    "left_friction_fv",
    "left_friction_k",
    "right_gravity_scale",
    "right_gravity_bias",
    "right_friction_fc",
    "right_friction_fv",
    "right_friction_k",
]


def write_params(path: Path, updates: dict[str, Any], *, base: dict[str, Any] | None = None) -> None:
    """Merge *updates* into existing yaml (or *base*) and write."""
    params = load_params(path) if base is None else dict(base)
    params.update(updates)
    # Keep g_scale / ff_gain aliases in sync when either is updated.
    if "g_scale" in updates and "ff_gain" not in updates:
        params["ff_gain"] = updates["g_scale"]
    if "ff_gain" in updates and "g_scale" not in updates:
        params["g_scale"] = updates["ff_gain"]

    lines = ["/**:", "  ros__parameters:"]
    seen: set[str] = set()
    for key in _WRITE_ORDER:
        if key not in params:
            continue
        lines.append(f"    {key}: {_fmt_value(params[key])}")
        seen.add(key)
    for key in sorted(params.keys()):
        if key in seen:
            continue
        lines.append(f"    {key}: {_fmt_value(params[key])}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n")
    try:
        path.chmod(0o666)
    except OSError:
        pass
