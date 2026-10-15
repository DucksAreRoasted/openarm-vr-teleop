"""Friction-cancelled inertial identification from two-sided soft holds.

The model is exactly what the runtime evaluates with distal gains of 1: KDL
links 1–7 from the URDF, plus the end effector and wrist camera as point masses
on link7 that load every joint fully. The URDF chain stops at link7, so the
gripper body/motor and camera exist only as those point masses.

The point masses stay at their configured values: a rigid mass on link7 is
indistinguishable from link7's own mass and CoM, so link7 absorbs any error.
Each pose contributes the mean of its +δ and −δ holds, which cancels static
friction. Unknowns are (m, m·c) per link, regularised toward CAD.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from lib.pinocchio_gravity_id import FINGER_MASS, analytical_static_regressor
from lib.runtime_gravity_fit import check_calibration_log, collect_holds, hold_target
from lib.urdf_arm_gravity import ArmModel, load_arm_model
from lib.calibration_coverage import validation_mask

EE_OFFSET = np.array([0.0, 0.0, 0.1475])  # same point mass offset as the runtime
NOISE_NM = 0.05
PRIOR_MASS_FRACTION = 0.30
PRIOR_COM_M = 0.03
MASS_SCALE_RANGE = (0.5, 2.0)
COM_LIMIT_M = 0.08  # runtime clamps link_com_delta to this


@dataclass
class PairedPoses:
    traj: np.ndarray       # (n,)
    q_plus: np.ndarray     # (n, 7)
    q_minus: np.ndarray    # (n, 7)
    target: np.ndarray     # (n, 7) mean physical torque of both holds


@dataclass
class SideIdentification:
    side: str
    mass_scale: np.ndarray        # (7,)
    com_delta: np.ndarray         # (7, 3)
    clipped: list[str]
    validation_rms: np.ndarray    # (7,) held-out, model fitted on training poses only
    cad_validation_rms: np.ndarray
    poses: int


def paired_poses(data: dict, side: str, speed_max: float = 0.05) -> PairedPoses:
    check_calibration_log(data)
    if "approach" not in data:
        raise ValueError("日志没有 approach 列，请用两侧接近的新版标定重新采数")
    traj, q_plus, q_minus, target = [], [], [], []
    q_all = np.column_stack([data[f"q_meas{j}"] for j in range(1, 8)])
    for key, held in collect_holds(data, side, speed_max).items():
        if 1 not in held or -1 not in held:
            continue
        traj.append(key[0])
        q_plus.append(np.median(q_all[held[1]], axis=0))
        q_minus.append(np.median(q_all[held[-1]], axis=0))
        target.append(0.5 * (hold_target(data, held[1]) + hold_target(data, held[-1])))
    poses = PairedPoses(np.array(traj), np.array(q_plus), np.array(q_minus), np.array(target))
    if len(traj) < 9 or not {1, 2, 3}.issubset(set(poses.traj // 100)):
        raise ValueError(f"{side}: 两侧都停稳的腕/肘/肩姿态不足，无法辨识")
    return poses


def cad_theta(arm: ArmModel) -> np.ndarray:
    theta = np.zeros(28)
    for i in range(7):
        theta[4*i] = arm.masses[i+1]
        theta[4*i+1:4*i+4] = arm.masses[i+1] * arm.coms[i+1]
    return theta


def distal_masses(params: dict, side: str) -> list[tuple[float, np.ndarray]]:
    masses = []
    if params.get(f"{side}_gripper_present", True):
        masses.append((FINGER_MASS + max(0.0, params["gripper_motor_kg"])
                       + max(0.0, params["ee_payload_kg"]), EE_OFFSET))
    if params.get(f"{side}_wrist_camera_present", True):
        masses.append((max(0.0, params["wrist_camera_kg"]),
                       np.asarray(params[f"wrist_camera_offset_{side}"], dtype=float)))
    return masses


def regressors(arm: ArmModel, poses: PairedPoses, distal) -> tuple[np.ndarray, np.ndarray]:
    """Two-sided mean link regressor (n, 7, 28) and the known distal torque (n, 7)."""
    ys = []
    known = []
    for q_plus, q_minus in zip(poses.q_plus, poses.q_minus):
        y = 0.5 * (analytical_static_regressor(arm, q_plus) + analytical_static_regressor(arm, q_minus))
        ys.append(y[:, :28])
        # A point mass on link7 is link7's mass column plus its first-moment columns.
        known.append(sum(y[:, 24] * mass + y[:, 25:28] @ (mass * offset) for mass, offset in distal)
                     if distal else np.zeros(7))
    return np.array(ys), np.array(known)


def solve_theta(y: np.ndarray, target: np.ndarray, theta0: np.ndarray, arm: ArmModel) -> np.ndarray:
    m0 = arm.masses[1:8]
    sigma = np.concatenate([[PRIOR_MASS_FRACTION * m, PRIOR_COM_M * m, PRIOR_COM_M * m, PRIOR_COM_M * m]
                            for m in m0])
    a = np.vstack([y.reshape(-1, 28) / NOISE_NM, np.diag(1.0 / sigma)])
    b = np.concatenate([target.reshape(-1) / NOISE_NM, theta0 / sigma])
    return np.linalg.lstsq(a, b, rcond=None)[0]


def theta_to_runtime(theta: np.ndarray, arm: ArmModel) -> tuple[np.ndarray, np.ndarray, list[str]]:
    scales, deltas, clipped = np.ones(7), np.zeros((7, 3)), []
    for i in range(7):
        m0, c0 = arm.masses[i+1], arm.coms[i+1]
        scale = theta[4*i] / m0
        scales[i] = np.clip(scale, *MASS_SCALE_RANGE)
        if scales[i] != scale:
            clipped.append(f"link{i+1} 质量倍率 {scale:.2f}")
        com = theta[4*i+1:4*i+4] / (scales[i] * m0)
        deltas[i] = np.clip(com - c0, -COM_LIMIT_M, COM_LIMIT_M)
        if not np.allclose(deltas[i], com - c0):
            clipped.append(f"link{i+1} 质心偏移 {np.round(com - c0, 3).tolist()} m")
    return scales, deltas, clipped


def runtime_theta(arm: ArmModel, scales: np.ndarray, deltas: np.ndarray) -> np.ndarray:
    theta = np.zeros(28)
    for i in range(7):
        m = scales[i] * arm.masses[i+1]
        theta[4*i] = m
        theta[4*i+1:4*i+4] = m * (arm.coms[i+1] + deltas[i])
    return theta


def identify_side(data: dict, side: str, urdf: Path, params: dict,
                  speed_max: float = 0.05) -> SideIdentification:
    arm = load_arm_model(str(urdf), side)
    poses = paired_poses(data, side, speed_max)
    y, known = regressors(arm, poses, distal_masses(params, side))
    links_target = poses.target - known
    theta0 = cad_theta(arm)
    held_out = validation_mask(poses.traj)
    train = ~held_out

    def rms(theta: np.ndarray) -> np.ndarray:
        error = links_target[held_out] - np.einsum("njk,k->nj", y[held_out], theta)
        return np.sqrt(np.mean(error**2, axis=0))

    scales_t, deltas_t, _ = theta_to_runtime(solve_theta(y[train], links_target[train], theta0, arm), arm)
    validation = rms(runtime_theta(arm, scales_t, deltas_t))
    scales, deltas, clipped = theta_to_runtime(solve_theta(y, links_target, theta0, arm), arm)
    return SideIdentification(side, scales, deltas, clipped, validation, rms(theta0), len(poses.traj))
