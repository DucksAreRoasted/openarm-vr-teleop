"""URDF serial-arm gravity: G(q) is linear in link masses (fixed CoMs).

Used for quasi-static inertial identification without Pinocchio.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np


def _rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def _xyz_rpy(origin: ET.Element | None) -> tuple[np.ndarray, np.ndarray]:
    xyz = np.zeros(3)
    rpy = np.zeros(3)
    if origin is None:
        return xyz, rpy
    if origin.get("xyz"):
        xyz = np.array([float(x) for x in origin.get("xyz").split()], dtype=np.float64)
    if origin.get("rpy"):
        rpy = np.array([float(x) for x in origin.get("rpy").split()], dtype=np.float64)
    return xyz, rpy


@dataclass
class ArmModel:
    side: str
    link_names: list[str]
    masses: np.ndarray  # (8,) link0..link7
    coms: np.ndarray  # (8,3)
    joint_xyz: np.ndarray  # (7,3)
    joint_rpy: np.ndarray  # (7,3)
    joint_axis: np.ndarray  # (7,3)
    mount_xyz: np.ndarray
    mount_rpy: np.ndarray


def load_arm_model(urdf_path: str, side: str) -> ArmModel:
    root = ET.parse(urdf_path).getroot()
    prefix = f"openarm_{side}_"

    def link_inertial(name: str) -> tuple[float, np.ndarray]:
        link = root.find(f"./link[@name='{name}']")
        if link is None:
            raise SystemExit(f"URDF 缺少 link {name}")
        iner = link.find("inertial")
        if iner is None:
            return 0.0, np.zeros(3)
        mass = float(iner.find("mass").get("value"))
        xyz, _ = _xyz_rpy(iner.find("origin"))
        return mass, xyz

    link_names = [f"{prefix}link{i}" for i in range(8)]
    masses = np.zeros(8)
    coms = np.zeros((8, 3))
    for i, name in enumerate(link_names):
        masses[i], coms[i] = link_inertial(name)

    mount = root.find(f"./joint[@name='openarm_{side}_openarm_body_link0_joint']")
    if mount is None:
        raise SystemExit(f"URDF 缺少 body→{side} mount joint")
    mount_xyz, mount_rpy = _xyz_rpy(mount.find("origin"))

    joint_xyz = np.zeros((7, 3))
    joint_rpy = np.zeros((7, 3))
    joint_axis = np.zeros((7, 3))
    for j in range(7):
        jname = f"{prefix}joint{j + 1}"
        joint = root.find(f"./joint[@name='{jname}']")
        if joint is None:
            raise SystemExit(f"URDF 缺少 {jname}")
        joint_xyz[j], joint_rpy[j] = _xyz_rpy(joint.find("origin"))
        axis_el = joint.find("axis")
        if axis_el is None or not axis_el.get("xyz"):
            raise SystemExit(f"{jname} 无 axis")
        joint_axis[j] = np.array([float(x) for x in axis_el.get("xyz").split()], dtype=np.float64)
        n = float(np.linalg.norm(joint_axis[j]))
        if n < 1e-9:
            raise SystemExit(f"{jname} axis 为零")
        joint_axis[j] /= n

    return ArmModel(
        side=side,
        link_names=link_names,
        masses=masses,
        coms=coms,
        joint_xyz=joint_xyz,
        joint_rpy=joint_rpy,
        joint_axis=joint_axis,
        mount_xyz=mount_xyz,
        mount_rpy=mount_rpy,
    )


def _fk_with_ee(
    model: ArmModel, q: np.ndarray, ee_com_in_link7: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    t = np.eye(4)
    t[:3, :3] = _rpy_matrix(model.mount_rpy)
    t[:3, 3] = model.mount_xyz

    com_w = np.zeros((8, 3))
    com_w[0] = t[:3, 3] + t[:3, :3] @ model.coms[0]
    joint_o = np.zeros((7, 3))
    joint_ax = np.zeros((7, 3))

    for j in range(7):
        t_j = np.eye(4)
        t_j[:3, :3] = _rpy_matrix(model.joint_rpy[j])
        t_j[:3, 3] = model.joint_xyz[j]
        t = t @ t_j
        axis_local = model.joint_axis[j]
        c, s = np.cos(q[j]), np.sin(q[j])
        kx, ky, kz = axis_local
        k_hat = np.array([[0, -kz, ky], [kz, 0, -kx], [-ky, kx, 0]], dtype=np.float64)
        r_q = np.eye(3) + s * k_hat + (1 - c) * (k_hat @ k_hat)
        joint_o[j] = t[:3, 3]
        joint_ax[j] = t[:3, :3] @ axis_local
        t_q = np.eye(4)
        t_q[:3, :3] = r_q
        t = t @ t_q
        com_w[j + 1] = t[:3, 3] + t[:3, :3] @ model.coms[j + 1]

    ee_w = t[:3, 3] + t[:3, :3] @ np.asarray(ee_com_in_link7, dtype=np.float64)
    return com_w, joint_o, joint_ax, ee_w


def gravity_regressor(
    model: ArmModel,
    q: np.ndarray,
    *,
    ee_com_in_link7: np.ndarray,
    g_world: np.ndarray | None = None,
) -> np.ndarray:
    """Y (7,8): G = Y @ [m_link1..m_link7, m_ee]."""
    if g_world is None:
        g_world = np.array([0.0, 0.0, -9.81])
    q = np.asarray(q, dtype=np.float64).reshape(7)
    com_w, joint_o, joint_ax, ee_w = _fk_with_ee(model, q, ee_com_in_link7)
    y = np.zeros((7, 8), dtype=np.float64)
    for j in range(7):
        axis = joint_ax[j]
        origin = joint_o[j]
        for col, link_i in enumerate(range(1, 8)):
            # link_i distal to joint j iff link_i >= j+1
            if link_i < j + 1:
                continue
            r = com_w[link_i] - origin
            # Match KDL ChainDynParam::JntToGravity sign: τ = axis · (g × r)
            # (equivalent to -axis · (r × g); the cross order was previously inverted).
            y[j, col] = float(axis @ np.cross(g_world, r))
        r_ee = ee_w - origin
        y[j, 7] = float(axis @ np.cross(g_world, r_ee))
    return y


def gravity_torque(
    model: ArmModel,
    q: np.ndarray,
    masses_moving: np.ndarray,
    ee_mass: float,
    ee_com_in_link7: np.ndarray,
) -> np.ndarray:
    y = gravity_regressor(model, q, ee_com_in_link7=ee_com_in_link7)
    theta = np.concatenate([np.asarray(masses_moving, dtype=np.float64).reshape(7), [ee_mass]])
    return y @ theta
