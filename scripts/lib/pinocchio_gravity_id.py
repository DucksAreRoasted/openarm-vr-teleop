"""Quasi-static inertial ID: Pinocchio static regressor, with analytical fallback.

Identifies per-link (m, mx, my, mz) for link1..link7 (+ EE point mass), then
converts to:
  - mass scales relative to URDF CAD mass
  - CoM deltas in each link frame (meters)

Adapted from lbr-stack/dynamic_parameter_identification
(https://github.com/lbr-stack/dynamic_parameter_identification) — quasi-static subset:

  ✓ CAD prior (mass light / CoM heavy in objective)
  ✓ iterative weighted LS (per-joint residual σ)
  ✓ hard-sample reweight
  ✓ physical projection (mass floor, CoM bounds)
  ✓ regressor condition-number report
  ✓ base-parameter hygiene: drop unexcited (dead) columns; WLS+ridge for rank

Not ported (need full dynamics + optas/casadi/IPOPT, little gain for soft-hold G(q)):
  ✗ full RNEA + 10-param inertia LMI + IPOPT
  ✗ Fourier traj NLP for cond(Y)
  ✗ Tracking-Differentiator velocity filter
  ✗ LBR full Pb/Kd base remapping (quasi-static soft-hold is too rank-deficient
    for their QR freeze; dead-column drop + ridge is the stable substitute)

Friction is fit separately on moving samples (tune_gravity_comp --friction-only).
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from lib.urdf_arm_gravity import ArmModel, _rpy_matrix, gravity_torque, load_arm_model

FINGER_MASS = 2.0 * 0.03602545343277134
EE_COM = np.array([0.00645, 0.0, 0.1245], dtype=np.float64)
G_WORLD = np.array([0.0, 0.0, -9.81], dtype=np.float64)
KP_HOLD = np.array([2.5, 2.5, 2.0, 2.0, 1.0, 1.0, 1.0], dtype=np.float64)
KD_HOLD = np.array([0.35, 0.35, 0.3, 0.3, 0.15, 0.12, 0.15], dtype=np.float64)
# Soft-hold home used by openarm_gravity_calib (left); right is mirrored in joint space.
HOME_Q_LEFT = np.array([0.0, -0.70, 0.0, 1.20, 0.0, 0.0, 0.0], dtype=np.float64)


def try_import_pinocchio():
    try:
        import pinocchio as pin  # type: ignore

        return pin
    except ImportError:
        pass
    for extra in (
        "/opt/ros/humble/local/lib/python3.10/dist-packages",
        "/opt/ros/humble/lib/python3.10/site-packages",
        "/opt/ros/humble/lib/python3.10/dist-packages",
    ):
        if extra not in sys.path:
            sys.path.insert(0, extra)
        try:
            import pinocchio as pin  # type: ignore

            return pin
        except ImportError:
            continue
    return None


@dataclass
class IdResult:
    mass_scales: np.ndarray  # (7,)
    com_delta: np.ndarray  # (7, 3) meters in link frame
    ee_mass: float
    rms: float
    n: int
    backend: str
    m0: np.ndarray


def analytical_static_regressor(model: ArmModel, q: np.ndarray) -> np.ndarray:
    """Y (7, 29): G = Y @ [m1,mx1,my1,mz1, … m7,mx7,my7,mz7, m_ee].

    mx=m*cx etc. expressed in each link frame (same as URDF inertial origin).
    """
    q = np.asarray(q, dtype=np.float64).reshape(7)
    # FK with zero CoM → link origins + rotations; then build regressor columns.
    t = np.eye(4)
    t[:3, :3] = _rpy_matrix(model.mount_rpy)
    t[:3, 3] = model.mount_xyz

    joint_o = np.zeros((7, 3))
    joint_ax = np.zeros((7, 3))
    link_R = np.zeros((8, 3, 3))
    link_o = np.zeros((8, 3))
    link_R[0] = t[:3, :3]
    link_o[0] = t[:3, 3]

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
        link_R[j + 1] = t[:3, :3]
        link_o[j + 1] = t[:3, 3]

    ee_w = link_o[7] + link_R[7] @ EE_COM
    y = np.zeros((7, 29), dtype=np.float64)
    for j in range(7):
        axis = joint_ax[j]
        origin = joint_o[j]
        for li in range(1, 8):
            if li < j + 1:
                continue
            col = (li - 1) * 4
            # τ = axis · (g × r), r = o_link + R c - joint_o
            # m term with c=0: axis·(g×(o-joint_o))
            # mx term: axis·(g×(R[:,0]))
            r0 = link_o[li] - origin
            y[j, col + 0] = float(axis @ np.cross(G_WORLD, r0))
            y[j, col + 1] = float(axis @ np.cross(G_WORLD, link_R[li] @ np.array([1.0, 0.0, 0.0])))
            y[j, col + 2] = float(axis @ np.cross(G_WORLD, link_R[li] @ np.array([0.0, 1.0, 0.0])))
            y[j, col + 3] = float(axis @ np.cross(G_WORLD, link_R[li] @ np.array([0.0, 0.0, 1.0])))
        y[j, 28] = float(axis @ np.cross(G_WORLD, ee_w - origin))
    return y


def _build_pinocchio_arm(pin, urdf_path: str, side: str):
    full = pin.buildModelFromUrdf(urdf_path)
    prefix = f"openarm_{side}_"
    keep = []
    for i in range(1, 8):
        name = f"{prefix}joint{i}"
        if not full.existJointName(name):
            raise RuntimeError(f"URDF 缺少 {name}")
        keep.append(full.getJointId(name))
    locked = []
    for jid in range(1, full.njoints):
        if full.nqs[jid] == 0:
            continue
        if jid not in keep:
            locked.append(jid)
    q_ref = pin.neutral(full)
    reduced = pin.buildReducedModel(full, locked, q_ref)
    if reduced.nv != 7:
        raise RuntimeError(f"缩减后 nv={reduced.nv}，期望 7")
    data = reduced.createData()
    return reduced, data, prefix


def home_q_for_side(side: str) -> np.ndarray:
    q = HOME_Q_LEFT.copy()
    if side == "right":
        q[0] = -q[0]
        q[1] = -q[1]
        q[4] = -q[4]
        q[5] = -q[5]
        q[6] = -q[6]
    return q


def nominal_gravity_from_urdf(
    urdf_path: str,
    side: str,
    q: np.ndarray | None = None,
    *,
    ee_mass: float = FINGER_MASS,
) -> tuple[np.ndarray, str, dict]:
    """Compute CAD gravity G(q) from URDF — the analytical initial model.

    Prefers Pinocchio ``computeGeneralizedGravity``; falls back to the serial-arm
    analytical model (same mass/CoM as URDF + EE point mass for fingers).
    """
    q = home_q_for_side(side) if q is None else np.asarray(q, dtype=np.float64).reshape(7)
    arm = load_arm_model(urdf_path, side)
    info = {
        "masses_link1_7": arm.masses[1:8].copy(),
        "coms_link1_7": arm.coms[1:8].copy(),
        "ee_mass": float(ee_mass),
        "q": q.copy(),
    }

    pin = try_import_pinocchio()
    if pin is not None:
        try:
            model, data, _ = _build_pinocchio_arm(pin, urdf_path, side)
            # Bake finger/EE point mass into terminal link (link7) inertia.
            ee_link = f"openarm_{side}_link7"
            if model.existFrame(ee_link) and ee_mass > 1e-9:
                fid = model.getFrameId(ee_link)
                parent = model.frames[fid].parent
                model.inertias[parent] += pin.Inertia(
                    ee_mass, EE_COM, np.zeros((3, 3), dtype=np.float64)
                )
            g = np.asarray(pin.computeGeneralizedGravity(model, data, q), dtype=np.float64).reshape(
                7
            )
            return g, "pinocchio", info
        except Exception as exc:  # noqa: BLE001
            info["pinocchio_error"] = str(exc)

    g = gravity_torque(arm, q, arm.masses[1:8], ee_mass, EE_COM)
    return np.asarray(g, dtype=np.float64).reshape(7), "analytical", info


def pinocchio_static_regressor(pin, model, data, q7: np.ndarray) -> np.ndarray:
    """Return Y (7, 29) matching analytical layout using Pinocchio static regressor."""
    q = np.asarray(q7, dtype=np.float64).reshape(7)
    # Reduced model nq==nv==7 with joint order joint1..joint7.
    y_full = pin.computeStaticRegressor(model, data, q)  # (7, 10 * njoints)
    # Body inertias: index 0 = universe, then joints. Moving links are joints 1..7.
    y = np.zeros((7, 29), dtype=np.float64)
    for body in range(1, 8):
        base = 10 * body
        col = (body - 1) * 4
        # Pinocchio static regressor packing: m, mx, my, mz, then inertia.
        y[:, col : col + 4] = y_full[:, base : base + 4]
    # EE point mass: approximate as extra mass at EE_COM on link7 via analytical column.
    # (Pinocchio model has no finger bodies.) Use analytical EE column only.
    arm = None  # filled by caller if needed
    _ = arm
    return y


def hold_pd_torque(data: dict[str, np.ndarray], row: int) -> np.ndarray:
    """Soft-hold PD torque. tau_ff also contains scale, bias and friction, so it is not G(q)."""
    err = np.array([data[f"e{j}"][row] for j in range(1, 8)], dtype=np.float64)
    if "dq1" in data and "dq_meas1" in data:
        edot = np.array(
            [data[f"dq{j}"][row] - data[f"dq_meas{j}"][row] for j in range(1, 8)],
            dtype=np.float64,
        )
    else:
        edot = np.zeros(7, dtype=np.float64)
    return KP_HOLD * err + KD_HOLD * edot


def target_tau_from_row(data: dict[str, np.ndarray], row: int) -> np.ndarray:
    """Static gravity torque.

    The motor command is tau_ff + PD, and tau_ff = tau_g + friction.
    tau_g = scale·g_model + bias. Friction is not gravity, so the regressor
    target is tau_g + PD. Logs without tau_g fall back to g_model + PD.
    """
    pd = hold_pd_torque(data, row)
    if "tau_g1" in data:
        tau_g = np.array([data[f"tau_g{j}"][row] for j in range(1, 8)], dtype=np.float64)
        return tau_g + pd
    if "g_model1" in data:
        g_model = np.array([data[f"g_model{j}"][row] for j in range(1, 8)], dtype=np.float64)
        return g_model + pd
    tau_ff = np.array([data[f"tau_ff{j}"][row] for j in range(1, 8)], dtype=np.float64)
    return tau_ff + pd


# Distal→proximal factory stages (traj_id bands from openarm_gravity_calib).
STAGE_WRIST = (100, 199)  # free: link5–7 + EE
STAGE_ELBOW = (200, 299)  # free: link3–4
STAGE_SHOULDER = (300, 399)  # free: link1–2


def _cad_prior(m0: np.ndarray, com0: np.ndarray) -> np.ndarray:
    prior = np.zeros(29, dtype=np.float64)
    for i in range(7):
        prior[4 * i + 0] = m0[i]
        prior[4 * i + 1] = m0[i] * com0[i, 0]
        prior[4 * i + 2] = m0[i] * com0[i, 1]
        prior[4 * i + 3] = m0[i] * com0[i, 2]
    prior[28] = FINGER_MASS
    return prior


def _theta_from_scales(
    m0: np.ndarray, com0: np.ndarray, scales: np.ndarray, com_delta: np.ndarray, ee_mass: float
) -> np.ndarray:
    theta = np.zeros(29, dtype=np.float64)
    for i in range(7):
        m = scales[i] * m0[i]
        c = com0[i] + com_delta[i]
        theta[4 * i + 0] = m
        theta[4 * i + 1 : 4 * i + 4] = m * c
    theta[28] = ee_mass
    return theta


def _clip_free_params(
    theta: np.ndarray,
    free_links: list[int],
    free_ee: bool,
    m0: np.ndarray,
    com0: np.ndarray,
    scales: np.ndarray,
    com_delta: np.ndarray,
    ee_mass: float,
    *,
    scale_lo: float,
    scale_hi: float,
    com_lim: float,
) -> tuple[np.ndarray, np.ndarray, float]:
    scales = scales.copy()
    com_delta = com_delta.copy()
    for i in free_links:
        m = float(np.clip(theta[4 * i], 0.15 * m0[i], 2.5 * m0[i]))
        scales[i] = float(np.clip(m / max(m0[i], 1e-6), scale_lo, scale_hi))
        m_use = scales[i] * m0[i]
        if m_use < 1e-6:
            continue
        cx = theta[4 * i + 1] / m_use
        cy = theta[4 * i + 2] / m_use
        cz = theta[4 * i + 3] / m_use
        com_delta[i, 0] = float(np.clip(cx - com0[i, 0], -com_lim, com_lim))
        com_delta[i, 1] = float(np.clip(cy - com0[i, 1], -com_lim, com_lim))
        com_delta[i, 2] = float(np.clip(cz - com0[i, 2], -com_lim, com_lim))
    if free_ee:
        ee_mass = float(np.clip(theta[28], FINGER_MASS * 0.5, FINGER_MASS + 0.35))
    return scales, com_delta, ee_mass


def _collect_quiet_regressors(
    data: dict[str, np.ndarray],
    side: str,
    urdf_path: str,
    *,
    speed_max: float,
    max_samples: int,
    traj_lo: int | None = None,
    traj_hi: int | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, ArmModel, Any, Any, Any, str]:
    mask = data["_side"] == side
    if not np.any(mask):
        raise SystemExit(f"日志里没有 side={side}")

    if "traj" in data and traj_lo is not None and traj_hi is not None:
        mask = mask & (data["traj"] >= traj_lo) & (data["traj"] <= traj_hi)

    side_idx = np.flatnonzero(mask)
    if len(side_idx) == 0:
        raise SystemExit(f"[{side}] traj∈[{traj_lo},{traj_hi}] 无样本")

    speed = np.zeros(len(side_idx))
    for j in range(1, 8):
        speed = np.maximum(speed, np.abs(data[f"dq_meas{j}"][side_idx]))
    quiet = speed < speed_max
    if int(quiet.sum()) < 20:
        raise SystemExit(
            f"[{side}] traj∈[{traj_lo},{traj_hi}] 准静态样本过少 n={int(quiet.sum())}"
        )

    quiet_idx = side_idx[quiet]
    rng = np.random.default_rng(0)
    if len(quiet_idx) > max_samples:
        quiet_idx = quiet_idx[rng.choice(len(quiet_idx), size=max_samples, replace=False)]

    arm_model = load_arm_model(urdf_path, side)
    pin = try_import_pinocchio()
    pin_model = pin_data = None
    backend = "analytical"
    if pin is not None:
        try:
            pin_model, pin_data, _ = _build_pinocchio_arm(pin, urdf_path, side)
            backend = "pinocchio"
        except Exception as exc:  # noqa: BLE001
            print(f"[{side}] Pinocchio 模型失败，回退解析回归器：{exc}")
            pin = None

    y_rows = []
    tau_rows = []
    q1_abs = []
    for r in quiet_idx:
        q = np.array([data[f"q_meas{j}"][r] for j in range(1, 8)], dtype=np.float64)
        target = target_tau_from_row(data, r)
        if pin is not None and pin_model is not None:
            y = pinocchio_static_regressor(pin, pin_model, pin_data, q)
            y_a = analytical_static_regressor(arm_model, q)
            y[:, 28] = y_a[:, 28]
        else:
            y = analytical_static_regressor(arm_model, q)
        y_rows.append(y)
        tau_rows.append(target)
        q1_abs.append(abs(q[0]))

    return (
        np.vstack(y_rows),
        np.concatenate(tau_rows),
        np.array(q1_abs),
        arm_model,
        pin,
        pin_model,
        pin_data,
        backend,
    )


def _ridge_for_cols(
    free_cols: list[int], *, ridge_mass: float, ridge_com: float
) -> np.ndarray:
    """LBR-style: mass prior lighter than CoM (their obj uses ~10 vs ~100)."""
    lam = np.zeros(len(free_cols), dtype=np.float64)
    for k, col in enumerate(free_cols):
        if col == 28:
            lam[k] = ridge_mass * 2.5
            continue
        link = col // 4
        kind = col % 4
        # Proximal links: stronger CAD pull (weaker excitation).
        w_m = ridge_mass * (10.0 if link < 2 else 6.0 if link < 4 else 3.0)
        w_c = ridge_com * (8.0 if link < 2 else 5.0 if link < 4 else 2.5)
        lam[k] = w_m if kind == 0 else w_c
    return lam


def _regressor_cond(y_f: np.ndarray, lam: np.ndarray) -> float:
    """√(λ_max/λ_min) of YᵀY+Λ — excitation quality (LBR traj optimizes this)."""
    ata = y_f.T @ y_f + np.diag(lam)
    try:
        eig = np.linalg.eigvalsh(ata)
        eig = eig[eig > 1e-12]
        if len(eig) < 2:
            return float("inf")
        return float(np.sqrt(eig[-1] / eig[0]))
    except np.linalg.LinAlgError:
        return float("inf")


def _solve_weighted(
    y_f: np.ndarray, tau_adj: np.ndarray, w: np.ndarray, prior_f: np.ndarray, lam: np.ndarray
) -> np.ndarray:
    yw = y_f * w.reshape(-1, 1)
    tw = tau_adj * w
    ata = yw.T @ yw + np.diag(lam)
    atb = yw.T @ tw + lam * prior_f
    try:
        return np.linalg.solve(ata, atb)
    except np.linalg.LinAlgError:
        theta_f, _, _, _ = np.linalg.lstsq(ata, atb, rcond=None)
        return theta_f


def find_alive_cols(y_f: np.ndarray, free_cols: list[int]) -> tuple[list[int], list[int]]:
    """Drop only unexcited columns (near-zero ‖Y[:,j]‖).

    Soft-hold gravity data is too rank-deficient for LBR's aggressive QR freeze
    (that incorrectly pinned mass scales to CAD). Dead-column drop is the safe
    base-parameter hygiene for quasi-static ID.
    """
    if y_f.size == 0 or y_f.shape[1] == 0:
        return [], []
    col_norm = np.linalg.norm(y_f, axis=0)
    peak = float(col_norm.max()) if col_norm.size else 0.0
    floor = max(peak * 1e-8, 1e-12)
    alive: list[int] = []
    dead: list[int] = []
    for i, (n, col) in enumerate(zip(col_norm, free_cols)):
        # Always keep mass & EE even if weakly seen — ridge handles them.
        is_mass_or_ee = col == 28 or (col < 28 and col % 4 == 0)
        if is_mass_or_ee or n > floor:
            alive.append(i)
        else:
            dead.append(i)
    if not alive:
        return list(range(y_f.shape[1])), []
    return alive, dead


def _solve_free(
    y_big: np.ndarray,
    tau_big: np.ndarray,
    q1_abs: np.ndarray,
    prior: np.ndarray,
    free_cols: list[int],
    *,
    ridge_mass: float,
    ridge_com: float,
    wls_iters: int = 4,
    hard_sample_frac: float = 0.25,
) -> tuple[np.ndarray, float, int, int]:
    """Solve free columns with LBR WLS + hard-sample + dead-column drop.

    Globally frozen / dead columns stay at prior. Returns (theta, cond, n_alive, n_dead).
    """
    frozen = [c for c in range(29) if c not in free_cols]
    tau_adj = tau_big - y_big[:, frozen] @ prior[frozen]
    y_full = y_big[:, free_cols]
    prior_full = prior[free_cols]
    lam_full = _ridge_for_cols(free_cols, ridge_mass=ridge_mass, ridge_com=ridge_com)

    alive_loc, dead_loc = find_alive_cols(y_full, free_cols)
    if dead_loc:
        tau_adj = tau_adj - y_full[:, dead_loc] @ prior_full[dead_loc]
        y_f = y_full[:, alive_loc]
        prior_f = prior_full[alive_loc]
        lam = lam_full[alive_loc]
    else:
        y_f = y_full
        prior_f = prior_full
        lam = lam_full
        alive_loc = list(range(len(free_cols)))

    cond = _regressor_cond(y_f, lam)

    n_pose = max(1, y_big.shape[0] // 7)
    lat = np.ones(n_pose)
    for i, qa in enumerate(q1_abs):
        lat[i] = 2.5 if qa > 0.55 else 1.0

    w = np.ones(y_big.shape[0])
    for i in range(n_pose):
        w[i * 7 : (i + 1) * 7] = lat[i]
        w[i * 7] *= 1.5

    theta_b = _solve_weighted(y_f, tau_adj, w, prior_f, lam)

    # Iterative WLS (lbr regression.weighted_least_squares) + hard samples.
    for _ in range(max(0, wls_iters)):
        resid = (tau_adj - y_f @ theta_b).reshape(-1, 7)
        sigma = np.std(resid, axis=0) + 1e-3
        rnorm = np.linalg.norm(resid, axis=1)
        boost_imp = np.ones(n_pose)
        n_hard = max(8, int(round(n_pose * hard_sample_frac)))
        if n_pose >= 16:
            top = np.argsort(rnorm)[-n_hard:]
            boost_imp[top] = 2.5
        w = np.ones(y_big.shape[0])
        for i in range(n_pose):
            for j in range(7):
                w[i * 7 + j] = (lat[i] * boost_imp[i]) / sigma[j]
            w[i * 7] *= 1.4
        theta_b = _solve_weighted(y_f, tau_adj, w, prior_f, lam)

    theta_f = prior_full.copy()
    theta_f[alive_loc] = theta_b

    theta = prior.copy()
    theta[free_cols] = theta_f
    return theta, cond, len(alive_loc), len(dead_loc)


def _physical_project(
    scales: np.ndarray,
    com_delta: np.ndarray,
    ee_mass: float,
    m0: np.ndarray,
    *,
    scale_lo: float,
    scale_hi: float,
    com_lim: float,
) -> tuple[np.ndarray, np.ndarray, float]:
    """Soft physical consistency inspired by LBR build_ineq_physical_con.

    Quasi-static subset: m > floor, |Δcom| ≤ lim, and CoM components not
    simultaneously saturating all three axes (triangle-style sparsity).
    """
    scales = np.clip(scales, scale_lo, scale_hi)
    for i in range(7):
        # Prefer at least ~10% of CAD mass (LBR uses m>0.1 absolute).
        scales[i] = max(scales[i], max(scale_lo, 0.10))
        d = com_delta[i]
        norm = float(np.linalg.norm(d))
        if norm > com_lim:
            com_delta[i] = d * (com_lim / norm)
        # If two axes already near limit, shrink the third (cross-term sparsity).
        abs_d = np.abs(com_delta[i])
        near = abs_d > 0.85 * com_lim
        if int(near.sum()) >= 2:
            com_delta[i, near] *= 0.7
    ee_mass = float(np.clip(ee_mass, FINGER_MASS * 0.5, FINGER_MASS + 0.35))
    return scales, com_delta, ee_mass


def has_staged_traj(data: dict[str, np.ndarray]) -> bool:
    if "traj" not in data:
        return False
    traj = data["traj"]
    return bool(np.any((traj >= 100) & (traj < 400)))


def identify_side(
    data: dict[str, np.ndarray],
    side: str,
    urdf_path: str,
    *,
    speed_max: float = 0.12,
    max_samples: int = 300,
    ridge_mass: float = 50.0,
    ridge_com: float = 1200.0,
    scale_lo: float = 0.55,
    scale_hi: float = 1.9,
    com_lim: float = 0.04,
    sequential: bool | None = None,
) -> IdResult:
    if sequential is None:
        sequential = has_staged_traj(data)
    if sequential:
        return identify_side_sequential(
            data,
            side,
            urdf_path,
            speed_max=speed_max,
            max_samples=max_samples,
            ridge_mass=ridge_mass,
            ridge_com=ridge_com,
            scale_lo=scale_lo,
            scale_hi=scale_hi,
            com_lim=com_lim,
        )

    y_big, tau_big, q1_abs, arm_model, _, _, _, backend = _collect_quiet_regressors(
        data, side, urdf_path, speed_max=speed_max, max_samples=max_samples
    )
    m0 = arm_model.masses[1:8].copy()
    com0 = arm_model.coms[1:8].copy()
    prior = _cad_prior(m0, com0)
    free_cols = list(range(29))
    theta, cond, n_base, n_dep = _solve_free(
        y_big, tau_big, q1_abs, prior, free_cols, ridge_mass=ridge_mass, ridge_com=ridge_com
    )
    print(f"[{side}] batch WLS cond≈{cond:.1f} alive={n_base} dead={n_dep}")

    scales = np.ones(7)
    com_delta = np.zeros((7, 3))
    scales, com_delta, ee_mass = _clip_free_params(
        theta,
        list(range(7)),
        True,
        m0,
        com0,
        scales,
        com_delta,
        FINGER_MASS,
        scale_lo=scale_lo,
        scale_hi=scale_hi,
        com_lim=com_lim,
    )
    scales, com_delta, ee_mass = _physical_project(
        scales, com_delta, ee_mass, m0, scale_lo=scale_lo, scale_hi=scale_hi, com_lim=com_lim
    )
    theta_clip = _theta_from_scales(m0, com0, scales, com_delta, ee_mass)
    resid = tau_big - y_big @ theta_clip
    rms = float(np.sqrt(np.mean(resid**2)))
    n = y_big.shape[0] // 7

    return IdResult(
        mass_scales=scales,
        com_delta=com_delta,
        ee_mass=ee_mass,
        rms=rms,
        n=n,
        backend=backend + "+batch+wls",
        m0=m0,
    )


def identify_side_sequential(
    data: dict[str, np.ndarray],
    side: str,
    urdf_path: str,
    *,
    speed_max: float = 0.12,
    max_samples: int = 200,
    ridge_mass: float = 50.0,
    ridge_com: float = 1200.0,
    scale_lo: float = 0.55,
    scale_hi: float = 1.9,
    com_lim: float = 0.04,
) -> IdResult:
    """j7→j1 staged ID: wrist → elbow → shoulder; freeze distal after each stage."""
    arm_probe = load_arm_model(urdf_path, side)
    m0 = arm_probe.masses[1:8].copy()
    com0 = arm_probe.coms[1:8].copy()
    scales = np.ones(7)
    com_delta = np.zeros((7, 3))
    ee_mass = FINGER_MASS
    backend = "analytical"
    stage_rms: list[float] = []
    n_total = 0

    stages: list[tuple[str, tuple[int, int], list[int], bool]] = [
        ("wrist", STAGE_WRIST, [4, 5, 6], True),  # link5–7 + EE
        ("elbow", STAGE_ELBOW, [2, 3], False),  # link3–4
        ("shoulder", STAGE_SHOULDER, [0, 1], False),  # link1–2
    ]

    for name, (lo, hi), free_links, free_ee in stages:
        try:
            y_big, tau_big, q1_abs, arm_model, _, _, _, backend = _collect_quiet_regressors(
                data,
                side,
                urdf_path,
                speed_max=speed_max,
                max_samples=max_samples,
                traj_lo=lo,
                traj_hi=hi,
            )
        except SystemExit as exc:
            print(f"[{side}] 跳过阶段 {name}：{exc}")
            continue

        m0 = arm_model.masses[1:8].copy()
        com0 = arm_model.coms[1:8].copy()
        prior = _theta_from_scales(m0, com0, scales, com_delta, ee_mass)
        free_cols: list[int] = []
        for i in free_links:
            free_cols.extend([4 * i, 4 * i + 1, 4 * i + 2, 4 * i + 3])
        if free_ee:
            free_cols.append(28)

        # Distal links already identified: keep frozen (not in free_cols).
        # Proximal not yet identified stay at CAD via prior.
        theta, cond, n_base, n_dep = _solve_free(
            y_big,
            tau_big,
            q1_abs,
            prior,
            free_cols,
            ridge_mass=ridge_mass,
            ridge_com=ridge_com,
        )
        scales, com_delta, ee_mass = _clip_free_params(
            theta,
            free_links,
            free_ee,
            m0,
            com0,
            scales,
            com_delta,
            ee_mass,
            scale_lo=scale_lo,
            scale_hi=scale_hi,
            com_lim=com_lim,
        )
        scales, com_delta, ee_mass = _physical_project(
            scales, com_delta, ee_mass, m0, scale_lo=scale_lo, scale_hi=scale_hi, com_lim=com_lim
        )
        theta_clip = _theta_from_scales(m0, com0, scales, com_delta, ee_mass)
        resid = tau_big - y_big @ theta_clip
        rms = float(np.sqrt(np.mean(resid**2)))
        stage_rms.append(rms)
        n_total += y_big.shape[0] // 7
        print(
            f"[{side}] stage={name} traj={lo}-{hi} n={y_big.shape[0]//7} "
            f"rms={rms:.4f} Nm cond≈{cond:.1f} alive={n_base}/dead={n_dep} "
            f"free=link{[i+1 for i in free_links]}"
            + ("+ee" if free_ee else "")
        )

    if n_total == 0:
        print(f"[{side}] 分阶段样本不足，回退整批辨识")
        return identify_side(
            data,
            side,
            urdf_path,
            speed_max=speed_max,
            max_samples=max_samples,
            ridge_mass=ridge_mass,
            ridge_com=ridge_com,
            scale_lo=scale_lo,
            scale_hi=scale_hi,
            com_lim=com_lim,
            sequential=False,
        )

    # Final residual on all staged quiet samples.
    try:
        y_all, tau_all, _, _, _, _, _, _ = _collect_quiet_regressors(
            data,
            side,
            urdf_path,
            speed_max=speed_max,
            max_samples=max_samples * 2,
            traj_lo=100,
            traj_hi=399,
        )
        theta_clip = _theta_from_scales(m0, com0, scales, com_delta, ee_mass)
        rms = float(np.sqrt(np.mean((tau_all - y_all @ theta_clip) ** 2)))
    except SystemExit:
        rms = float(np.mean(stage_rms)) if stage_rms else float("nan")

    scales, com_delta, ee_mass = _physical_project(
        scales, com_delta, ee_mass, m0, scale_lo=scale_lo, scale_hi=scale_hi, com_lim=com_lim
    )
    return IdResult(
        mass_scales=scales,
        com_delta=com_delta,
        ee_mass=ee_mass,
        rms=rms,
        n=n_total,
        backend=backend + "+j7→j1+wls",
        m0=m0,
    )


def residual_report(
    data: dict[str, np.ndarray],
    side: str,
    urdf_path: str,
    scales: np.ndarray,
    com_delta: np.ndarray,
    ee_mass: float,
    *,
    speed_max: float = 0.12,
    max_samples: int = 400,
) -> dict[str, Any]:
    """Per-joint residual of identified model vs soft-hold target τ*."""
    y_big, tau_big, _, arm_model, _, _, _, backend = _collect_quiet_regressors(
        data,
        side,
        urdf_path,
        speed_max=speed_max,
        max_samples=max_samples,
        traj_lo=100 if has_staged_traj(data) else None,
        traj_hi=399 if has_staged_traj(data) else None,
    )
    m0 = arm_model.masses[1:8].copy()
    com0 = arm_model.coms[1:8].copy()
    theta = _theta_from_scales(m0, com0, scales, com_delta, ee_mass)
    resid = (tau_big - y_big @ theta).reshape(-1, 7)
    rms_j = np.sqrt(np.mean(resid**2, axis=0))
    mae_j = np.mean(np.abs(resid), axis=0)
    rms = float(np.sqrt(np.mean(resid**2)))
    return {
        "side": side,
        "backend": backend,
        "n": int(resid.shape[0]),
        "rms": rms,
        "rms_joint": [float(x) for x in rms_j],
        "mae_joint": [float(x) for x in mae_j],
    }


def com_delta_to_list(com_delta: np.ndarray) -> list[float]:
    return [float(v) for v in np.asarray(com_delta, dtype=np.float64).reshape(-1)]
