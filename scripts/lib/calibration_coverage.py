"""Whole-pose validation split, including an independent near-vertical hold."""
import numpy as np

VERTICAL_IDS = frozenset((350, 351, 352, 353))


def validation_mask(traj: np.ndarray) -> np.ndarray:
    mask = np.zeros(len(traj), dtype=bool)
    vertical = np.isin(traj, list(VERTICAL_IDS))
    for stage in (1, 2, 3):
        rows = np.flatnonzero((traj // 100 == stage) & ~vertical)
        mask[rows[1::3]] = True
    # Keep the vertical centre out of training; adjacent poses constrain the fit.
    mask[traj == 351] = True
    return mask


def require_vertical_pairs(data: dict, sides=("left", "right")) -> None:
    # Local import avoids a cycle: runtime fitting also uses validation_mask.
    from lib.runtime_gravity_fit import collect_holds
    for side in sides:
        present = {key[0] for key, held in collect_holds(data, side).items()
                   if 1 in held and -1 in held}
        missing = sorted(VERTICAL_IDS - present)
        if missing:
            raise ValueError(f"{side}: 垂直附近姿态缺少双向停稳数据 {missing}，保留原配置")
