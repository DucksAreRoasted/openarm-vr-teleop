# Source: unitreerobotics/televuer src/televuer/tv_wrapper.py
# Basis change from OpenXR (Y up) to the robot frame (Z up).
# The humanoid waist offset and the Unitree hand-frame 90° rotation are not
# applied: OpenArm's IK target is its own TCP, not a G1 waist or Unitree hand.

import numpy as np

T_ROBOT_OPENXR = np.array(
    [[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]], dtype=float
)

T_OPENXR_ROBOT = np.array(
    [[0, -1, 0, 0], [0, 0, 1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]], dtype=float
)

R_ROBOT_OPENXR = T_ROBOT_OPENXR[:3, :3].copy()
R_OPENXR_ROBOT = T_OPENXR_ROBOT[:3, :3].copy()


def openxr_pose_to_robot(pose):
    """Brobot = T_robot_openxr * Bxr * T_openxr_robot, as in TeleVuerWrapper."""
    pose = np.asarray(pose, dtype=float)
    return T_ROBOT_OPENXR @ pose @ T_OPENXR_ROBOT
